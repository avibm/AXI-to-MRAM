--------------------------------------------------------------------------------
-- tb_mram_sram.vhd  (simulation only)
--
-- Self-checking testbench for the SRAM boot copy: mram_top with
-- qspi_mram_model on the MRAM pins, axi_sram_model on the boot copy's AXI
-- master port (m_axi_*) and axi_master_checker watching that port. The
-- copy size is reduced (G_COPY bytes) to keep the run short; the logic does
-- not depend on the size.
--
--   T0  boot_hold = 1: the copy waits after power-up, the MRAM AXI port
--       works, no SRAM traffic; then boot_hold is released while PCI reads
--       stream from the MRAM (all of them correct)
--   T1  boot: SRAM = MRAM master copy byte for byte, nothing written past
--       the end, cpu_reset_n only together with boot_done, boot_status
--       clean, no master traffic after boot_done (checker "quiet")
--   T2  MRAM AXI write + read back after boot
--   T3  sram_reload with PCI-style MRAM reads running back to back: the
--       CPU goes into reset at once, corrupted SRAM is restored, every PCI
--       read completes with correct data, a read issued during the copy
--       completes only after it
--   T4  one corrupted byte in the SRAM -> verify mismatch -> one retry
--   T5  one BRESP = SLVERR -> one retry
--   T6  one RRESP = SLVERR -> one retry
--   T7  permanent corruption -> boot_fail after 3 retries, CPU kept in
--       reset, MRAM AXI port open; reload after the fault is gone -> boot
--   T8  every SRAM channel stalled past the watchdog -> drain -> retry
--   T10 reload while a slow PCI write burst is half done: the copy waits
--       until the burst completes, then runs; the write data is correct
--   T11 sram_reload during a running copy is ignored
--   T12 watchdog in the verify pass -> drain -> retry
--   T13 wrong BID -> retry
--   T14 aresetn in the middle of a copy -> clean restart
--   T15 slave protocol violations (R with no AR, B with no write, RLAST on
--       the wrong beat, wrong RID) -> retry, error bit 14
--   T9  B withheld -> watchdog -> drain timeout -> stuck: boot_fail, no new
--       AXI traffic, reload ignored, MRAM AXI port open; when the B responses
--       come out, the copy leaves the stuck state by itself and retries
--   T9b stuck repeatedly until no retry is left -> failed state; reload boots
--   T9c B never returned -> stays stuck; aresetn recovers (run last)
--   Throughout: AXI protocol checker on m_axi_* (0 errors), SRAM model
--   errors 0, MRAM model protocol errors 0, mram_rd_slip 0.
--
-- Run: sim/run_sram.sh [-gG_STALL_PCT=0 -gG_SEED=3 ...]
-- Full 1MB boot (T0-T2 only): sim/run_sram.sh -gG_COPY=1048576 -gG_ONLY_BOOT=true
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;
use work.qspi_mram_mem_pkg.all;
use work.axi_sram_mem_pkg.all;

entity tb_mram_sram is
    generic (
        G_COPY      : natural := 16384;  -- bytes copied (1MB in the real design)
        G_STALL_PCT : natural := 30;     -- SRAM model stall probability, percent
        G_SEED      : natural := 1;
        G_WATCHDOG  : natural := 20_000;
        G_ONLY_BOOT : boolean := false   -- stop after T2 (for a full-size 1MB run)
    );
end entity tb_mram_sram;

architecture sim of tb_mram_sram is

    constant T_CLK    : time    := 6.667 ns;
    constant SRC      : natural := 16#7F00000#;
    constant BB       : natural := C_BEAT_BYTES;
    constant MID_W    : natural := C_M_AXI_ID_WIDTH;
    constant SZB      : std_logic_vector(2 downto 0) := SIZE_BEAT;
    constant T_END    : time := 5 ms + G_COPY * 500 ns;  -- longest boot or reload
    constant G_MAX_RETRIES_TB : natural := 3;             -- mram_top default G_BOOT_MAX_RETRIES

    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';

    -- s_axi (PCI / CPU side of the MRAM)
    signal awid, arid, bid, rid : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0) := (others => '0');
    signal awaddr, araddr : std_logic_vector(31 downto 0) := (others => '0');
    signal awlen, arlen   : std_logic_vector(7 downto 0) := (others => '0');
    signal awvalid, awready, wvalid, wready, wlast, bvalid, bready : std_logic := '0';
    signal arvalid, arready, rvalid, rready, rlast : std_logic := '0';
    signal wdata, rdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal wstrb        : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0) := (others => '1');
    signal bresp, rresp : std_logic_vector(1 downto 0);

    -- m_axi (boot copy -> SRAM)
    signal m_awid, m_arid, m_bid, m_rid : std_logic_vector(MID_W - 1 downto 0);
    signal m_awaddr, m_araddr : std_logic_vector(31 downto 0);
    signal m_awlen, m_arlen   : std_logic_vector(7 downto 0);
    signal m_awsize, m_arsize, m_awprot, m_arprot : std_logic_vector(2 downto 0);
    signal m_awburst, m_arburst, m_bresp, m_rresp : std_logic_vector(1 downto 0);
    signal m_awcache, m_arcache : std_logic_vector(3 downto 0);
    signal m_awlock, m_arlock : std_logic;
    signal m_awvalid, m_awready, m_wvalid, m_wready, m_wlast, m_bvalid, m_bready : std_logic;
    signal m_arvalid, m_arready, m_rvalid, m_rready, m_rlast : std_logic;
    signal m_wdata, m_rdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal m_wstrb : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);

    signal cs_n, sclk : std_logic;
    signal io : std_logic_vector(3 downto 0);
    signal boot_hold, sram_reload : std_logic := '0';
    signal cpu_reset_n, boot_done, boot_fail, rd_slip : std_logic;
    signal boot_status : std_logic_vector(15 downto 0);
    signal model_errs, model_rd, model_wr : natural;

    -- SRAM model controls
    signal stall_all, stop_b, corrupt_always : std_logic := '0';
    signal inj_b, inj_r, inj_c, inj_id : natural := 0;
    signal inj_xb, inj_xr, inj_rl, inj_rid : natural := 0;
    signal resp_errs : natural;
    signal corrupt_addr : natural := 0;
    signal s_naw, s_nar, s_errs : natural;
    signal chk_errs, aw_outst, ar_outst : natural;
    signal quiet : std_logic;

    -- PCI-side command interface (one process drives s_axi)
    type pci_op_t is (P_NONE, P_READ, P_WRITE, P_WRSLOW, P_STREAM);
    signal pci_op   : pci_op_t := P_NONE;
    signal pci_addr : natural := 0;
    signal pci_req, pci_ack : boolean := false;
    signal pci_stop : boolean := false;
    signal pci_reads, pci_errs : natural := 0;
    signal pci_aw_n, pci_b_n   : natural := 0;   -- PCI AW accepted / B received
    signal pci_done_when_booted : boolean := false;

    signal errs : natural := 0;

    function pat(a : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((a * 7 + a / 256 + 3) mod 256, 8));
    end function;
    function phase(st : std_logic_vector(15 downto 0)) return natural is
    begin
        return to_integer(unsigned(st(2 downto 0)));
    end function;

begin

    aclk  <= not aclk after T_CLK / 2;
    io    <= (others => 'H');
    quiet <= boot_done;

    dut : entity work.mram_top
        generic map (
            G_BOOT_COPY_SIZE => G_COPY,
            G_BOOT_WATCHDOG  => G_WATCHDOG,
            G_POWERUP_CYCLES => 1500,
            G_M_ID           => 5
        )
        port map (
            aclk => aclk, aresetn => aresetn,
            s_axi_awid => awid, s_axi_awaddr => awaddr, s_axi_awlen => awlen, s_axi_awsize => SZB,
            s_axi_awburst => "01", s_axi_awvalid => awvalid, s_axi_awready => awready,
            s_axi_wdata => wdata, s_axi_wstrb => wstrb, s_axi_wlast => wlast,
            s_axi_wvalid => wvalid, s_axi_wready => wready,
            s_axi_bid => bid, s_axi_bresp => bresp, s_axi_bvalid => bvalid, s_axi_bready => bready,
            s_axi_arid => arid, s_axi_araddr => araddr, s_axi_arlen => arlen, s_axi_arsize => SZB,
            s_axi_arburst => "01", s_axi_arvalid => arvalid, s_axi_arready => arready,
            s_axi_rid => rid, s_axi_rdata => rdata, s_axi_rresp => rresp, s_axi_rlast => rlast,
            s_axi_rvalid => rvalid, s_axi_rready => rready,
            m_axi_awid => m_awid, m_axi_awaddr => m_awaddr, m_axi_awlen => m_awlen,
            m_axi_awsize => m_awsize, m_axi_awburst => m_awburst, m_axi_awlock => m_awlock,
            m_axi_awcache => m_awcache, m_axi_awprot => m_awprot,
            m_axi_awvalid => m_awvalid, m_axi_awready => m_awready,
            m_axi_wdata => m_wdata, m_axi_wstrb => m_wstrb, m_axi_wlast => m_wlast,
            m_axi_wvalid => m_wvalid, m_axi_wready => m_wready,
            m_axi_bid => m_bid, m_axi_bresp => m_bresp, m_axi_bvalid => m_bvalid, m_axi_bready => m_bready,
            m_axi_arid => m_arid, m_axi_araddr => m_araddr, m_axi_arlen => m_arlen,
            m_axi_arsize => m_arsize, m_axi_arburst => m_arburst, m_axi_arlock => m_arlock,
            m_axi_arcache => m_arcache, m_axi_arprot => m_arprot,
            m_axi_arvalid => m_arvalid, m_axi_arready => m_arready,
            m_axi_rid => m_rid, m_axi_rdata => m_rdata, m_axi_rresp => m_rresp, m_axi_rlast => m_rlast,
            m_axi_rvalid => m_rvalid, m_axi_rready => m_rready,
            key_ok => '0',
            cmd_wren => '0', cmd_wrdi => '0', cmd_rdsr => '0', cmd_wrsr => '0', cmd_rdid => '0',
            cmd_wrsr_data => x"00", cmd_rdsr_data => open, cmd_rdid_data => open, cmd_done => open,
            boot_hold => boot_hold, sram_reload => sram_reload,
            mram_wr_pending => open, mram_rd_slip => rd_slip,
            cpu_reset_n => cpu_reset_n, boot_done => boot_done, boot_fail => boot_fail,
            boot_status => boot_status,
            mram_cs_n => cs_n, mram_sclk => sclk, mram_io => io
        );

    mdl : entity work.qspi_mram_model
        generic map (G_TPU => 5 us)
        port map (cs_n => cs_n, sclk => sclk, io => io, errors => model_errs,
                  n_rd => model_rd, n_wr => model_wr);

    sram_mdl : entity work.axi_sram_model
        generic map (G_ID_WIDTH => MID_W, G_DATA_W => C_AXI_DATA_WIDTH, G_SEED => G_SEED,
                     G_STALL_PCT => G_STALL_PCT)
        port map (
            aclk => aclk, aresetn => aresetn,
            s_axi_awid => m_awid, s_axi_awaddr => m_awaddr, s_axi_awlen => m_awlen,
            s_axi_awsize => m_awsize, s_axi_awburst => m_awburst,
            s_axi_awvalid => m_awvalid, s_axi_awready => m_awready,
            s_axi_wdata => m_wdata, s_axi_wstrb => m_wstrb, s_axi_wlast => m_wlast,
            s_axi_wvalid => m_wvalid, s_axi_wready => m_wready,
            s_axi_bid => m_bid, s_axi_bresp => m_bresp, s_axi_bvalid => m_bvalid, s_axi_bready => m_bready,
            s_axi_arid => m_arid, s_axi_araddr => m_araddr, s_axi_arlen => m_arlen,
            s_axi_arsize => m_arsize, s_axi_arburst => m_arburst,
            s_axi_arvalid => m_arvalid, s_axi_arready => m_arready,
            s_axi_rid => m_rid, s_axi_rdata => m_rdata, s_axi_rresp => m_rresp, s_axi_rlast => m_rlast,
            s_axi_rvalid => m_rvalid, s_axi_rready => m_rready,
            stall_all => stall_all, stop_b => stop_b, inj_b => inj_b, inj_r => inj_r, inj_c => inj_c,
            inj_id => inj_id, inj_xb => inj_xb, inj_xr => inj_xr, inj_rl => inj_rl,
            inj_rid => inj_rid,
            corrupt_addr => corrupt_addr, corrupt_always => corrupt_always,
            n_aw => s_naw, n_ar => s_nar, errors => s_errs);

    chk : entity work.axi_master_checker
        generic map (G_ID_WIDTH => MID_W, G_DATA_W => C_AXI_DATA_WIDTH, G_NAME => "m_axi")
        port map (
            aclk => aclk, aresetn => aresetn,
            awid => m_awid, awaddr => m_awaddr, awlen => m_awlen, awsize => m_awsize,
            awburst => m_awburst, awvalid => m_awvalid, awready => m_awready,
            wdata => m_wdata, wstrb => m_wstrb, wlast => m_wlast, wvalid => m_wvalid, wready => m_wready,
            bvalid => m_bvalid, bready => m_bready,
            arid => m_arid, araddr => m_araddr, arlen => m_arlen, arsize => m_arsize,
            arburst => m_arburst, arvalid => m_arvalid, arready => m_arready,
            rlast => m_rlast, rvalid => m_rvalid, rready => m_rready,
            quiet => quiet, errors => chk_errs, resp_errors => resp_errs, aw_outst => aw_outst, ar_outst => ar_outst);

    -- Preload the MRAM master copy; SRAM starts as zeros. The MRAM model
    -- stores 1MB per 32MB region (address bits 19:0), so G_COPY must not
    -- exceed 1MB or the image would alias onto itself.
    process
    begin
        assert G_COPY <= 1048576 and G_COPY mod 64 = 0
            report "G_COPY must be a multiple of 64 and at most 1MB (MRAM model size)"
            severity failure;
        for a in 0 to G_COPY - 1 loop
            mem.write(SRC + a, pat(a));
        end loop;
        wait;
    end process;

    -- Static checks that must hold at every cycle (each failure counts)
    process (aclk)
    begin
        if rising_edge(aclk) and aresetn = '1' then
            if (cpu_reset_n = '1' and boot_done = '0') or (boot_done = '1' and boot_fail = '1')
               or rd_slip = '1'
               or (phase(boot_status) >= 3 and phase(boot_status) <= 5
                   and ((arvalid = '1' and arready = '1') or (awvalid = '1' and awready = '1'))) then
                errs <= errs + 1;
            end if;
            assert not (cpu_reset_n = '1' and boot_done = '0')
                report "cpu_reset_n released without boot_done" severity error;
            assert not (boot_done = '1' and boot_fail = '1')
                report "boot_done and boot_fail both set" severity error;
            assert rd_slip /= '1' report "mram_rd_slip set" severity error;
            -- while the copy owns the MRAM (copy, verify, drain), the AXI
            -- side must accept no new burst
            if phase(boot_status) >= 3 and phase(boot_status) <= 5 then
                assert not (arvalid = '1' and arready = '1')
                    report "AR accepted on s_axi while the boot copy owns the MRAM" severity error;
                assert not (awvalid = '1' and awready = '1')
                    report "AW accepted on s_axi while the boot copy owns the MRAM" severity error;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- PCI-style AXI master on s_axi: 64-byte bursts to the MRAM, data checked
    -- against the MRAM model's backdoor.
    ----------------------------------------------------------------------------
    p_pci : process
        variable nbeat : natural := 64 / BB;
        variable id    : natural := 0;
        variable e     : natural := 0;
        variable a     : natural;

        procedure rd64(addr : natural) is
            variable b : natural := 0;
        begin
            id      := (id + 1) mod 2 ** C_AXI_ID_WIDTH;
            araddr  <= std_logic_vector(to_unsigned(addr, 32));
            arlen   <= std_logic_vector(to_unsigned(nbeat - 1, 8));
            arid    <= std_logic_vector(to_unsigned(id, C_AXI_ID_WIDTH));
            arvalid <= '1';
            loop wait until rising_edge(aclk); exit when arready = '1'; end loop;
            arvalid <= '0';
            rready  <= '1';
            while b < nbeat loop
                wait until rising_edge(aclk);
                if rvalid = '1' then
                    for i in 0 to BB - 1 loop
                        if rdata(8 * i + 7 downto 8 * i) /= mem.read(addr + b * BB + i) then
                            e := e + 1;
                            report "PCI read " & to_hstring(to_unsigned(addr + b * BB + i, 32))
                                   & " wrong" severity error;
                        end if;
                    end loop;
                    if rresp /= "00" or unsigned(rid) /= id
                       or (rlast = '1') /= (b = nbeat - 1) then
                        e := e + 1;
                        report "PCI read: RRESP/RID/RLAST wrong" severity error;
                    end if;
                    b := b + 1;
                end if;
            end loop;
            rready <= '0';
        end procedure;

        procedure wr64(addr : natural; gap : natural := 0) is
        begin
            id      := (id + 1) mod 2 ** C_AXI_ID_WIDTH;
            awaddr  <= std_logic_vector(to_unsigned(addr, 32));
            awlen   <= std_logic_vector(to_unsigned(nbeat - 1, 8));
            awid    <= std_logic_vector(to_unsigned(id, C_AXI_ID_WIDTH));
            awvalid <= '1';
            loop wait until rising_edge(aclk); exit when awready = '1'; end loop;
            awvalid <= '0';
            pci_aw_n <= pci_aw_n + 1;
            for b in 0 to nbeat - 1 loop
                -- a slow bridge: W beats with gaps (no WVALID in between)
                if gap > 0 then
                    wvalid <= '0';
                    for g in 1 to gap loop wait until rising_edge(aclk); end loop;
                end if;
                for i in 0 to BB - 1 loop
                    wdata(8 * i + 7 downto 8 * i) <= pat(addr + b * BB + i + 77);
                end loop;
                wlast  <= '1' when b = nbeat - 1 else '0';
                wvalid <= '1';
                loop wait until rising_edge(aclk); exit when wready = '1'; end loop;
            end loop;
            wvalid <= '0';
            wlast  <= '0';
            bready <= '1';
            loop wait until rising_edge(aclk); exit when bvalid = '1'; end loop;
            bready <= '0';
            pci_b_n <= pci_b_n + 1;
            if bresp /= "00" or unsigned(bid) /= id then
                e := e + 1;
                report "PCI write: BRESP/BID wrong" severity error;
            end if;
            -- let the posted write reach the MRAM, then compare the model
            for i in 1 to 2000 loop wait until rising_edge(aclk); end loop;
            for i in 0 to 63 loop
                if mem.read(addr + i) /= pat(addr + i + 77) then
                    e := e + 1;
                    report "PCI write: MRAM byte " & integer'image(addr + i) & " wrong" severity error;
                end if;
            end loop;
        end procedure;
    begin
        loop
            wait on pci_req;
            case pci_op is
                when P_READ  => rd64(pci_addr);
                when P_WRITE => wr64(pci_addr);
                when P_WRSLOW => wr64(pci_addr, 3000);
                when P_STREAM =>
                    a := 0;
                    while not pci_stop loop
                        rd64(SRC + a);
                        a := (a + 64) mod G_COPY;
                        pci_reads <= pci_reads + 1;
                    end loop;
                when P_NONE => null;
            end case;
            pci_errs <= e;
            pci_ack  <= pci_req;
        end loop;
    end process;

    ----------------------------------------------------------------------------
    -- Test sequence
    ----------------------------------------------------------------------------
    p_main : process
        variable e : natural := 0;
        variable t0 : time;
        variable n : natural;

        procedure check(c : boolean; msg : string) is
        begin
            if not c then
                e := e + 1;
                report "CHECK FAILED: " & msg severity error;
            end if;
        end procedure;

        procedure clk(cycles : natural) is
        begin
            for i in 1 to cycles loop wait until rising_edge(aclk); end loop;
        end procedure;

        procedure pci(op : pci_op_t; addr : natural) is
        begin
            pci_op   <= op;
            pci_addr <= addr;
            pci_req  <= not pci_req;
            wait until pci_ack = pci_req;
        end procedure;

        procedure wait_end(what : string) is
        begin
            -- up to 4 attempts x 2 MRAM passes at about 53 ns per byte
            wait until boot_done = '1' or boot_fail = '1' for T_END;
            check(boot_done = '1' or boot_fail = '1', what & ": no boot_done/boot_fail within "
                  & time'image(T_END));
            clk(2);
        end procedure;

        procedure check_sram(what : string) is
            variable bad : natural := 0;
        begin
            for a in 0 to G_COPY - 1 loop
                if sram.read(a) /= pat(a) then
                    bad := bad + 1;
                    if bad <= 4 then
                        report what & ": SRAM byte " & integer'image(a) & " = "
                               & to_hstring(sram.read(a)) & ", expected " & to_hstring(pat(a))
                               severity error;
                    end if;
                end if;
            end loop;
            check(bad = 0, what & ": " & integer'image(bad) & " SRAM bytes wrong");
            for a in G_COPY to G_COPY + 63 loop
                check(sram.read(a) = x"00", what & ": SRAM written past the copy at " & integer'image(a));
            end loop;
        end procedure;

        procedure check_ok(what : string; retries : natural; err_bits : std_logic_vector(14 downto 8)) is
        begin
            check(boot_done = '1' and boot_fail = '0' and cpu_reset_n = '1',
                  what & ": boot_done/boot_fail/cpu_reset_n wrong");
            check(phase(boot_status) = 0, what & ": phase " & integer'image(phase(boot_status)));
            check(to_integer(unsigned(boot_status(7 downto 4))) = retries,
                  what & ": retries " & integer'image(to_integer(unsigned(boot_status(7 downto 4))))
                  & ", expected " & integer'image(retries));
            check(boot_status(14 downto 8) = err_bits,
                  what & ": error bits " & to_string(boot_status(14 downto 8))
                  & ", expected " & to_string(err_bits));
            check(aw_outst = 0 and ar_outst = 0, what & ": AXI bursts left outstanding");
            check_sram(what);
        end procedure;

        procedure reload is
        begin
            sram_reload <= '1';
            clk(20);
            sram_reload <= '0';
        end procedure;

        procedure corrupt_sram is
        begin
            for a in 0 to G_COPY - 1 loop
                if a mod 97 = 0 then
                    sram.write(a, not pat(a));
                end if;
            end loop;
        end procedure;
    begin
        ------------------------------------------------------------------------
        -- T0: boot_hold
        ------------------------------------------------------------------------
        boot_hold <= '1';
        clk(20);
        aresetn <= '1';
        clk(3000);   -- past the 1500-cycle power-up
        check(phase(boot_status) = 1 and cpu_reset_n = '0' and boot_done = '0',
              "T0: not held after power-up");
        pci(P_READ, SRC + 128);
        pci(P_WRITE, 16#0010_0040#);
        check(s_naw = 0 and s_nar = 0, "T0: SRAM traffic while held");
        -- release boot_hold while PCI reads stream from the MRAM
        pci_stop <= false;
        pci_op   <= P_STREAM;
        pci_req  <= not pci_req;
        clk(2000);
        check(pci_reads > 0, "T0: PCI stream not running");
        report "T0 done (boot_hold)";
        boot_hold <= '0';

        ------------------------------------------------------------------------
        -- T1: boot
        ------------------------------------------------------------------------
        t0 := now;
        wait_end("T1");
        report "T1 boot took " & time'image(now - t0) & " for " & integer'image(G_COPY) & " bytes";
        check_ok("T1", 0, "0000000");
        pci_stop <= true;
        wait until pci_ack = pci_req;
        check(pci_errs = 0, "T1: PCI read errors around the boot_hold release");
        check(s_naw = G_COPY / 64 and s_nar = G_COPY / 64,
              "T1: " & integer'image(s_naw) & " AW / " & integer'image(s_nar) & " AR, expected "
              & integer'image(G_COPY / 64) & " each");

        ------------------------------------------------------------------------
        -- T2: MRAM AXI access after boot
        ------------------------------------------------------------------------
        pci(P_WRITE, 16#0010_0080#);
        pci(P_READ, 16#0010_0080#);
        pci(P_READ, SRC + 256);
        clk(200);
        check(s_naw = G_COPY / 64, "T2: SRAM traffic after boot_done");
        report "T2 done (MRAM access after boot)";

        if not G_ONLY_BOOT then

        ------------------------------------------------------------------------
        -- T3: reload while PCI reads stream from the MRAM
        ------------------------------------------------------------------------
        corrupt_sram;
        pci_stop <= false;
        pci_op   <= P_STREAM;
        pci_req  <= not pci_req;
        clk(3000);
        n := pci_reads;
        check(n > 0, "T3: PCI stream not running");
        reload;
        clk(4);
        check(cpu_reset_n = '0' and boot_done = '0', "T3: CPU not put into reset by sram_reload");
        wait until phase(boot_status) = 3 for 1 ms;
        check(phase(boot_status) = 3, "T3: copy did not start");
        wait_end("T3");
        check(pci_reads > n, "T3: PCI reads did not resume after the reload");
        pci_stop <= true;
        wait until pci_ack = pci_req;
        check(pci_errs = 0, "T3: PCI read errors");
        check_ok("T3", 0, "0000000");
        -- a read issued during the copy completes only after it
        reload;
        wait until phase(boot_status) = 3 for 1 ms;
        pci(P_READ, SRC + 512);
        check(boot_done = '1', "T3: MRAM read completed while the copy owned the MRAM");
        wait_end("T3b");
        check_ok("T3b", 0, "0000000");
        report "T3 done (reload with PCI traffic)";

        ------------------------------------------------------------------------
        -- T4..T6: single faults, one retry each
        ------------------------------------------------------------------------
        corrupt_addr <= 4096 + 13;
        inj_c <= 1;
        reload;
        wait_end("T4");
        check_ok("T4", 1, "0000001");
        report "T4 done (one corrupted byte -> retry)";

        inj_b <= 1;
        reload;
        wait_end("T5");
        check_ok("T5", 1, "0000010");
        report "T5 done (BRESP SLVERR -> retry)";

        inj_r <= 1;
        reload;
        wait_end("T6");
        check_ok("T6", 1, "0000100");
        report "T6 done (RRESP SLVERR -> retry)";

        ------------------------------------------------------------------------
        -- T7: permanent fault -> boot_fail; reload after the fault is gone
        ------------------------------------------------------------------------
        corrupt_addr   <= G_COPY - 1;
        corrupt_always <= '1';
        reload;
        wait_end("T7");
        check(boot_fail = '1' and boot_done = '0' and cpu_reset_n = '0', "T7: expected boot_fail");
        check(phase(boot_status) = 6, "T7: phase " & integer'image(phase(boot_status)));
        check(to_integer(unsigned(boot_status(7 downto 4))) = 3, "T7: expected 3 retries");
        check(boot_status(8) = '1', "T7: mismatch bit not set");
        pci(P_READ, SRC + 1024);       -- MRAM port open after boot_fail
        check(pci_errs = 0, "T7: PCI read after boot_fail");
        corrupt_always <= '0';
        reload;
        wait_end("T7b");
        check_ok("T7b", 0, "0000000");
        report "T7 done (boot_fail, then reload)";

        ------------------------------------------------------------------------
        -- T8: SRAM stalled past the watchdog, released during the drain
        ------------------------------------------------------------------------
        reload;
        wait until phase(boot_status) = 3 for 1 ms;
        clk(500);
        stall_all <= '1';
        wait until phase(boot_status) = 5 for 2 ms;
        check(phase(boot_status) = 5, "T8: watchdog did not fire");
        clk(G_WATCHDOG / 4);
        stall_all <= '0';
        wait_end("T8");
        check_ok("T8", 1, "0010000");
        report "T8 done (watchdog -> drain -> retry)";

        ------------------------------------------------------------------------
        -- T10: reload while a slow PCI write burst is half done: the copy
        -- waits (phase 2) until the burst has completed, then runs
        ------------------------------------------------------------------------
        n := pci_b_n;
        pci_op   <= P_WRSLOW;
        pci_addr <= 16#0010_0100#;
        pci_req  <= not pci_req;
        wait until pci_aw_n'event for 1 ms;
        clk(100);                       -- AW accepted, first W beats pending
        reload;
        clk(200);
        check(phase(boot_status) = 2, "T10: copy did not wait for the PCI write burst (phase "
              & integer'image(phase(boot_status)) & ")");
        check(pci_b_n = n, "T10: PCI write finished too early for the test");
        wait until phase(boot_status) = 3 for 2 ms;
        check(pci_b_n = n + 1, "T10: copy took the MRAM before the PCI write burst completed");
        wait_end("T10");
        check_ok("T10", 0, "0000000");
        wait until pci_ack = pci_req for 1 ms;
        check(pci_ack = pci_req and pci_errs = 0, "T10: PCI write data wrong or not finished");
        report "T10 done (reload waits for a slow PCI write burst)";

        ------------------------------------------------------------------------
        -- T11: sram_reload during a running copy is ignored (one copy only)
        ------------------------------------------------------------------------
        n := s_naw;
        reload;
        wait until phase(boot_status) = 3 for 1 ms;
        clk(500);
        reload;
        wait_end("T11");
        check_ok("T11", 0, "0000000");
        clk(5000);
        check(boot_done = '1' and phase(boot_status) = 0, "T11: a second copy started");
        check(s_naw = n + G_COPY / 64, "T11: " & integer'image(s_naw - n) & " AW, expected "
              & integer'image(G_COPY / 64));
        report "T11 done (reload ignored while copying)";

        ------------------------------------------------------------------------
        -- T12: watchdog in the verify pass
        ------------------------------------------------------------------------
        reload;
        wait until phase(boot_status) = 4 for 2 ms;
        check(phase(boot_status) = 4, "T12: verify pass not reached");
        clk(200);
        stall_all <= '1';
        wait until phase(boot_status) = 5 for 2 ms;
        check(phase(boot_status) = 5, "T12: watchdog did not fire in the verify pass");
        clk(G_WATCHDOG / 4);
        stall_all <= '0';
        wait_end("T12");
        check_ok("T12", 1, "0010000");
        report "T12 done (watchdog in the verify pass)";

        ------------------------------------------------------------------------
        -- T13: wrong BID -> retry
        ------------------------------------------------------------------------
        inj_id <= 1;
        reload;
        wait_end("T13");
        check_ok("T13", 1, "1000000");
        report "T13 done (wrong BID -> retry)";

        ------------------------------------------------------------------------
        -- T14: aresetn in the middle of a copy -> clean restart
        ------------------------------------------------------------------------
        reload;
        wait until phase(boot_status) = 3 for 1 ms;
        clk(1000);
        aresetn <= '0';
        clk(20);
        check(boot_done = '0' and cpu_reset_n = '0', "T14: reset did not clear boot_done");
        aresetn <= '1';
        wait_end("T14");
        check_ok("T14", 0, "0000000");
        report "T14 done (aresetn during a copy)";

        ------------------------------------------------------------------------
        -- T15: a slave that breaks the protocol: each fault -> retry, bit 14
        ------------------------------------------------------------------------
        reload;                                   -- R with no AR (copy pass)
        wait until phase(boot_status) = 3 for 1 ms;
        clk(300);
        inj_xr <= 1;
        wait_end("T15a");
        check_ok("T15a", 1, "1000000");
        reload;                                   -- B with no write (verify pass)
        wait until phase(boot_status) = 4 for 2 ms;
        clk(300);
        inj_xb <= 1;
        wait_end("T15b");
        check_ok("T15b", 1, "1000000");
        inj_rl <= 1;                              -- RLAST on the wrong beat
        reload;
        wait_end("T15c");
        check_ok("T15c", 1, "1000000");
        inj_rid <= 1;                             -- wrong RID
        reload;
        wait_end("T15d");
        check_ok("T15d", 1, "1000000");
        check(resp_errs = 3, "T15: checker saw " & integer'image(resp_errs)
              & " slave violations, expected 3");
        report "T15 done (slave protocol violations -> retry)";

        ------------------------------------------------------------------------
        -- T9: B withheld -> stuck; the fabric answers again -> automatic retry
        ------------------------------------------------------------------------
        stop_b <= '1';
        reload;
        wait until phase(boot_status) = 7 for 5 ms;
        check(phase(boot_status) = 7 and boot_fail = '1' and cpu_reset_n = '0',
              "T9: expected the stuck state");
        check(boot_status(12) = '1' and boot_status(13) = '1', "T9: timeout bits not set");
        n := s_naw;
        clk(5000);
        check(s_naw = n, "T9: new AW after the fabric was declared stuck");
        reload;                        -- must be ignored while stuck
        clk(2000);
        check(phase(boot_status) = 7 and s_naw = n, "T9: reload acted on in the stuck state");
        pci(P_READ, SRC + 2048);       -- MRAM side handed back
        check(pci_errs = 0, "T9: PCI read while stuck");
        stop_b <= '0';                 -- the withheld B responses come out
        wait until phase(boot_status) /= 7 for 1 ms;
        check(boot_fail = '0', "T9: boot_fail not cleared when leaving the stuck state");
        wait_end("T9");
        check_ok("T9", 1, "0110000");
        report "T9 done (stuck fabric answers again -> automatic retry)";

        ------------------------------------------------------------------------
        -- T9b: stuck again and again until no retry is left -> failed state;
        -- then a reload boots
        ------------------------------------------------------------------------
        stop_b <= '1';
        reload;
        for k in 1 to G_MAX_RETRIES_TB + 1 loop
            wait until phase(boot_status) = 7 for 5 ms;
            check(phase(boot_status) = 7, "T9b: not stuck, round " & integer'image(k));
            stop_b <= '0';
            wait until phase(boot_status) /= 7 for 1 ms;
            clk(50);                   -- the retry has taken the MRAM again
            stop_b <= '1';
        end loop;
        clk(G_WATCHDOG);
        check(phase(boot_status) = 6 and boot_fail = '1' and cpu_reset_n = '0',
              "T9b: expected the failed state, phase " & integer'image(phase(boot_status)));
        check(to_integer(unsigned(boot_status(7 downto 4))) = G_MAX_RETRIES_TB,
              "T9b: retries " & integer'image(to_integer(unsigned(boot_status(7 downto 4)))));
        pci(P_READ, SRC + 3072);       -- MRAM port open after the failure
        stop_b <= '0';
        reload;
        wait_end("T9b");
        check_ok("T9b", 0, "0000000");
        report "T9b done (stuck repeatedly -> failed; reload boots)";

        ------------------------------------------------------------------------
        -- T9c: B never returned -> stuck for good; aresetn recovers
        ------------------------------------------------------------------------
        stop_b <= '1';
        reload;
        wait until phase(boot_status) = 7 for 5 ms;
        check(phase(boot_status) = 7, "T9c: expected the stuck state");
        clk(4 * G_WATCHDOG);
        check(phase(boot_status) = 7 and boot_fail = '1', "T9c: left the stuck state without B");
        stop_b  <= '0';
        aresetn <= '0';
        clk(20);
        for a in 0 to G_COPY - 1 loop sram.write(a, x"00"); end loop;
        aresetn <= '1';
        wait_end("T9c");
        check_ok("T9c", 0, "0000000");
        report "T9c done (stuck for good, recovered by aresetn)";
        end if;

        ------------------------------------------------------------------------
        e := e + errs;
        report "m_axi checker errors=" & integer'image(chk_errs) & ", SRAM model errors="
               & integer'image(s_errs) & ", MRAM model errors=" & integer'image(model_errs)
               & ", PCI errors=" & integer'image(pci_errs);
        e := e + chk_errs + s_errs + model_errs + pci_errs;
        if e = 0 then
            report "TEST PASSED";
        else
            report "TEST FAILED: " & integer'image(e) & " errors" severity failure;
        end if;
        std.env.finish;
    end process;

end architecture sim;
