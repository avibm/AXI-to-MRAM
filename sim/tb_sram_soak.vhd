--------------------------------------------------------------------------------
-- tb_sram_soak.vhd  (simulation only)
--
-- Randomized soak test of the SRAM boot copy inside mram_top: G_ITER
-- reloads, each with a randomly chosen scenario, while a PCI-style master
-- keeps issuing random MRAM traffic on s_axi (64-byte reads of the master
-- image and of a data area, 64-byte writes to the data area, writes to the
-- protected master copy without the key, which must get SLVERR and change
-- nothing).
--
-- Scenarios (chosen at random per iteration):
--   0  plain reload
--   1  one corrupted byte in the SRAM at a random address  -> 1 retry, bit 8
--   2  one BRESP SLVERR                                    -> 1 retry, bit 9
--   3  one RRESP SLVERR                                    -> 1 retry, bit 10
--   4  all SRAM channels stalled for 0.5-3.5 x the watchdog limit -> 0 or 1
--      retry, only bits 12 (watchdog) and 13 (stuck, then automatic retry)
--   5  master image update: 64 random bytes written to the master copy
--      through PCI with key_ok = 1, then reload           -> SRAM = new image
--   6  aresetn at a random moment of the copy              -> clean boot
--   7  one wrong BID                                       -> 1 retry, bit 14
-- After every iteration: boot_done = 1, cpu_reset_n = 1, SRAM = the MRAM
-- master image (read from the MRAM model) byte for byte, the expected
-- boot_status, no outstanding AXI burst. Throughout: AXI master checker,
-- SRAM and MRAM model checks, every PCI read and write checked.
-- An aresetn in the middle of an MRAM access raises CS# mid-command; the
-- MRAM model reports that as a protocol error. Such errors within 10 cycles
-- of a deliberate reset (scenario 6) are counted and reported separately;
-- any other model error fails the test.
--
-- Run: sim/run_soak.sh [-gG_SEED=5 -gG_ITER=60 -gG_STALL_PCT=50]
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.mram_pkg.all;
use work.qspi_mram_mem_pkg.all;
use work.axi_sram_mem_pkg.all;

entity tb_sram_soak is
    generic (
        G_COPY      : natural := 4096;
        G_ITER      : natural := 40;
        G_SEED      : positive := 1;
        G_STALL_PCT : natural := 30;
        G_WATCHDOG  : natural := 4000
    );
end entity tb_sram_soak;

architecture sim of tb_sram_soak is

    constant T_CLK  : time    := 6.667 ns;
    constant SRC    : natural := 16#6000000#;
    constant DATA   : natural := 16#0010_0000#;  -- general MRAM data area used by PCI
    constant BB     : natural := C_BEAT_BYTES;
    constant MID_W  : natural := C_M_AXI_ID_WIDTH;
    constant SZB    : std_logic_vector(2 downto 0) := SIZE_BEAT;
    constant T_END  : time    := 5 ms + G_COPY * 500 ns;

    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';

    signal awid, arid, bid, rid : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0) := (others => '0');
    signal awaddr, araddr : std_logic_vector(31 downto 0) := (others => '0');
    signal awlen, arlen   : std_logic_vector(7 downto 0) := (others => '0');
    signal awvalid, awready, wvalid, wready, wlast, bvalid, bready : std_logic := '0';
    signal arvalid, arready, rvalid, rready, rlast : std_logic := '0';
    signal wdata, rdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal wstrb        : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0) := (others => '1');
    signal bresp, rresp : std_logic_vector(1 downto 0);
    signal key_ok       : std_logic := '0';

    signal m_awid, m_arid, m_bid, m_rid : std_logic_vector(MID_W - 1 downto 0);
    signal m_awaddr, m_araddr : std_logic_vector(31 downto 0);
    signal m_awlen, m_arlen   : std_logic_vector(7 downto 0);
    signal m_awsize, m_arsize : std_logic_vector(2 downto 0);
    signal m_awburst, m_arburst, m_bresp, m_rresp : std_logic_vector(1 downto 0);
    signal m_awvalid, m_awready, m_wvalid, m_wready, m_wlast, m_bvalid, m_bready : std_logic;
    signal m_arvalid, m_arready, m_rvalid, m_rready, m_rlast : std_logic;
    signal m_wdata, m_rdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal m_wstrb : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);

    signal cs_n, sclk : std_logic;
    signal io : std_logic_vector(3 downto 0);
    signal sram_reload : std_logic := '0';
    signal cpu_reset_n, boot_done, boot_fail, rd_slip : std_logic;
    signal boot_status : std_logic_vector(15 downto 0);
    signal model_errs : natural;

    signal stall_all : std_logic := '0';
    signal inj_b, inj_r, inj_c, inj_id : natural := 0;
    signal corrupt_addr : natural := 0;
    signal s_errs, chk_errs, resp_errs, aw_outst, ar_outst : natural;
    signal quiet : std_logic;

    -- PCI side: background traffic plus image updates on request
    signal img_req, img_ack : boolean := false;
    signal img_addr, img_seed : natural := 0;
    signal pci_on, pci_busy : boolean := false;
    signal pci_ops, pci_errs : natural := 0;
    signal static_errs : natural := 0;

    function pat(a : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((a * 7 + a / 256 + 3) mod 256, 8));
    end function;
    function pat2(a, seed : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((a * 13 + seed * 101 + a / 64) mod 256, 8));
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
            G_BOOT_COPY_SIZE => G_COPY, G_BOOT_WATCHDOG => G_WATCHDOG,
            G_POWERUP_CYCLES => 1500, G_M_ID => 3)
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
            m_axi_awsize => m_awsize, m_axi_awburst => m_awburst, m_axi_awlock => open,
            m_axi_awcache => open, m_axi_awprot => open,
            m_axi_awvalid => m_awvalid, m_axi_awready => m_awready,
            m_axi_wdata => m_wdata, m_axi_wstrb => m_wstrb, m_axi_wlast => m_wlast,
            m_axi_wvalid => m_wvalid, m_axi_wready => m_wready,
            m_axi_bid => m_bid, m_axi_bresp => m_bresp, m_axi_bvalid => m_bvalid, m_axi_bready => m_bready,
            m_axi_arid => m_arid, m_axi_araddr => m_araddr, m_axi_arlen => m_arlen,
            m_axi_arsize => m_arsize, m_axi_arburst => m_arburst, m_axi_arlock => open,
            m_axi_arcache => open, m_axi_arprot => open,
            m_axi_arvalid => m_arvalid, m_axi_arready => m_arready,
            m_axi_rid => m_rid, m_axi_rdata => m_rdata, m_axi_rresp => m_rresp, m_axi_rlast => m_rlast,
            m_axi_rvalid => m_rvalid, m_axi_rready => m_rready,
            key_ok => key_ok,
            cmd_wren => '0', cmd_wrdi => '0', cmd_rdsr => '0', cmd_wrsr => '0', cmd_rdid => '0',
            cmd_wrsr_data => x"00", cmd_rdsr_data => open, cmd_rdid_data => open, cmd_done => open,
            boot_hold => '0', sram_reload => sram_reload,
            mram_wr_pending => open, mram_rd_slip => rd_slip,
            cpu_reset_n => cpu_reset_n, boot_done => boot_done, boot_fail => boot_fail,
            boot_status => boot_status,
            mram_cs_n => cs_n, mram_sclk => sclk, mram_io => io);

    mdl : entity work.qspi_mram_model
        generic map (G_TPU => 5 us)
        port map (cs_n => cs_n, sclk => sclk, io => io, errors => model_errs);

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
            stall_all => stall_all, inj_b => inj_b, inj_r => inj_r, inj_c => inj_c,
            inj_id => inj_id, corrupt_addr => corrupt_addr, errors => s_errs);

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
            quiet => quiet, errors => chk_errs, resp_errors => resp_errs,
            aw_outst => aw_outst, ar_outst => ar_outst);

    process
    begin
        for a in 0 to G_COPY - 1 loop
            mem.write(SRC + a, pat(a));
        end loop;
        wait;
    end process;

    process (aclk)
        variable n : natural := 0;
    begin
        if rising_edge(aclk) and aresetn = '1' then
            if (cpu_reset_n = '1' and boot_done = '0') or (boot_done = '1' and boot_fail = '1')
               or rd_slip = '1'
               or (phase(boot_status) >= 3 and phase(boot_status) <= 5
                   and ((arvalid = '1' and arready = '1') or (awvalid = '1' and awready = '1'))) then
                n := n + 1;
                report "static check failed" severity error;
            end if;
            static_errs <= n;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- PCI master: random MRAM traffic, image updates on request
    ----------------------------------------------------------------------------
    p_pci : process
        variable s1, s2 : positive := G_SEED + 17;
        variable r      : real;
        variable id     : natural := 0;
        variable e, ops : natural := 0;
        variable nbeat  : natural := 64 / BB;

        impure function rnd(n : natural) return natural is
        begin
            uniform(s1, s2, r);
            return natural(floor(r * real(n))) mod n;
        end function;

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
            while b < nbeat loop
                rready <= '1' when rnd(4) /= 0 else '0';   -- random RREADY stalls
                wait until rising_edge(aclk);
                if rvalid = '1' and rready = '1' then
                    for i in 0 to BB - 1 loop
                        if rdata(8 * i + 7 downto 8 * i) /= mem.read(addr + b * BB + i) then
                            e := e + 1;
                            report "PCI read " & to_hstring(to_unsigned(addr + b * BB + i, 32))
                                   & " wrong" severity error;
                        end if;
                    end loop;
                    if rresp /= "00" or unsigned(rid) /= id or (rlast = '1') /= (b = nbeat - 1) then
                        e := e + 1;
                        report "PCI read: RRESP/RID/RLAST wrong" severity error;
                    end if;
                    b := b + 1;
                end if;
            end loop;
            rready <= '0';
        end procedure;

        -- 64-byte write; protected = expect SLVERR and no change
        procedure wr64(addr, seed : natural; to_guard : boolean) is
            variable old : std_logic_vector(7 downto 0);
            variable exp : std_logic_vector(1 downto 0);
            type snap_t is array (0 to 63) of std_logic_vector(7 downto 0);
            variable snap : snap_t;
        begin
            for i in 0 to 63 loop snap(i) := mem.read(addr + i); end loop;
            id      := (id + 1) mod 2 ** C_AXI_ID_WIDTH;
            awaddr  <= std_logic_vector(to_unsigned(addr, 32));
            awlen   <= std_logic_vector(to_unsigned(nbeat - 1, 8));
            awid    <= std_logic_vector(to_unsigned(id, C_AXI_ID_WIDTH));
            awvalid <= '1';
            loop wait until rising_edge(aclk); exit when awready = '1'; end loop;
            awvalid <= '0';
            for b in 0 to nbeat - 1 loop
                if rnd(3) = 0 then                      -- random WVALID gaps
                    wvalid <= '0';
                    for g in 1 to rnd(20) loop wait until rising_edge(aclk); end loop;
                end if;
                for i in 0 to BB - 1 loop
                    wdata(8 * i + 7 downto 8 * i) <= pat2(addr + b * BB + i, seed);
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
            if to_guard then exp := "10"; else exp := "00"; end if;
            if bresp /= exp or unsigned(bid) /= id then
                e := e + 1;
                report "PCI write " & to_hstring(to_unsigned(addr, 32)) & ": BRESP/BID wrong"
                       severity error;
            end if;
            for i in 1 to 1500 loop wait until rising_edge(aclk); end loop;  -- posted write lands
            for i in 0 to 63 loop
                if to_guard then
                    old := snap(i);
                else
                    old := pat2(addr + i, seed);
                end if;
                if mem.read(addr + i) /= old then
                    e := e + 1;
                    report "PCI write: MRAM byte " & to_hstring(to_unsigned(addr + i, 32))
                           & " wrong" severity error;
                end if;
            end loop;
        end procedure;

        variable k : natural;
    begin
        wait until aresetn = '1';
        loop
            if img_req /= img_ack then
                -- master image update, unlocked by key_ok
                key_ok <= '1';
                for i in 1 to 10 loop wait until rising_edge(aclk); end loop;
                wr64(img_addr, img_seed, false);
                key_ok <= '0';
                for i in 1 to 10 loop wait until rising_edge(aclk); end loop;
                img_ack <= img_req;
            elsif pci_on then
                pci_busy <= true;
                k := rnd(10);
                if k < 4 then
                    rd64(SRC + 64 * rnd(G_COPY / 64));
                elsif k < 6 then
                    rd64(DATA + 64 * rnd(64));
                elsif k < 9 then
                    wr64(DATA + 64 * rnd(64), rnd(1000), false);
                else
                    wr64(SRC + 64 * rnd(G_COPY / 64), rnd(1000), true);  -- no key: SLVERR
                end if;
                ops := ops + 1;
                pci_busy <= false;
            else
                wait until rising_edge(aclk);
            end if;
            pci_ops  <= ops;
            pci_errs <= e;
            -- in reset the s_axi port stalls; wait it out
            if aresetn = '0' then
                wait until aresetn = '1';
            end if;
        end loop;
    end process;

    ----------------------------------------------------------------------------
    -- Scenario loop
    ----------------------------------------------------------------------------
    p_main : process
        variable s1, s2 : positive := G_SEED;
        variable r : real;
        variable e : natural := 0;
        type cnt_t is array (0 to 7) of natural;
        variable cnt : cnt_t := (others => 0);
        variable sc, d : natural;
        variable exp_retries : natural;
        variable exp_bits    : std_logic_vector(14 downto 8);
        variable stall_case  : boolean;
        -- MRAM model errors caused by a deliberate aresetn in the middle of
        -- an MRAM access (CS# rises mid-command); reported, not failures
        variable reset_aborts, me0 : natural := 0;

        impure function rnd(n : natural) return natural is
        begin
            uniform(s1, s2, r);
            return natural(floor(r * real(n))) mod n;
        end function;

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

        procedure wait_end(what : string) is
        begin
            wait until boot_done = '1' or boot_fail = '1' for T_END;
            check(boot_done = '1', what & ": no boot_done within " & time'image(T_END)
                  & " (boot_fail=" & std_logic'image(boot_fail) & ")");
            clk(2);
        end procedure;

        procedure check_sram(what : string) is
            variable bad : natural := 0;
        begin
            for a in 0 to G_COPY - 1 loop
                if sram.read(a) /= mem.read(SRC + a) then
                    bad := bad + 1;
                end if;
            end loop;
            check(bad = 0, what & ": " & integer'image(bad) & " SRAM bytes differ from the MRAM image");
        end procedure;

        procedure reload is
        begin
            sram_reload <= '1';
            clk(10);
            sram_reload <= '0';
        end procedure;
    begin
        clk(20);
        aresetn <= '1';
        pci_on  <= true;
        wait_end("initial boot");
        check_sram("initial boot");

        for it in 1 to G_ITER loop
            sc := rnd(8);
            cnt(sc) := cnt(sc) + 1;
            exp_retries := 0;
            exp_bits    := (others => '0');
            stall_case  := false;
            clk(rnd(3000));                          -- random time after boot_done
            case sc is
                when 0 =>
                    reload;
                when 1 =>
                    corrupt_addr <= rnd(G_COPY);
                    inj_c <= inj_c + 1;
                    exp_retries := 1; exp_bits(8) := '1';
                    reload;
                when 2 =>
                    inj_b <= inj_b + 1;
                    exp_retries := 1; exp_bits(9) := '1';
                    reload;
                when 3 =>
                    inj_r <= inj_r + 1;
                    exp_retries := 1; exp_bits(10) := '1';
                    reload;
                when 4 =>
                    stall_case := true;
                    reload;
                    wait until phase(boot_status) = 3 or phase(boot_status) = 4 for 2 ms;
                    clk(rnd(500));
                    stall_all <= '1';
                    -- shorter than the watchdog (no retry), between watchdog and
                    -- watchdog + drain limit (drain -> retry), or longer (stuck,
                    -- then automatic retry when the stall ends)
                    clk(G_WATCHDOG / 2 + rnd(3 * G_WATCHDOG));
                    stall_all <= '0';
                when 5 =>
                    -- update 64 bytes of the master image through PCI (key_ok), then reload
                    img_addr <= SRC + 64 * rnd(G_COPY / 64);
                    img_seed <= rnd(1000) + 1000;
                    img_req  <= not img_req;
                    wait until img_ack = img_req for 5 ms;
                    check(img_ack = img_req, "image update did not complete");
                    reload;
                when 6 =>
                    -- a system reset also resets the PCI bridge: no PCI
                    -- transfer may span it in this testbench
                    pci_on <= false;
                    if pci_busy then wait until not pci_busy; end if;
                    reload;
                    wait until phase(boot_status) = 3 or phase(boot_status) = 4 for 2 ms;
                    clk(rnd(4000));
                    report "scenario 6: aresetn asserted (phase "
                           & integer'image(phase(boot_status)) & ")";
                    me0 := model_errs;
                    aresetn <= '0';
                    clk(5);
                    aresetn <= '1';
                    clk(5);
                    reset_aborts := reset_aborts + (model_errs - me0);
                    pci_on <= true;
                when others =>
                    inj_id <= inj_id + 1;
                    exp_retries := 1; exp_bits(14) := '1';
                    reload;
            end case;
            wait_end("iteration " & integer'image(it) & " scenario " & integer'image(sc));
            check(cpu_reset_n = '1' and phase(boot_status) = 0,
                  "iteration " & integer'image(it) & ": CPU not released / phase "
                  & integer'image(phase(boot_status)));
            if stall_case then
                check(to_integer(unsigned(boot_status(7 downto 4))) <= 1
                      and (boot_status(14 downto 8) and "1001111") = "0000000",
                      "iteration " & integer'image(it) & " (stall): status "
                      & to_hstring(boot_status));
            else
                check(to_integer(unsigned(boot_status(7 downto 4))) = exp_retries
                      and boot_status(14 downto 8) = exp_bits,
                      "iteration " & integer'image(it) & " scenario " & integer'image(sc)
                      & ": status " & to_hstring(boot_status));
            end if;
            check(aw_outst = 0 and ar_outst = 0, "iteration " & integer'image(it) & ": bursts outstanding");
            check_sram("iteration " & integer'image(it) & " scenario " & integer'image(sc));
        end loop;

        pci_on <= false;
        clk(20000);
        report "scenarios: plain " & integer'image(cnt(0)) & ", corrupt " & integer'image(cnt(1))
               & ", BRESP " & integer'image(cnt(2)) & ", RRESP " & integer'image(cnt(3))
               & ", stall " & integer'image(cnt(4)) & ", image update " & integer'image(cnt(5))
               & ", aresetn " & integer'image(cnt(6)) & ", wrong BID " & integer'image(cnt(7))
               & "; PCI operations " & integer'image(pci_ops);
        report "m_axi checker errors=" & integer'image(chk_errs) & ", SRAM model errors="
               & integer'image(s_errs) & ", MRAM model errors=" & integer'image(model_errs)
               & ", PCI errors=" & integer'image(pci_errs) & ", static=" & integer'image(static_errs);
        report "MRAM accesses aborted by a deliberate aresetn (model reports CS# high mid-command): "
               & integer'image(reset_aborts);
        e := e + chk_errs + s_errs + (model_errs - reset_aborts) + pci_errs + static_errs;
        if e = 0 then
            report "TEST PASSED";
        else
            report "TEST FAILED: " & integer'image(e) & " errors" severity failure;
        end if;
        std.env.finish;
    end process;

end architecture sim;
