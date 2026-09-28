--------------------------------------------------------------------------------
-- tb_mram_top.vhd  (simulation only)
--
-- Self-checking testbench for mram_top against qspi_mram_model. Uses a
-- 512-byte boot image so the boot copy finishes quickly. Every write is
-- checked byte-by-byte through the model's backdoor (including bytes that
-- must NOT change), and every read is compared with the backdoor.
--
-- Covered: boot copy + verify, an AXI read issued during the boot copy
-- (must stall, then return correct data), 64-byte / narrow / unaligned /
-- sparse-strobe writes, INCR bursts, write-guard blocking and unlocking,
-- FIXED-burst rejection, reads concurrent with writes, randomized
-- BREADY/RREADY back-pressure, exactly one B / RLAST per transaction, and
-- AXI addresses with upper (window base) bits set.
--
-- MRAM register commands from a 33MHz "PCI" clock domain: RDID, RDSR,
-- WREN, WRDI, WRSR with the four-phase cmd_*/cmd_done handshake, while
-- boot_hold keeps the boot copy waiting, and again later concurrently with
-- AXI traffic. Also reproduces "a write does not apply": with the status
-- register block-protect bits set, an AXI write gets OKAY but the MRAM
-- content does not change.
--
-- Run: see sim/run_ghdl.sh
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;
use work.qspi_mram_mem_pkg.all;

entity tb_mram_top is
end entity tb_mram_top;

architecture sim of tb_mram_top is

    constant T_CLK      : time    := 6.667 ns; -- 150MHz
    constant SRC_BASE   : natural := 16#6000000#;
    constant BOOT_BYTES : natural := 512;

    subtype beat_t is std_logic_vector(511 downto 0);
    subtype strb_t is std_logic_vector(63 downto 0);
    type beat_arr_t is array (0 to 15) of beat_t;
    type strb_arr_t is array (0 to 15) of strb_t;

    signal aclk    : std_logic := '0';
    signal aresetn : std_logic := '0';

    signal awid    : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0) := (others => '0');
    signal awaddr  : std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal awlen   : std_logic_vector(7 downto 0) := (others => '0');
    signal awsize  : std_logic_vector(2 downto 0) := "110";
    signal awburst : std_logic_vector(1 downto 0) := "01";
    signal awvalid, awready : std_logic := '0';
    signal wdata   : beat_t := (others => '0');
    signal wstrb   : strb_t := (others => '0');
    signal wlast, wvalid, wready : std_logic := '0';
    signal bid     : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0);
    signal bresp   : std_logic_vector(1 downto 0);
    signal bvalid  : std_logic;
    signal bready  : std_logic := '0';
    signal arid    : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0) := (others => '0');
    signal araddr  : std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal arlen   : std_logic_vector(7 downto 0) := (others => '0');
    signal arsize  : std_logic_vector(2 downto 0) := "110";
    signal arburst : std_logic_vector(1 downto 0) := "01";
    signal arvalid, arready : std_logic := '0';
    signal rid     : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0);
    signal rdata   : beat_t;
    signal rresp   : std_logic_vector(1 downto 0);
    signal rlast, rvalid : std_logic;
    signal rready  : std_logic := '0';

    signal key_ok      : std_logic := '0';

    constant T_PCI      : time := 30 ns;  -- 33MHz
    constant DEVICE_ID  : std_logic_vector(31 downto 0) := x"E6212801";
    constant I_WREN : natural := 0;
    constant I_WRDI : natural := 1;
    constant I_RDSR : natural := 2;
    constant I_WRSR : natural := 3;
    constant I_RDID : natural := 4;
    signal pci_clk       : std_logic := '0';
    signal cmd_req       : std_logic_vector(4 downto 0) := (others => '0');
    signal cmd_wrsr_data : std_logic_vector(7 downto 0) := (others => '0');
    signal cmd_rdsr_data : std_logic_vector(7 downto 0);
    signal cmd_rdid_data : std_logic_vector(31 downto 0);
    signal cmd_done      : std_logic;
    signal boot_hold     : std_logic := '1';
    signal pci_step      : natural := 0;
    signal wr_paused     : boolean := false; -- writer idle, PCI may change WREN/SR
    signal wr_t14_done   : boolean := false;
    signal pci_finish    : boolean := false;
    signal tb_errs_p     : natural := 0;
    signal cpu_reset_n : std_logic;
    signal boot_done   : std_logic;
    signal boot_fail   : std_logic;

    signal cs_n, sclk : std_logic;
    signal io         : std_logic_vector(3 downto 0);
    signal model_errs, model_rd, model_wr : natural;

    -- test sequencing between the writer and reader processes
    signal wr_step   : natural := 0;
    signal rd_step   : natural := 0;
    signal wr_finish : boolean := false;
    signal rd_finish : boolean := false;
    signal tb_errs_w : natural := 0;
    signal tb_errs_r : natural := 0;

    -- handshake counters from the monitor
    signal n_b, n_rlast : natural := 0;

    function pattern(a : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((a * 7 + (a / 256) * 13 + 16#5A#) mod 256, 8));
    end function;

    function mk_beat(seed : natural) return beat_t is
        variable b : beat_t;
    begin
        for i in 0 to 63 loop
            b(8 * i + 7 downto 8 * i) := std_logic_vector(to_unsigned((seed * 31 + i * 5 + 1) mod 256, 8));
        end loop;
        return b;
    end function;

    function size_bytes(sz : natural) return natural is
    begin
        return 2 ** sz;
    end function;

    -- AXI INCR beat address
    function beat_addr(start, sz, beat : natural) return natural is
        variable n : natural := size_bytes(sz);
    begin
        if beat = 0 then
            return start;
        end if;
        return (start / n) * n + beat * n;
    end function;

begin

    aclk    <= not aclk after T_CLK / 2;
    pci_clk <= not pci_clk after T_PCI / 2;
    aresetn <= '1' after 20 * T_CLK;
    io      <= (others => 'H'); -- board pull-ups

    dut : entity work.mram_top
        generic map (
            G_BOOT_COPY_SIZE => BOOT_BYTES,
            G_BOOT_WATCHDOG  => 100_000,
            G_POWERUP_CYCLES => 1500      -- 10us instead of 25ms, to keep the run short
        )
        port map (
            aclk => aclk, aresetn => aresetn,
            s_axi_awid => awid, s_axi_awaddr => awaddr, s_axi_awlen => awlen,
            s_axi_awsize => awsize, s_axi_awburst => awburst,
            s_axi_awvalid => awvalid, s_axi_awready => awready,
            s_axi_wdata => wdata, s_axi_wstrb => wstrb, s_axi_wlast => wlast,
            s_axi_wvalid => wvalid, s_axi_wready => wready,
            s_axi_bid => bid, s_axi_bresp => bresp, s_axi_bvalid => bvalid, s_axi_bready => bready,
            s_axi_arid => arid, s_axi_araddr => araddr, s_axi_arlen => arlen,
            s_axi_arsize => arsize, s_axi_arburst => arburst,
            s_axi_arvalid => arvalid, s_axi_arready => arready,
            s_axi_rid => rid, s_axi_rdata => rdata, s_axi_rresp => rresp,
            s_axi_rlast => rlast, s_axi_rvalid => rvalid, s_axi_rready => rready,
            pci_clk => pci_clk,
            cmd_wren => cmd_req(I_WREN), cmd_wrdi => cmd_req(I_WRDI),
            cmd_rdsr => cmd_req(I_RDSR), cmd_wrsr => cmd_req(I_WRSR),
            cmd_rdid => cmd_req(I_RDID), cmd_wrsr_data => cmd_wrsr_data,
            cmd_rdsr_data => cmd_rdsr_data, cmd_rdid_data => cmd_rdid_data,
            cmd_done => cmd_done, boot_hold => boot_hold,
            key_ok => key_ok, cpu_reset_n => cpu_reset_n, boot_done => boot_done,
            boot_fail => boot_fail,
            mram_cs_n => cs_n, mram_sclk => sclk, mram_io => io
        );

    mdl : entity work.qspi_mram_model
        generic map (G_TPU => 10 us, G_DEVICE_ID => DEVICE_ID)
        port map (cs_n => cs_n, sclk => sclk, io => io,
                  errors => model_errs, n_rd => model_rd, n_wr => model_wr);

    -- Preload the master copy region before reset is released.
    process
    begin
        for a in 0 to 16383 loop
            mem.write(SRC_BASE + a, pattern(SRC_BASE + a));
            mem.write(a, x"EE"); -- stale working copy / general RAM
        end loop;
        wait;
    end process;

    -- Handshake monitor
    process (aclk)
    begin
        if rising_edge(aclk) then
            if bvalid = '1' and bready = '1' then
                n_b <= n_b + 1;
            end if;
            if rvalid = '1' and rready = '1' and rlast = '1' then
                n_rlast <= n_rlast + 1;
            end if;
        end if;
    end process;

    ----------------------------------------------------------------------------
    -- Writer: owns AW / W / B
    ----------------------------------------------------------------------------
    p_wr : process
        variable errs : natural := 0;
        variable lfsr : unsigned(15 downto 0) := x"ACE1";
        variable writes_done : natural := 0;
        type snap_t is array (0 to 17 * 64 - 1) of std_logic_vector(7 downto 0);
        variable snap : snap_t;

        procedure check(cond : boolean; msg : string) is
        begin
            if not cond then
                errs := errs + 1;
                tb_errs_w <= errs;
                report "WRITER: " & msg severity error;
            end if;
        end procedure;

        impure function rnd(maxv : natural) return natural is
        begin
            lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
            return to_integer(lfsr(7 downto 0)) mod (maxv + 1);
        end function;

        procedure axi_write(addr, len, sz : natural; burst : std_logic_vector(1 downto 0);
                            beats : beat_arr_t; strbs : strb_arr_t;
                            exp_resp : std_logic_vector(1 downto 0); expect_change : boolean) is
            variable wi     : natural := 0;
            variable aw_ok  : boolean := false;
            variable base   : natural := (addr / 64) * 64;
            variable ba, lo, hi, a : natural;
            variable exp    : std_logic_vector(7 downto 0);
        begin
            for i in snap'range loop
                snap(i) := mem.read(base + i);
            end loop;

            awaddr  <= std_logic_vector(to_unsigned(addr, awaddr'length));
            awlen   <= std_logic_vector(to_unsigned(len, 8));
            awsize  <= std_logic_vector(to_unsigned(sz, 3));
            awburst <= burst;
            awid    <= std_logic_vector(to_unsigned(len + 3, awid'length));
            awvalid <= '1';
            wdata   <= beats(0);
            wstrb   <= strbs(0);
            wlast   <= '1' when len = 0 else '0';
            wvalid  <= '1';
            loop
                wait until rising_edge(aclk);
                if awvalid = '1' and awready = '1' then
                    awvalid <= '0';
                    aw_ok   := true;
                end if;
                if wvalid = '1' and wready = '1' then
                    wi := wi + 1;
                    if wi > len then
                        wvalid <= '0';
                        wlast  <= '0';
                    else
                        wdata <= beats(wi);
                        wstrb <= strbs(wi);
                        wlast <= '1' when wi = len else '0';
                    end if;
                end if;
                exit when aw_ok and wi > len;
            end loop;

            -- B channel with random back-pressure; keep BREADY high a few
            -- cycles after the handshake to catch duplicate responses.
            for i in 1 to rnd(6) loop
                wait until rising_edge(aclk);
            end loop;
            bready <= '1';
            loop
                wait until rising_edge(aclk);
                exit when bvalid = '1' and bready = '1';
            end loop;
            check(bresp = exp_resp, "BRESP " & to_string(bresp) & " expected " & to_string(exp_resp)
                  & " at 0x" & to_hstring(to_unsigned(addr, 32)));
            check(bid = std_logic_vector(to_unsigned(len + 3, bid'length)), "BID mismatch");
            for i in 1 to 4 loop
                wait until rising_edge(aclk);
            end loop;
            bready <= '0';
            writes_done := writes_done + 1;
            check(n_b = writes_done, "B handshakes: saw " & integer'image(n_b)
                  & ", expected " & integer'image(writes_done));

            -- Byte-exact check of every byte the burst could have touched.
            for b in 0 to len loop
                ba := beat_addr(addr, sz, b);
                lo := ba mod 64;
                hi := ((ba mod 64) / size_bytes(sz)) * size_bytes(sz) + size_bytes(sz);
                for lane in 0 to 63 loop
                    a := (ba / 64) * 64 + lane;
                    if expect_change and lane >= lo and lane < hi and strbs(b)(lane) = '1' then
                        exp := beats(b)(8 * lane + 7 downto 8 * lane);
                    else
                        exp := snap(a - base);
                    end if;
                    -- a later beat may legitimately overwrite an earlier one
                    -- only in FIXED bursts, which are rejected here anyway
                    if lane >= lo and lane < hi then
                        check(mem.read(a) = exp, "byte 0x" & to_hstring(to_unsigned(a, 32))
                              & " = " & to_hstring(mem.read(a)) & ", expected " & to_hstring(exp));
                    end if;
                end loop;
                -- bytes just outside the beat's container must be untouched
                if lo > 0 and ((ba / 64) * 64 + lo - 1) >= base then
                    a := (ba / 64) * 64 + lo - 1;
                    if b = 0 then
                        check(mem.read(a) = snap(a - base), "byte below first beat changed");
                    end if;
                end if;
            end loop;
        end procedure;

        variable beats : beat_arr_t;
        variable strbs : strb_arr_t;
    begin
        wait until boot_done = '1';
        wait until rising_edge(aclk);
        report "boot_done at " & time'image(now);
        check(cpu_reset_n = '1', "cpu_reset_n not released");
        check(boot_fail = '0', "boot_fail set");
        for a in 0 to BOOT_BYTES - 1 loop
            check(mem.read(a) = pattern(SRC_BASE + a), "working copy byte " & integer'image(a) & " wrong");
        end loop;
        check(mem.read(BOOT_BYTES) = x"EE", "boot copy wrote past its end");
        wr_step <= 1;

        -- T3: full 64-byte write
        beats(0) := mk_beat(1); strbs(0) := (others => '1');
        axi_write(16#1000#, 0, 6, "01", beats, strbs, "00", true);

        -- T4: narrow 4-byte write at an offset inside the line
        beats(0) := mk_beat(2); strbs(0) := (others => '0'); strbs(0)(7 downto 4) := "1111";
        axi_write(16#2004#, 0, 2, "01", beats, strbs, "00", true);

        -- T5: sparse, non-contiguous strobes on a full beat
        beats(0) := mk_beat(3);
        strbs(0) := x"F0F0_8001_0F0F_1234";
        axi_write(16#3000#, 0, 6, "01", beats, strbs, "00", true);

        -- T5b: all strobes off -> nothing written, still OKAY
        beats(0) := mk_beat(9); strbs(0) := (others => '0');
        axi_write(16#3040#, 0, 6, "01", beats, strbs, "00", true);

        -- T6: 4-beat 64-byte INCR burst
        for b in 0 to 3 loop
            beats(b) := mk_beat(10 + b); strbs(b) := (others => '1');
        end loop;
        axi_write(16#4000#, 3, 6, "01", beats, strbs, "00", true);

        -- T7: narrow (4-byte) INCR burst starting unaligned at 0x5002
        for b in 0 to 3 loop
            beats(b) := mk_beat(20 + b); strbs(b) := (others => '1');
        end loop;
        axi_write(16#5002#, 3, 2, "01", beats, strbs, "00", true);

        -- T7b: 8-byte burst crossing a 64-byte line
        for b in 0 to 3 loop
            beats(b) := mk_beat(30 + b); strbs(b) := (others => '1');
        end loop;
        axi_write(16#5030#, 3, 3, "01", beats, strbs, "00", true);
        wr_step <= 2;

        -- T8: protected region, locked then unlocked
        beats(0) := mk_beat(40); strbs(0) := (others => '1');
        key_ok <= '0';
        axi_write(SRC_BASE + 16#100#, 0, 6, "01", beats, strbs, "10", false);
        key_ok <= '1';
        for i in 1 to 4 loop wait until rising_edge(aclk); end loop; -- synchronizer
        axi_write(SRC_BASE + 16#100#, 0, 6, "01", beats, strbs, "00", true);
        key_ok <= '0';
        for i in 1 to 4 loop wait until rising_edge(aclk); end loop;
        beats(0) := mk_beat(41);
        axi_write(SRC_BASE + 16#140#, 0, 6, "01", beats, strbs, "10", false);

        -- T11: FIXED burst rejected (all beats drained), next write still fine
        for b in 0 to 1 loop
            beats(b) := mk_beat(50 + b); strbs(b) := (others => '1');
        end loop;
        axi_write(16#1100#, 1, 6, "00", beats, strbs, "10", false);
        beats(0) := mk_beat(52); strbs(0) := (others => '1');
        axi_write(16#1100#, 0, 6, "01", beats, strbs, "00", true);

        -- T13: upper AXI address bits (window base) must be ignored
        beats(0) := mk_beat(55); strbs(0) := (others => '1');
        axi_write(16#2000_1200#, 0, 6, "01", beats, strbs, "00", true);

        -- T14: block-protect bits set over PCI -> write is accepted (OKAY)
        -- by AXI but does not change the MRAM ("write doesn't apply")
        -- AXI writes clear the device's WREN bit, so the PCI WREN -> WRSR
        -- sequence must run while no AXI write is in flight.
        wr_paused <= true;
        if pci_step < 1 then wait until pci_step >= 1; end if;
        wr_paused <= false;
        beats(0) := mk_beat(56); strbs(0) := (others => '1');
        axi_write(16#3900#, 0, 6, "01", beats, strbs, "00", false);
        wr_t14_done <= true;
        if pci_step < 2 then wait until pci_step >= 2; end if; -- unprotected again
        wr_step <= 3;

        -- T10: writes while the reader is streaming reads elsewhere
        for i in 0 to 7 loop
            beats(0) := mk_beat(60 + i); strbs(0) := (others => '1');
            strbs(0)(i) := '0';
            axi_write(16#3800# + 64 * i, 0, 6, "01", beats, strbs, "00", true);
        end loop;
        wr_step <= 4;

        report "writer done, " & integer'image(writes_done) & " writes, errors=" & integer'image(errs);
        wr_finish <= true;
        wait;
    end process;

    ----------------------------------------------------------------------------
    -- Reader: owns AR / R
    ----------------------------------------------------------------------------
    p_rd : process
        variable errs  : natural := 0;
        variable lfsr  : unsigned(15 downto 0) := x"1D0F";
        variable reads_done : natural := 0;

        procedure check(cond : boolean; msg : string) is
        begin
            if not cond then
                errs := errs + 1;
                tb_errs_r <= errs;
                report "READER: " & msg severity error;
            end if;
        end procedure;

        impure function rnd(maxv : natural) return natural is
        begin
            lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
            return to_integer(lfsr(7 downto 0)) mod (maxv + 1);
        end function;

        procedure axi_read(addr, len, sz : natural; burst : std_logic_vector(1 downto 0);
                           exp_resp : std_logic_vector(1 downto 0)) is
            variable beat : natural := 0;
            variable ba, lo, hi, a : natural;
        begin
            araddr  <= std_logic_vector(to_unsigned(addr, araddr'length));
            arlen   <= std_logic_vector(to_unsigned(len, 8));
            arsize  <= std_logic_vector(to_unsigned(sz, 3));
            arburst <= burst;
            arid    <= std_logic_vector(to_unsigned(len + 5, arid'length));
            arvalid <= '1';
            loop
                wait until rising_edge(aclk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';

            loop
                rready <= '0';
                for i in 1 to rnd(3) loop
                    wait until rising_edge(aclk);
                end loop;
                rready <= '1';
                loop
                    wait until rising_edge(aclk);
                    exit when rvalid = '1' and rready = '1';
                end loop;
                check(rresp = exp_resp, "RRESP " & to_string(rresp) & " at 0x"
                      & to_hstring(to_unsigned(addr, 32)));
                check(rid = std_logic_vector(to_unsigned(len + 5, rid'length)), "RID mismatch");
                check((rlast = '1') = (beat = len), "RLAST wrong on beat " & integer'image(beat));
                if exp_resp = "00" then
                    ba := beat_addr(addr, sz, beat);
                    lo := ba mod 64;
                    hi := ((ba mod 64) / size_bytes(sz)) * size_bytes(sz) + size_bytes(sz);
                    for lane in lo to hi - 1 loop
                        a := (ba / 64) * 64 + lane;
                        check(rdata(8 * lane + 7 downto 8 * lane) = mem.read(a),
                              "read 0x" & to_hstring(to_unsigned(a, 32)) & " = "
                              & to_hstring(rdata(8 * lane + 7 downto 8 * lane))
                              & ", expected " & to_hstring(mem.read(a)));
                    end loop;
                end if;
                exit when beat = len;
                beat := beat + 1;
            end loop;
            rready <= '0';
            reads_done := reads_done + 1;
            wait until rising_edge(aclk);
            check(n_rlast = reads_done, "RLAST handshakes: saw " & integer'image(n_rlast)
                  & ", expected " & integer'image(reads_done));
        end procedure;

        variable t_issue : time;
    begin
        -- T12: read issued while the boot copy is still running
        wait until aresetn = '1';
        for i in 1 to 50 loop wait until rising_edge(aclk); end loop;
        check(boot_done = '0', "boot finished before the early read was issued");
        t_issue := now;
        axi_read(16#0000#, 1, 6, "01", "00");
        check(boot_done = '1', "early read completed before boot_done");
        report "early read (issued " & time'image(t_issue) & ") completed " & time'image(now);

        if wr_step < 2 then wait until wr_step >= 2; end if;
        axi_read(16#1000#, 0, 6, "01", "00");      -- T3
        axi_read(16#2004#, 0, 2, "01", "00");      -- T4 narrow
        axi_read(16#2000#, 0, 6, "01", "00");      -- T4 whole line
        axi_read(16#3000#, 0, 6, "01", "00");      -- T5 sparse
        axi_read(16#4000#, 3, 6, "01", "00");      -- T6 burst
        axi_read(16#5002#, 3, 2, "01", "00");      -- T7 narrow unaligned burst
        axi_read(16#5030#, 3, 3, "01", "00");      -- T7b
        axi_read(16#5000#, 0, 0, "01", "00");      -- single byte
        axi_read(16#5001#, 0, 0, "01", "00");      -- single byte, odd lane
        axi_read(16#1000#, 1, 6, "10", "10");      -- WRAP rejected, 2 SLVERR beats
        axi_read(16#1000#, 0, 6, "01", "00");      -- still healthy afterwards

        if wr_step < 3 then wait until wr_step >= 3; end if;
        axi_read(SRC_BASE + 16#100#, 0, 6, "01", "00"); -- T8: unlocked write landed
        axi_read(SRC_BASE + 16#140#, 0, 6, "01", "00"); -- T8: locked write did not
        axi_read(16#0000_1200#, 0, 6, "01", "00");      -- T13 via the offset
        axi_read(16#4800_1200#, 0, 6, "01", "00");      -- T13 via another base

        -- T10: stream reads while the writer works in another region
        for i in 0 to 15 loop
            axi_read(16#1000# + 64 * (i mod 2), 0, 6, "01", "00");
            axi_read(16#2004#, 0, 2, "01", "00");
        end loop;
        if wr_step < 4 then wait until wr_step >= 4; end if;
        for i in 0 to 7 loop
            axi_read(16#3800# + 64 * i, 0, 6, "01", "00");
        end loop;

        report "reader done, " & integer'image(reads_done) & " reads, errors=" & integer'image(errs);
        rd_finish <= true;
        wait;
    end process;

    ----------------------------------------------------------------------------
    -- "PCI": register commands in the pci_clk domain
    ----------------------------------------------------------------------------
    p_pci : process
        variable errs : natural := 0;

        procedure check(cond : boolean; msg : string) is
        begin
            if not cond then
                errs := errs + 1;
                tb_errs_p <= errs;
                report "PCI: " & msg severity error;
            end if;
        end procedure;

        procedure do_cmd(i : natural; wr_data : std_logic_vector(7 downto 0) := x"00") is
            variable n : natural := 0;
        begin
            wait until rising_edge(pci_clk);
            check(cmd_done = '0', "cmd_done high before a request");
            cmd_wrsr_data <= wr_data;       -- same PCI write as the request
            cmd_req(i)    <= '1';
            loop
                wait until rising_edge(pci_clk);
                n := n + 1;
                exit when cmd_done = '1' or n > 5000;
            end loop;
            check(cmd_done = '1', "no cmd_done for command " & integer'image(i));
            cmd_req(i) <= '0';
            n := 0;
            loop
                wait until rising_edge(pci_clk);
                n := n + 1;
                exit when cmd_done = '0' or n > 100;
            end loop;
            check(cmd_done = '0', "cmd_done did not clear");
        end procedure;

        procedure expect_sr(v : std_logic_vector(7 downto 0); what : string) is
        begin
            do_cmd(I_RDSR);
            check(cmd_rdsr_data = v, what & ": status " & to_hstring(cmd_rdsr_data)
                  & ", expected " & to_hstring(v));
        end procedure;
    begin
        -- boot_hold is '1' from time 0: the copy must not start
        do_cmd(I_RDID);
        check(cmd_rdid_data = DEVICE_ID, "RDID " & to_hstring(cmd_rdid_data));
        check(boot_done = '0' and model_rd = 0, "boot copy ran despite boot_hold");
        expect_sr(x"00", "after reset");
        do_cmd(I_WREN);
        expect_sr(x"02", "after WREN");
        do_cmd(I_WRDI);
        expect_sr(x"00", "after WRDI");
        do_cmd(I_WRSR, x"14");                     -- no WREN: must be ignored
        expect_sr(x"00", "WRSR without WREN");
        do_cmd(I_WREN);
        do_cmd(I_WRSR, x"14");                     -- BP=101 TBPSEL=0: top 1/4
        expect_sr(x"14", "after WRSR 14h");         -- WREN cleared by WRSR
        do_cmd(I_WREN);
        do_cmd(I_WRSR, x"00");
        expect_sr(x"00", "after WRSR 00h");
        check(model_rd = 0 and model_wr = 0, "memory traffic while boot_hold");
        report "PCI: register commands OK, releasing boot_hold at " & time'image(now);
        boot_hold <= '0';

        -- T14: protect everything, let the writer try, then unprotect
        if not wr_paused then wait until wr_paused; end if;
        do_cmd(I_WREN);
        do_cmd(I_WRSR, x"1C");                     -- BP=111: whole array protected
        expect_sr(x"1C", "all protected");
        pci_step <= 1;
        if not wr_t14_done then wait until wr_t14_done; end if;
        do_cmd(I_WREN);
        do_cmd(I_WRSR, x"00");
        expect_sr(x"00", "unprotected again");
        pci_step <= 2;

        -- commands interleaved with AXI traffic
        for i in 1 to 6 loop
            do_cmd(I_RDID);
            check(cmd_rdid_data = DEVICE_ID, "RDID under traffic " & to_hstring(cmd_rdid_data));
            expect_sr(x"00", "RDSR under traffic");
        end loop;

        report "PCI done, errors=" & integer'image(errs);
        pci_finish <= true;
        wait;
    end process;

    ----------------------------------------------------------------------------
    -- End of test
    ----------------------------------------------------------------------------
    process
    begin
        wait until (wr_finish and rd_finish and pci_finish) or boot_fail = '1' for 20 ms;
        for i in 1 to 10 loop wait until rising_edge(aclk); end loop;
        report "model: " & integer'image(model_rd) & " RDQI, " & integer'image(model_wr)
               & " 4WQIO, " & integer'image(model_errs) & " protocol errors";
        if not (wr_finish and rd_finish and pci_finish) then
            report "TEST FAILED: timeout / boot_fail=" & std_logic'image(boot_fail) severity failure;
        elsif tb_errs_w + tb_errs_r + tb_errs_p + model_errs = 0 then
            report "TEST PASSED";
        else
            report "TEST FAILED: " & integer'image(tb_errs_w + tb_errs_r + tb_errs_p + model_errs)
                   & " errors" severity failure;
        end if;
        std.env.finish;
    end process;

end architecture sim;
