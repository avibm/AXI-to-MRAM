--------------------------------------------------------------------------------
-- tb_dma.vhd  (simulation only)
--
-- Stress test for mixed traffic on the one AXI port, as on the board: a PCI
-- DMA read (G_NBURST INCR bursts of G_LEN+1 beats of 2**G_SZ bytes, e.g. the
-- bridge's 32-bit len 15 bursts) with up to G_OUTST ARs outstanding and random
-- RREADY stalls, interleaved with CPU-like traffic (G_CPU): 32-byte line
-- reads near address 0 and a steady stream of 8-byte posted writes.
-- Checks every read byte, RID, RLAST and RRESP, and fails on a hang
-- (no completion within 20 ms of simulated time).
--
-- Run: sim/run_dma.sh [-gG_SZ=3 -gG_LEN=7 -gG_SEED=5 -gG_CPU=false ...]
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;
use work.qspi_mram_mem_pkg.all;
use work.axi_sram_mem_pkg.all;

entity tb_dma is
    generic (G_SZ : natural := 2; G_LEN : natural := 15; G_NBURST : natural := 64;
             G_OUTST : natural := 4; G_SEED : natural := 7;
             G_CPU : boolean := true);
end entity;

architecture sim of tb_dma is
    constant T_CLK : time := 6.667 ns;
    constant BB : natural := C_BEAT_BYTES;
    signal aclk, aresetn : std_logic := '0';
    signal arid : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0) := (others => '0');
    signal araddr : std_logic_vector(31 downto 0) := (others => '0');
    signal arlen : std_logic_vector(7 downto 0) := (others => '0');
    signal arsize : std_logic_vector(2 downto 0) := "010";
    signal arvalid, arready, rvalid, rready, rlast : std_logic := '0';
    signal rid : std_logic_vector(C_AXI_ID_WIDTH - 1 downto 0);
    signal rdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal rresp : std_logic_vector(1 downto 0);
    signal cs_n, sclk : std_logic;
    signal io : std_logic_vector(3 downto 0);
    signal boot_done : std_logic := '0';
    -- boot copy AXI master -> behavioural SRAM (axi_sram_model)
    signal m_awid, m_arid, m_bid, m_rid : std_logic_vector(C_M_AXI_ID_WIDTH - 1 downto 0);
    signal m_awaddr, m_araddr : std_logic_vector(31 downto 0);
    signal m_awlen, m_arlen   : std_logic_vector(7 downto 0);
    signal m_awsize, m_arsize : std_logic_vector(2 downto 0);
    signal m_awburst, m_arburst, m_bresp, m_rresp : std_logic_vector(1 downto 0);
    signal m_awvalid, m_awready, m_wvalid, m_wready, m_wlast, m_bvalid, m_bready : std_logic;
    signal m_arvalid, m_arready, m_rvalid, m_rready, m_rlast : std_logic;
    signal m_wdata, m_rdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal m_wstrb : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);
    signal merrs, mrd, mwr : natural;
    signal ar_q : natural := 0;    -- ARs issued
    signal r_done_n : natural := 0; -- bursts fully received
    type nat_arr is array (0 to 4095) of natural;
    signal q_addr, q_sz, q_len : nat_arr := (others => 0);
    signal n_cpu : natural := 0;
    -- write side (CPU-like stack/data writes, posted B)
    signal awaddr : std_logic_vector(31 downto 0) := (others => '0');
    signal awlen : std_logic_vector(7 downto 0) := (others => '0');
    signal awvalid, awready, wvalid, wready, wlast, bvalid, bready : std_logic := '0';
    signal wdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal wstrb : std_logic_vector(C_BEAT_BYTES - 1 downto 0) := (others => '0');
    signal n_wr_done : natural := 0;
    signal errs : natural := 0;
    signal done : boolean := false;
    function pat(a : natural) return std_logic_vector is
    begin return std_logic_vector(to_unsigned((a * 13 + a / 256 + 7) mod 256, 8)); end;
begin
    aclk <= not aclk after T_CLK / 2;
    aresetn <= '1' after 20 * T_CLK;
    io <= (others => 'H');
    dut : entity work.mram_top
        generic map (G_BOOT_COPY_SIZE => 512, G_POWERUP_CYCLES => 1500)
        port map (aclk => aclk, aresetn => aresetn,
            s_axi_awid => (others => '1'), s_axi_awaddr => awaddr, s_axi_awlen => awlen,
            s_axi_awsize => "011", s_axi_awburst => "01", s_axi_awvalid => awvalid, s_axi_awready => awready,
            s_axi_wdata => wdata, s_axi_wstrb => wstrb, s_axi_wlast => wlast,
            s_axi_wvalid => wvalid, s_axi_wready => wready, s_axi_bid => open, s_axi_bresp => open,
            s_axi_bvalid => bvalid, s_axi_bready => bready,
            s_axi_arid => arid, s_axi_araddr => araddr, s_axi_arlen => arlen, s_axi_arsize => arsize,
            s_axi_arburst => "01", s_axi_arvalid => arvalid, s_axi_arready => arready,
            s_axi_rid => rid, s_axi_rdata => rdata, s_axi_rresp => rresp, s_axi_rlast => rlast,
            s_axi_rvalid => rvalid, s_axi_rready => rready,
            key_ok => '0', cmd_wren => '0', cmd_wrdi => '0', cmd_rdsr => '0', cmd_wrsr => '0',
            cmd_rdid => '0', cmd_wrsr_data => x"00", cmd_rdsr_data => open, cmd_rdid_data => open,
            cmd_done => open, boot_hold => '0', mram_wr_pending => open, mram_rd_slip => open,
            cpu_reset_n => open, boot_done => boot_done, boot_fail => open,
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
            m_axi_rvalid => m_rvalid, m_axi_rready => m_rready, boot_status => open,
            mram_cs_n => cs_n, mram_sclk => sclk, mram_io => io);
    sram_mdl : entity work.axi_sram_model
        generic map (G_ID_WIDTH => C_M_AXI_ID_WIDTH, G_DATA_W => C_AXI_DATA_WIDTH, G_STALL_PCT => 20)
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
            s_axi_rvalid => m_rvalid, s_axi_rready => m_rready);

    mdl : entity work.qspi_mram_model
        generic map (G_TPU => 10 us)
        port map (cs_n => cs_n, sclk => sclk, io => io, errors => merrs, n_rd => mrd, n_wr => mwr);

    process begin
        -- master copy, and the CPU line-read area 0x0-0xFFF (with the SRAM
        -- boot copy nothing is copied there any more); CPU writes go to 0x1000..
        for a in 0 to 16383 loop
            mem.write(16#6000000# + a, pat(a));
            if a < 4096 then mem.write(a, pat(a)); else mem.write(a, x"EE"); end if;
        end loop;
        wait;
    end process;

    -- AR issuer: DMA bursts interleaved with CPU-like cache-line reads
    p_ar : process
        variable lfsr : unsigned(15 downto 0) := to_unsigned(16#1D0F# + G_SEED, 16);
        variable b, i, ad, sz, ln : natural := 0;
    begin
        wait until boot_done = '1';
        while b < G_NBURST loop
            loop
                wait until rising_edge(aclk);
                exit when ar_q - r_done_n < G_OUTST;
            end loop;
            lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
            if G_CPU and lfsr(1 downto 0) = "00" then
                ad := to_integer(lfsr(8 downto 5)) * 32; sz := 3; ln := 3;    -- CPU line fill
                n_cpu <= n_cpu + 1;
            else
                ad := 16#6000000# + b * (G_LEN + 1) * 2 ** G_SZ; sz := G_SZ; ln := G_LEN;
                b := b + 1;
            end if;
            q_addr(i) <= ad; q_sz(i) <= sz; q_len(i) <= ln;
            araddr  <= std_logic_vector(to_unsigned(ad, 32));
            arlen   <= std_logic_vector(to_unsigned(ln, 8));
            arsize  <= std_logic_vector(to_unsigned(sz, 3));
            arid    <= std_logic_vector(to_unsigned(i mod 8, arid'length));
            arvalid <= '1';
            loop
                wait until rising_edge(aclk);
                exit when arready = '1';
            end loop;
            arvalid <= '0';
            i := i + 1;
            ar_q <= i;
        end loop;
        wait;
    end process;

    -- CPU-like writes (8 bytes, posted B) to 0x1000.. while the reads run
    p_w : process
        variable lfsr : unsigned(15 downto 0) := to_unsigned(16#BEEF# + G_SEED, 16);
        variable k : natural := 0;
    begin
        wait until boot_done = '1';
        while G_CPU and not done loop
            lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
            for j in 1 to to_integer(lfsr(7 downto 0)) loop wait until rising_edge(aclk); end loop;
            awaddr  <= std_logic_vector(to_unsigned(16#1000# + 8 * (k mod 256), 32));
            awlen   <= x"00";
            awvalid <= '1';
            wdata   <= (others => '0');
            wdata(63 downto 0) <= std_logic_vector(to_unsigned(k, 32)) & std_logic_vector(to_unsigned(k, 32));
            wstrb   <= (others => '0');
            wstrb(((16#1000# + 8 * (k mod 256)) mod BB) + 7 downto (16#1000# + 8 * (k mod 256)) mod BB) <= (others => '1');
            wlast   <= '1'; wvalid <= '1'; bready <= '1';
            loop
                wait until rising_edge(aclk);
                if awvalid = '1' and awready = '1' then awvalid <= '0'; end if;
                if wvalid = '1' and wready = '1' then wvalid <= '0'; end if;
                exit when bvalid = '1';
            end loop;
            bready <= '0';
            k := k + 1;
            n_wr_done <= k;
        end loop;
        wait;
    end process;

    -- R consumer: random RREADY stalls, byte-exact check against the queue
    p_r : process
        variable lfsr : unsigned(15 downto 0) := to_unsigned(16#ACE1# + G_SEED, 16);
        variable beat, i, a, e, nb, n, dma : natural := 0;
        variable t0 : time;
    begin
        wait until boot_done = '1';
        t0 := now;
        while dma < G_NBURST loop
            lfsr := lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
            if lfsr(2 downto 0) = "000" then
                rready <= '0';
                for j in 1 to to_integer(lfsr(9 downto 4)) loop wait until rising_edge(aclk); end loop;
            end if;
            rready <= '1';
            wait until rising_edge(aclk);
            if rvalid = '1' and rready = '1' then
                if i >= ar_q then e := e + 1; report "R without AR" severity error; end if;
                n := 2 ** q_sz(i);
                for k in 0 to n - 1 loop
                    a := q_addr(i) + beat * n + k;
                    if rdata(8 * (a mod BB) + 7 downto 8 * (a mod BB)) /= pat(a mod 16#6000000#) then
                        e := e + 1;
                        if e < 10 then report "DATA AR#" & integer'image(i) & " addr 0x"
                            & to_hstring(to_unsigned(a, 32)) severity error; end if;
                    end if;
                end loop;
                if (rlast = '1') /= (beat = q_len(i)) then e := e + 1; report "RLAST wrong" severity error; end if;
                if rid /= std_logic_vector(to_unsigned(i mod 8, rid'length)) then
                    e := e + 1; report "RID wrong" severity error; end if;
                if rresp /= "00" then e := e + 1; report "RRESP error" severity error; end if;
                if beat = q_len(i) then
                    if q_addr(i) >= 16#6000000# then dma := dma + 1; end if;
                    beat := 0; i := i + 1; r_done_n <= i;
                else
                    beat := beat + 1;
                end if;
            end if;
        end loop;
        rready <= '0';
        report "DMA read: " & integer'image(G_NBURST) & " DMA bursts + " & integer'image(n_cpu)
               & " CPU reads + " & integer'image(n_wr_done) & " CPU writes in " & time'image(now - t0)
               & ", errors=" & integer'image(e) & ", model RDQI=" & integer'image(mrd)
               & ", model errors=" & integer'image(merrs);
        done <= true;
        wait;
    end process;

    process begin
        wait until done for 20 ms;
        if not done then
            report "HANG: bursts received " & integer'image(r_done_n) & ", ARs issued " & integer'image(ar_q)
                   & ", CPU writes done " & integer'image(n_wr_done) severity failure;
        end if;
        std.env.finish;
    end process;
end architecture;
