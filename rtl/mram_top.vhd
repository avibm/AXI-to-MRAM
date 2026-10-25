--------------------------------------------------------------------------------
-- mram_top.vhd
--
-- Two AXI masters (RISC-V/NOEL-V and PCI) reach this design through the
-- AXI interconnect, which arbitrates between them -- this block presents
-- a single AXI4 slave port and does no master-side arbitration of its
-- own. Both masters use identity addressing across the full 128MB range.
--
-- sram_boot_copy copies the firmware (G_BOOT_COPY_SIZE, 1MB) from the
-- MRAM master copy at MRAM offset G_BOOT_SRC_BASE (0x6000000) into the
-- on-chip SRAM at AXI address G_BOOT_DST_ADDR (0x0, NOEL-V's reset vector)
-- through its own AXI4 master port (m_axi_*, an extra initiator on the
-- interconnect), verifies it, and only then releases cpu_reset_n. It runs
-- after every aresetn and again on every rising edge of sram_reload
-- (which first puts the CPU back into reset). See doc/SRAM_ARCH.md.
-- mram_write_guard protects the master copy from ordinary AXI writes,
-- gated by key_ok -- this is what lets PCI deliberately reprogram it.
--
-- Wiring: sram_boot_copy and axi4_slave_wrapper both feed a mux selected
-- by boot_own (not an arbiter), whose output passes through
-- mram_write_guard to mram_qspi_backend. The boot copy only reads the
-- MRAM (always allowed by the guard).
--
-- Hand-over of the backend, without resetting the AXI side: the boot copy
-- raises boot_hold_axi, the wrapper stops accepting new AW/AR (a legal
-- stall) and finishes what it has accepted; when it reports idle, the boot
-- copy takes the backend (boot_own). Each side sees backend responses only
-- while it owns the backend. A PCI burst in flight therefore always
-- completes, and new MRAM accesses simply stall for the length of the copy
-- (roughly 0.12 s for 1MB, an estimate). The SRAM itself is not behind
-- this block: PCI accesses to it during the copy are not blocked here and
-- race the copy (the host must wait for boot_done).
--
-- MRAM register commands (WREN, WRDI, RDSR, WRSR, RDID) for bring-up and
-- debug are driven from PCI registers through mram_cmd_ctrl, which
-- synchronizes the requests into aclk and runs the four-phase cmd_* /
-- cmd_done handshake (see that file). cmd_done is an aclk register; the
-- PCI side synchronizes it. boot_hold = '1' keeps the boot copy from
-- starting after the power-up wait, so the device can be inspected first.
--
-- key_ok is a plain input here; the comparison that produces it (password
-- check or otherwise) is handled entirely outside this file.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use work.mram_pkg.all;

entity mram_top is
    generic (
        G_ID_WIDTH         : integer := C_AXI_ID_WIDTH;
        -- Boot copy (defaults: 1MB image at MRAM 0x6000000 copied to SRAM AXI 0x0)
        G_BOOT_SRC_BASE    : natural := 100663296;
        G_BOOT_DST_ADDR    : natural := 0;
        G_BOOT_COPY_SIZE   : natural := 1048576;
        G_BOOT_FIFO_BLOCKS : natural := 2;
        G_BOOT_MAX_RETRIES : natural := 3;
        G_BOOT_WATCHDOG    : natural := 1_000_000;
        G_M_ID_WIDTH       : natural := 4;      -- interconnect INITIATOR2 ID[3:0]
        G_M_ID             : natural := 0;
        G_POWERUP_CYCLES   : natural := 3_750_000;  -- MRAM tPU 25ms at 150MHz
        -- Write guard (defaults: protect 0x6000000..0x7FFFFFF)
        G_PROTECT_BASE     : natural := 100663296;
        G_PROTECT_SIZE     : natural := 33554432;
        G_SYNC_KEY_OK      : boolean := true;
        -- QSPI backend
        G_DUMMY_CYCLES         : integer := 8;   -- = MRAM CR2 latency (default 8)
        G_SCLK_HALF_PERIOD     : integer := 2;   -- SCLK = aclk / (2*N); datasheet max 54MHz
        G_CS_HIGH_WRITE_CYCLES : integer := 92;  -- tCS3 600ns at 150MHz
        G_POSTED_WRITES        : boolean := true; -- B as soon as the write data is held
        G_STREAM_WRITES        : boolean := true; -- one SPI write for contiguous AXI writes
        G_MERGE_WRITES         : boolean := true; -- false = no write merging at all (Oct 1 behaviour)
        G_ARB_MAX_WAIT         : natural := 256;  -- max aclk a read/write waits behind the other side
        G_WR_LINGER_CYCLES     : natural := 256;  -- CS# low wait for the next write; 0 = off
        G_STREAM_READS         : boolean := true; -- one RDQI for contiguous reads
        G_RD_LINGER_CYCLES     : natural := 256   -- CS# low wait for the next read; 0 = off
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic; -- system reset (NOT gated by cpu_reset_n)

        -- AXI4 slave, one arbitrated port shared by RISC-V and PCI via the
        -- AXI interconnect -- passed straight through to axi4_slave_wrapper.
        s_axi_awid     : in  std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_awaddr   : in  std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        s_axi_awlen    : in  std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        s_axi_awsize   : in  std_logic_vector(2 downto 0);
        s_axi_awburst  : in  std_logic_vector(1 downto 0);
        s_axi_awvalid  : in  std_logic;
        s_axi_awready  : out std_logic;
        s_axi_wdata    : in  std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        s_axi_wstrb    : in  std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);
        s_axi_wlast    : in  std_logic;
        s_axi_wvalid   : in  std_logic;
        s_axi_wready   : out std_logic;
        s_axi_bid      : out std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_bresp    : out std_logic_vector(1 downto 0);
        s_axi_bvalid   : out std_logic;
        s_axi_bready   : in  std_logic;
        s_axi_arid     : in  std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_araddr   : in  std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        s_axi_arlen    : in  std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        s_axi_arsize   : in  std_logic_vector(2 downto 0);
        s_axi_arburst  : in  std_logic_vector(1 downto 0);
        s_axi_arvalid  : in  std_logic;
        s_axi_arready  : out std_logic;
        s_axi_rid      : out std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_rdata    : out std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        s_axi_rresp    : out std_logic_vector(1 downto 0);
        s_axi_rlast    : out std_logic;
        s_axi_rvalid   : out std_logic;
        s_axi_rready   : in  std_logic;

        -- AXI4 master of the boot copy, to the SRAM through the interconnect.
        -- Only bursts of 64 bytes, INCR, full-width beats, all strobes.
        m_axi_awid    : out std_logic_vector(G_M_ID_WIDTH - 1 downto 0);
        m_axi_awaddr  : out std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        m_axi_awlen   : out std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        m_axi_awsize  : out std_logic_vector(2 downto 0);
        m_axi_awburst : out std_logic_vector(1 downto 0);
        m_axi_awlock  : out std_logic;
        m_axi_awcache : out std_logic_vector(3 downto 0);
        m_axi_awprot  : out std_logic_vector(2 downto 0);
        m_axi_awvalid : out std_logic;
        m_axi_awready : in  std_logic;
        m_axi_wdata   : out std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        m_axi_wstrb   : out std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);
        m_axi_wlast   : out std_logic;
        m_axi_wvalid  : out std_logic;
        m_axi_wready  : in  std_logic;
        m_axi_bid     : in  std_logic_vector(G_M_ID_WIDTH - 1 downto 0);
        m_axi_bresp   : in  std_logic_vector(1 downto 0);
        m_axi_bvalid  : in  std_logic;
        m_axi_bready  : out std_logic;
        m_axi_arid    : out std_logic_vector(G_M_ID_WIDTH - 1 downto 0);
        m_axi_araddr  : out std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        m_axi_arlen   : out std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        m_axi_arsize  : out std_logic_vector(2 downto 0);
        m_axi_arburst : out std_logic_vector(1 downto 0);
        m_axi_arlock  : out std_logic;
        m_axi_arcache : out std_logic_vector(3 downto 0);
        m_axi_arprot  : out std_logic_vector(2 downto 0);
        m_axi_arvalid : out std_logic;
        m_axi_arready : in  std_logic;
        m_axi_rid     : in  std_logic_vector(G_M_ID_WIDTH - 1 downto 0);
        m_axi_rdata   : in  std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        m_axi_rresp   : in  std_logic_vector(1 downto 0);
        m_axi_rlast   : in  std_logic;
        m_axi_rvalid  : in  std_logic;
        m_axi_rready  : out std_logic;

        key_ok : in std_logic; -- protected-region write unlock; comparison done externally
                               -- (synchronized inside mram_write_guard unless G_SYNC_KEY_OK = false)

        -- MRAM register commands from PCI registers (see mram_cmd_ctrl.vhd).
        -- Requests, cmd_wrsr_data and boot_hold may be asynchronous to aclk
        -- (synchronized inside); cmd_done and the results are aclk registers.
        cmd_wren      : in  std_logic;
        cmd_wrdi      : in  std_logic;
        cmd_rdsr      : in  std_logic;
        cmd_wrsr      : in  std_logic;
        cmd_rdid      : in  std_logic;
        cmd_rdar      : in  std_logic := '0';  -- Read Any Register  (address: cmd_reg_addr)
        cmd_wrar      : in  std_logic := '0';  -- Write Any Register (needs WREN first)
        cmd_wrsr_data : in  std_logic_vector(7 downto 0);   -- WRSR / WRAR data
        cmd_reg_addr  : in  std_logic_vector(7 downto 0) := (others => '0'); -- RDAR / WRAR
        cmd_rdsr_data : out std_logic_vector(7 downto 0);
        cmd_rdid_data : out std_logic_vector(31 downto 0);
        cmd_done      : out std_logic;
        boot_hold     : in  std_logic; -- '1' = do not start the boot copy yet
        -- Rising edge = reset the CPU and copy + verify the SRAM again
        -- (synchronized inside; may be asynchronous to aclk, e.g. a PCI
        -- register bit). Acted on after boot_done or boot_fail; ignored
        -- while a copy is running.
        sram_reload   : in  std_logic := '0';

        -- MRAM read sample delay, aclk cycles after the SCLK rising edge
        -- (see mram_qspi_backend.vhd). Quasi-static, e.g. from a PCI
        -- register: change only while the MRAM is idle (boot_hold = 1, no
        -- MRAM access in flight). "010" suits 37.5MHz SCLK on the first board.
        rd_sample_dly : in  std_logic_vector(2 downto 0) := "010";

        -- '1' = no WREN before MRAM array writes. Set only after CR1 WRENS
        -- has been written to 01 (SRAM mode) with WRAR and checked with
        -- RDAR; otherwise every write is silently ignored by the MRAM.
        -- Quasi-static (PCI register). Must be '0' at power-up so the boot
        -- copy works with the default CR1.
        skip_wren     : in  std_logic := '0';
        -- '1' while accepted (posted) write data has not reached the MRAM
        mram_wr_pending : out std_logic;
        -- Sticky debug flag: a read's capture count slipped (see backend)
        mram_rd_slip    : out std_logic;

        cpu_reset_n : out std_logic; -- wire to the actual CPU reset input externally
        boot_done   : out std_logic;
        boot_fail   : out std_logic; -- wire to a fault indicator; stays low unless retries exhausted
        boot_status : out std_logic_vector(15 downto 0); -- see sram_boot_copy.vhd

        mram_cs_n : out std_logic;
        mram_sclk : out std_logic;
        mram_io   : inout std_logic_vector(3 downto 0)
    );
end entity mram_top;

architecture rtl of mram_top is

    signal boot_req, axi_req, muxed_req, guarded_req : core_req_t;
    signal guard_resp                                  : core_resp_t; -- from mram_write_guard
    signal boot_resp, axi_resp                         : core_resp_t; -- masked by ownership
    signal backend_resp                                 : core_resp_t; -- raw, from mram_qspi_backend
    signal boot_own, boot_hold_axi, axi_idle           : std_logic;

    signal boot_hold_sync, powerup_done               : std_logic;
    signal reload_q1, reload_q2, reload_q3, reload_pulse : std_logic := '0';
    signal reg_cmd_valid, reg_cmd_accept, reg_cmd_done : std_logic;
    signal reg_cmd_op                                  : reg_cmd_t;
    signal reg_cmd_wdata, reg_cmd_addr                 : std_logic_vector(7 downto 0);
    signal reg_cmd_rdata                               : std_logic_vector(31 downto 0);

    signal backend_io_o, backend_io_oe, backend_io_i : std_logic_vector(3 downto 0);

    -- sram_reload synchronizer stages (asynchronous input)
    attribute syn_preserve : boolean;
    attribute syn_preserve of reload_q1, reload_q2 : signal is true;

begin

    -- sram_reload: two-flop synchronizer, then rising-edge detect
    process (aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                reload_q1    <= '0';
                reload_q2    <= '0';
                reload_q3    <= '0';
                reload_pulse <= '0';
            else
                reload_q1    <= sram_reload;
                reload_q2    <= reload_q1;
                reload_q3    <= reload_q2;
                reload_pulse <= reload_q2 and not reload_q3;
            end if;
        end if;
    end process;

    u_boot_copy : entity work.sram_boot_copy
        generic map (
            G_SRC_BASE       => G_BOOT_SRC_BASE,
            G_DST_ADDR       => G_BOOT_DST_ADDR,
            G_COPY_SIZE      => G_BOOT_COPY_SIZE,
            G_FIFO_BLOCKS    => G_BOOT_FIFO_BLOCKS,
            G_MAX_RETRIES    => G_BOOT_MAX_RETRIES,
            G_WATCHDOG_LIMIT => G_BOOT_WATCHDOG,
            G_POWERUP_CYCLES => G_POWERUP_CYCLES,
            G_M_ID_WIDTH     => G_M_ID_WIDTH,
            G_M_ID           => G_M_ID
        )
        port map (
            aclk          => aclk,
            aresetn       => aresetn,
            core_req      => boot_req,
            core_resp     => boot_resp,
            mram_hold     => boot_hold_axi,
            mram_idle     => axi_idle,
            mram_own      => boot_own,
            m_axi_awid    => m_axi_awid,
            m_axi_awaddr  => m_axi_awaddr,
            m_axi_awlen   => m_axi_awlen,
            m_axi_awsize  => m_axi_awsize,
            m_axi_awburst => m_axi_awburst,
            m_axi_awlock  => m_axi_awlock,
            m_axi_awcache => m_axi_awcache,
            m_axi_awprot  => m_axi_awprot,
            m_axi_awvalid => m_axi_awvalid,
            m_axi_awready => m_axi_awready,
            m_axi_wdata   => m_axi_wdata,
            m_axi_wstrb   => m_axi_wstrb,
            m_axi_wlast   => m_axi_wlast,
            m_axi_wvalid  => m_axi_wvalid,
            m_axi_wready  => m_axi_wready,
            m_axi_bid     => m_axi_bid,
            m_axi_bresp   => m_axi_bresp,
            m_axi_bvalid  => m_axi_bvalid,
            m_axi_bready  => m_axi_bready,
            m_axi_arid    => m_axi_arid,
            m_axi_araddr  => m_axi_araddr,
            m_axi_arlen   => m_axi_arlen,
            m_axi_arsize  => m_axi_arsize,
            m_axi_arburst => m_axi_arburst,
            m_axi_arlock  => m_axi_arlock,
            m_axi_arcache => m_axi_arcache,
            m_axi_arprot  => m_axi_arprot,
            m_axi_arvalid => m_axi_arvalid,
            m_axi_arready => m_axi_arready,
            m_axi_rid     => m_axi_rid,
            m_axi_rdata   => m_axi_rdata,
            m_axi_rresp   => m_axi_rresp,
            m_axi_rlast   => m_axi_rlast,
            m_axi_rvalid  => m_axi_rvalid,
            m_axi_rready  => m_axi_rready,
            reload        => reload_pulse,
            boot_hold     => boot_hold_sync,
            cpu_reset_n   => cpu_reset_n,
            boot_done     => boot_done,
            boot_fail     => boot_fail,
            powerup_done  => powerup_done,
            boot_held     => open,
            boot_status   => boot_status
        );

    u_cmd_ctrl : entity work.mram_cmd_ctrl
        port map (
            aclk           => aclk,
            aresetn        => aresetn,
            cmd_wren       => cmd_wren,
            cmd_wrdi       => cmd_wrdi,
            cmd_rdsr       => cmd_rdsr,
            cmd_wrsr       => cmd_wrsr,
            cmd_rdid       => cmd_rdid,
            cmd_rdar       => cmd_rdar,
            cmd_wrar       => cmd_wrar,
            cmd_wrsr_data  => cmd_wrsr_data,
            cmd_reg_addr   => cmd_reg_addr,
            cmd_rdsr_data  => cmd_rdsr_data,
            cmd_rdid_data  => cmd_rdid_data,
            cmd_done       => cmd_done,
            boot_hold      => boot_hold,
            boot_hold_sync => boot_hold_sync,
            mem_ready      => powerup_done,
            reg_cmd_valid  => reg_cmd_valid,
            reg_cmd_op     => reg_cmd_op,
            reg_cmd_wdata  => reg_cmd_wdata,
            reg_cmd_addr   => reg_cmd_addr,
            reg_cmd_accept => reg_cmd_accept,
            reg_cmd_done   => reg_cmd_done,
            reg_cmd_rdata  => reg_cmd_rdata
        );

    u_axi_wrapper : entity work.axi4_slave_wrapper
        generic map (
            G_ID_WIDTH     => G_ID_WIDTH,
            G_MERGE_WRITES => G_MERGE_WRITES,
            G_ARB_MAX_WAIT => G_ARB_MAX_WAIT
        )
        port map (
            aclk           => aclk,
            aresetn        => aresetn,
            hold           => boot_hold_axi,
            idle           => axi_idle,
            s_axi_awid     => s_axi_awid,
            s_axi_awaddr   => s_axi_awaddr,
            s_axi_awlen    => s_axi_awlen,
            s_axi_awsize   => s_axi_awsize,
            s_axi_awburst  => s_axi_awburst,
            s_axi_awvalid  => s_axi_awvalid,
            s_axi_awready  => s_axi_awready,
            s_axi_wdata    => s_axi_wdata,
            s_axi_wstrb    => s_axi_wstrb,
            s_axi_wlast    => s_axi_wlast,
            s_axi_wvalid   => s_axi_wvalid,
            s_axi_wready   => s_axi_wready,
            s_axi_bid      => s_axi_bid,
            s_axi_bresp    => s_axi_bresp,
            s_axi_bvalid   => s_axi_bvalid,
            s_axi_bready   => s_axi_bready,
            s_axi_arid     => s_axi_arid,
            s_axi_araddr   => s_axi_araddr,
            s_axi_arlen    => s_axi_arlen,
            s_axi_arsize   => s_axi_arsize,
            s_axi_arburst  => s_axi_arburst,
            s_axi_arvalid  => s_axi_arvalid,
            s_axi_arready  => s_axi_arready,
            s_axi_rid      => s_axi_rid,
            s_axi_rdata    => s_axi_rdata,
            s_axi_rresp    => s_axi_rresp,
            s_axi_rlast    => s_axi_rlast,
            s_axi_rvalid   => s_axi_rvalid,
            s_axi_rready   => s_axi_rready,
            core_req       => axi_req,
            core_resp      => axi_resp
        );

    -- Plain mux, not an arbiter: exactly one side owns the backend, and the
    -- ownership (a register in sram_boot_copy) only changes while neither
    -- side has a request in flight. Responses go only to the owner.
    muxed_req <= boot_req when boot_own = '1' else axi_req;

    process (all)
    begin
        boot_resp <= guard_resp;
        axi_resp  <= guard_resp;
        if boot_own = '1' then
            axi_resp.ready  <= '0';
            axi_resp.rvalid <= '0';
            axi_resp.bvalid <= '0';
        else
            boot_resp.ready  <= '0';
            boot_resp.rvalid <= '0';
            boot_resp.bvalid <= '0';
        end if;
    end process;

    u_write_guard : entity work.mram_write_guard
        generic map (
            G_PROTECT_BASE => G_PROTECT_BASE,
            G_PROTECT_SIZE => G_PROTECT_SIZE,
            G_SYNC_KEY_OK  => G_SYNC_KEY_OK
        )
        port map (
            aclk           => aclk,
            aresetn        => aresetn,
            key_ok         => key_ok,
            core_req_in    => muxed_req,
            core_resp_out  => guard_resp,
            core_req_out   => guarded_req,
            core_resp_in   => backend_resp
        );

    u_backend : entity work.mram_qspi_backend
        generic map (
            G_DUMMY_CYCLES         => G_DUMMY_CYCLES,
            G_SCLK_HALF_PERIOD     => G_SCLK_HALF_PERIOD,
            G_CS_HIGH_WRITE_CYCLES => G_CS_HIGH_WRITE_CYCLES,
            G_POSTED_WRITES        => G_POSTED_WRITES,
            G_STREAM_WRITES        => G_STREAM_WRITES,
            G_MERGE_WRITES         => G_MERGE_WRITES,
            G_WR_LINGER_CYCLES     => G_WR_LINGER_CYCLES,
            G_STREAM_READS         => G_STREAM_READS,
            G_RD_LINGER_CYCLES     => G_RD_LINGER_CYCLES
        )
        port map (
            aclk       => aclk,
            aresetn    => aresetn,
            core_req   => guarded_req,
            core_resp  => backend_resp,
            reg_cmd_valid  => reg_cmd_valid,
            reg_cmd_op     => reg_cmd_op,
            reg_cmd_wdata  => reg_cmd_wdata,
            reg_cmd_addr   => reg_cmd_addr,
            reg_cmd_accept => reg_cmd_accept,
            reg_cmd_done   => reg_cmd_done,
            reg_cmd_rdata  => reg_cmd_rdata,
            mram_cs_n  => mram_cs_n,
            mram_sclk  => mram_sclk,
            mram_io_o  => backend_io_o,
            mram_io_oe => backend_io_oe,
            mram_io_i  => backend_io_i,
            rd_sample_dly => rd_sample_dly,
            skip_wren     => skip_wren,
            wr_pending    => mram_wr_pending,
            rd_slip       => mram_rd_slip
        );

    gen_pins : for i in 0 to 3 generate
        mram_io(i)      <= backend_io_o(i) when backend_io_oe(i) = '1' else 'Z';
        backend_io_i(i) <= mram_io(i);
    end generate;

end architecture rtl;
