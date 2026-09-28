--------------------------------------------------------------------------------
-- mram_top.vhd
--
-- Two AXI masters (RISC-V/NOEL-V and PCI) reach this design through the
-- AXI interconnect, which arbitrates between them -- this block presents
-- a single AXI4 slave port and does no master-side arbitration of its
-- own. Both masters use identity addressing across the full 128MB range.
--
-- mram_boot_copy refreshes MRAM address 0 (the CPU's working copy, where
-- NOEL-V's fixed reset vector points) from the pristine master copy at
-- MRAM 0x6000000 on every reset, holding the CPU in reset until the copy
-- is verified. This is needed because the CPU's own runtime read/write
-- traffic to general memory would otherwise leave stale or corrupted data
-- at address 0 by the next boot. mram_write_guard separately protects the
-- master copy at 0x6000000 from ordinary AXI writes, gated by a 16-bit
-- key_ok signal -- this is what lets PCI deliberately reprogram it. The two
-- mechanisms address different problems and both are needed.
--
-- Wiring: mram_boot_copy and axi4_slave_wrapper both feed a boot_done-
-- selected mux (not an arbiter), whose output passes through
-- mram_write_guard before reaching
-- mram_qspi_backend. mram_boot_copy's own traffic is never actually
-- affected by the guard: its reads (from the protected master copy) are
-- always allowed, and its writes (to the unprotected working-copy region)
-- are always allowed -- the guard exists for AXI-side writes to the
-- protected region, which mram_boot_copy never attempts.
--
-- The CPU is held in reset during the copy, but the PCI master is not, so
-- AXI traffic can arrive while mram_boot_copy owns the backend. To keep
-- that traffic from seeing boot-copy responses, axi4_slave_wrapper is held
-- in reset until boot_done: AWREADY/ARREADY stay low and AXI requests
-- simply stall at the interconnect for the whole copy (roughly 9 s, or up
-- to roughly 35 s with retries -- see mram_boot_copy.vhd). Any PCI-side
-- completion timeout, and whether the host should instead wait for
-- boot_done before touching this window, must be handled at system level.
--
-- MRAM register commands (WREN, WRDI, RDSR, WRSR, RDID) for bring-up and
-- debug are driven from PCI registers through mram_cmd_ctrl, which handles
-- the pci_clk / aclk crossing and the four-phase cmd_* / cmd_done
-- handshake (see that file). boot_hold = '1' keeps the boot copy from
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
        -- Boot copy (defaults: 32MB image at 0x6000000 copied to 0x0)
        G_BOOT_SRC_BASE    : natural := 100663296;
        G_BOOT_DST_BASE    : natural := 0;
        G_BOOT_COPY_SIZE   : natural := 33554432;
        G_BOOT_MAX_RETRIES : natural := 3;
        G_BOOT_WATCHDOG    : natural := 1_000_000;
        G_POWERUP_CYCLES   : natural := 3_750_000;  -- MRAM tPU 25ms at 150MHz
        -- Write guard (defaults: protect 0x6000000..0x7FFFFFF)
        G_PROTECT_BASE     : natural := 100663296;
        G_PROTECT_SIZE     : natural := 33554432;
        G_SYNC_KEY_OK      : boolean := true;
        -- QSPI backend
        G_DUMMY_CYCLES         : integer := 8;   -- = MRAM CR2 latency (default 8)
        G_SCLK_HALF_PERIOD     : integer := 2;   -- SCLK = aclk / (2*N); datasheet max 54MHz
        G_CS_HIGH_WRITE_CYCLES : integer := 92   -- tCS3 600ns at 150MHz
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

        key_ok : in std_logic; -- protected-region write unlock; comparison done externally
                               -- (synchronized inside mram_write_guard unless G_SYNC_KEY_OK = false)

        -- MRAM register commands from PCI registers (see mram_cmd_ctrl.vhd).
        -- Requests, cmd_wrsr_data and boot_hold are asynchronous to aclk;
        -- cmd_done is synchronous to pci_clk.
        pci_clk       : in  std_logic;
        cmd_wren      : in  std_logic;
        cmd_wrdi      : in  std_logic;
        cmd_rdsr      : in  std_logic;
        cmd_wrsr      : in  std_logic;
        cmd_rdid      : in  std_logic;
        cmd_wrsr_data : in  std_logic_vector(7 downto 0);
        cmd_rdsr_data : out std_logic_vector(7 downto 0);
        cmd_rdid_data : out std_logic_vector(31 downto 0);
        cmd_done      : out std_logic;
        boot_hold     : in  std_logic; -- '1' = do not start the boot copy yet

        cpu_reset_n : out std_logic; -- wire to the actual CPU reset input externally
        boot_done   : out std_logic;
        boot_fail   : out std_logic; -- wire to a fault indicator; stays low unless retries exhausted

        mram_cs_n : out std_logic;
        mram_sclk : out std_logic;
        mram_io   : inout std_logic_vector(3 downto 0)
    );
end entity mram_top;

architecture rtl of mram_top is

    signal boot_req, axi_req, muxed_req, guarded_req : core_req_t;
    signal guard_resp                                  : core_resp_t; -- final, to boot_copy & wrapper
    signal backend_resp                                 : core_resp_t; -- raw, from mram_qspi_backend
    signal boot_done_i                                  : std_logic;
    signal axi_resetn                                   : std_logic;

    signal boot_hold_sync, powerup_done               : std_logic;
    signal reg_cmd_valid, reg_cmd_accept, reg_cmd_done : std_logic;
    signal reg_cmd_op                                  : reg_cmd_t;
    signal reg_cmd_wdata                               : std_logic_vector(7 downto 0);
    signal reg_cmd_rdata                               : std_logic_vector(31 downto 0);

    signal backend_io_o, backend_io_oe, backend_io_i : std_logic_vector(3 downto 0);

begin

    boot_done <= boot_done_i;

    -- The AXI side stays in reset until the boot copy has finished.
    axi_resetn <= aresetn and boot_done_i;

    u_boot_copy : entity work.mram_boot_copy
        generic map (
            G_SRC_BASE       => G_BOOT_SRC_BASE,
            G_DST_BASE       => G_BOOT_DST_BASE,
            G_COPY_SIZE      => G_BOOT_COPY_SIZE,
            G_MAX_RETRIES    => G_BOOT_MAX_RETRIES,
            G_WATCHDOG_LIMIT => G_BOOT_WATCHDOG,
            G_POWERUP_CYCLES => G_POWERUP_CYCLES
        )
        port map (
            aclk        => aclk,
            aresetn     => aresetn,
            core_req    => boot_req,
            core_resp   => guard_resp,
            cpu_reset_n => cpu_reset_n,
            boot_done    => boot_done_i,
            boot_fail    => boot_fail,
            boot_hold    => boot_hold_sync,
            powerup_done => powerup_done
        );

    u_cmd_ctrl : entity work.mram_cmd_ctrl
        port map (
            aclk           => aclk,
            aresetn        => aresetn,
            pci_clk        => pci_clk,
            cmd_wren       => cmd_wren,
            cmd_wrdi       => cmd_wrdi,
            cmd_rdsr       => cmd_rdsr,
            cmd_wrsr       => cmd_wrsr,
            cmd_rdid       => cmd_rdid,
            cmd_wrsr_data  => cmd_wrsr_data,
            cmd_rdsr_data  => cmd_rdsr_data,
            cmd_rdid_data  => cmd_rdid_data,
            cmd_done       => cmd_done,
            boot_hold      => boot_hold,
            boot_hold_sync => boot_hold_sync,
            mem_ready      => powerup_done,
            reg_cmd_valid  => reg_cmd_valid,
            reg_cmd_op     => reg_cmd_op,
            reg_cmd_wdata  => reg_cmd_wdata,
            reg_cmd_accept => reg_cmd_accept,
            reg_cmd_done   => reg_cmd_done,
            reg_cmd_rdata  => reg_cmd_rdata
        );

    u_axi_wrapper : entity work.axi4_slave_wrapper
        generic map (
            G_ID_WIDTH => G_ID_WIDTH
        )
        port map (
            aclk           => aclk,
            aresetn        => axi_resetn,
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
            core_resp      => guard_resp
        );

    -- Plain mux, not an arbiter -- the CPU is held in reset and the AXI
    -- wrapper is held in reset for the entire time mram_boot_copy runs.
    muxed_req <= boot_req when boot_done_i = '0' else axi_req;

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
            G_CS_HIGH_WRITE_CYCLES => G_CS_HIGH_WRITE_CYCLES
        )
        port map (
            aclk       => aclk,
            aresetn    => aresetn,
            core_req   => guarded_req,
            core_resp  => backend_resp,
            reg_cmd_valid  => reg_cmd_valid,
            reg_cmd_op     => reg_cmd_op,
            reg_cmd_wdata  => reg_cmd_wdata,
            reg_cmd_accept => reg_cmd_accept,
            reg_cmd_done   => reg_cmd_done,
            reg_cmd_rdata  => reg_cmd_rdata,
            mram_cs_n  => mram_cs_n,
            mram_sclk  => mram_sclk,
            mram_io_o  => backend_io_o,
            mram_io_oe => backend_io_oe,
            mram_io_i  => backend_io_i
        );

    gen_pins : for i in 0 to 3 generate
        mram_io(i)      <= backend_io_o(i) when backend_io_oe(i) = '1' else 'Z';
        backend_io_i(i) <= mram_io(i);
    end generate;

end architecture rtl;
