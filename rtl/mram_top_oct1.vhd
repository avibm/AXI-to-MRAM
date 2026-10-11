--------------------------------------------------------------------------------
-- mram_top_oct1.vhd
--
-- Diagnostic top level: the current MRAM IP behind EXACTLY the generic and
-- port list of the Oct 1 mram_top (rev. 5, the last version that passed
-- the PCI file copy on the board). Use it in place of mram_top, connected
-- exactly as in the Oct 1 build, to find out whether the PCI write hang
-- that started with the Oct 4 version comes from the IP or from the way
-- the newer top-level ports are connected.
--
-- Inside, every feature added after Oct 1 that changes behaviour on the
-- AXI bus or the MRAM is switched off (constants below):
--   * no write merging, no posted B, no streaming, no linger: each core
--     write gets its bvalid only after its MRAM write has finished, and the
--     wrapper takes the next W beat only after that, as on Oct 1;
--   * no read streaming, no read linger;
--   * the newer inputs are tied off: cmd_rdar = cmd_wrar = '0',
--     cmd_reg_addr = 0, skip_wren = '0' (WREN before every write);
--   * the newer outputs (mram_wr_pending, mram_rd_slip) are left open.
-- Still different from Oct 1, and not switchable: the 64-bit capable data
-- path (C_AXI_DATA_WIDTH in mram_pkg sets the AXI width; set it to match
-- the interconnect, 512 for the Oct 1 interconnect configuration), the
-- read engine issuing beat reads ahead of the R channel, the boot copy
-- moving 64-byte blocks, and the read capture fix.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity mram_top_oct1 is
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
        -- Requests, cmd_wrsr_data and boot_hold may be asynchronous to aclk
        -- (synchronized inside); cmd_done and the results are aclk registers.
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

        -- MRAM read sample delay, aclk cycles after the SCLK rising edge
        -- (see mram_qspi_backend.vhd). Quasi-static, e.g. from a PCI
        -- register: change only while the MRAM is idle (boot_hold = 1, no
        -- MRAM access in flight). "010" suits 37.5MHz SCLK on the first board.
        rd_sample_dly : in  std_logic_vector(2 downto 0) := "010";

        cpu_reset_n : out std_logic; -- wire to the actual CPU reset input externally
        boot_done   : out std_logic;
        boot_fail   : out std_logic; -- wire to a fault indicator; stays low unless retries exhausted

        mram_cs_n : out std_logic;
        mram_sclk : out std_logic;
        mram_io   : inout std_logic_vector(3 downto 0)
    );
end entity mram_top_oct1;


architecture rtl of mram_top_oct1 is

    -- Post-Oct-1 features, all off for this diagnostic build
    constant C_POSTED_WRITES    : boolean := false;
    constant C_STREAM_WRITES    : boolean := false;
    constant C_MERGE_WRITES     : boolean := false;
    constant C_WR_LINGER_CYCLES : natural := 0;
    constant C_STREAM_READS     : boolean := false;
    constant C_RD_LINGER_CYCLES : natural := 0;

begin

    u_mram : entity work.mram_top
        generic map (
            G_ID_WIDTH             => G_ID_WIDTH,
            G_BOOT_SRC_BASE        => G_BOOT_SRC_BASE,
            G_BOOT_DST_BASE        => G_BOOT_DST_BASE,
            G_BOOT_COPY_SIZE       => G_BOOT_COPY_SIZE,
            G_BOOT_MAX_RETRIES     => G_BOOT_MAX_RETRIES,
            G_BOOT_WATCHDOG        => G_BOOT_WATCHDOG,
            G_POWERUP_CYCLES       => G_POWERUP_CYCLES,
            G_PROTECT_BASE         => G_PROTECT_BASE,
            G_PROTECT_SIZE         => G_PROTECT_SIZE,
            G_SYNC_KEY_OK          => G_SYNC_KEY_OK,
            G_DUMMY_CYCLES         => G_DUMMY_CYCLES,
            G_SCLK_HALF_PERIOD     => G_SCLK_HALF_PERIOD,
            G_CS_HIGH_WRITE_CYCLES => G_CS_HIGH_WRITE_CYCLES,
            G_POSTED_WRITES        => C_POSTED_WRITES,
            G_STREAM_WRITES        => C_STREAM_WRITES,
            G_MERGE_WRITES         => C_MERGE_WRITES,
            G_WR_LINGER_CYCLES     => C_WR_LINGER_CYCLES,
            G_STREAM_READS         => C_STREAM_READS,
            G_RD_LINGER_CYCLES     => C_RD_LINGER_CYCLES
        )
        port map (
            aclk           => aclk,
            aresetn        => aresetn,
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
            key_ok         => key_ok,
            cmd_wren       => cmd_wren,
            cmd_wrdi       => cmd_wrdi,
            cmd_rdsr       => cmd_rdsr,
            cmd_wrsr       => cmd_wrsr,
            cmd_rdid       => cmd_rdid,
            cmd_rdar       => '0',
            cmd_wrar       => '0',
            cmd_wrsr_data  => cmd_wrsr_data,
            cmd_reg_addr   => (others => '0'),
            cmd_rdsr_data  => cmd_rdsr_data,
            cmd_rdid_data  => cmd_rdid_data,
            cmd_done       => cmd_done,
            boot_hold      => boot_hold,
            rd_sample_dly  => rd_sample_dly,
            skip_wren      => '0',
            mram_wr_pending => open,
            mram_rd_slip    => open,
            cpu_reset_n    => cpu_reset_n,
            boot_done      => boot_done,
            boot_fail      => boot_fail,
            mram_cs_n      => mram_cs_n,
            mram_sclk      => mram_sclk,
            mram_io        => mram_io
        );

end architecture rtl;
