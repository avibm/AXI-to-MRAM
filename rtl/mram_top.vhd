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
-- in reset until the AXI side owns the backend (axi_owns): after
-- boot_done, after boot_fail (so PCI can inspect or reprogram the MRAM on a
-- failed board; the CPU stays in reset), or while boot_hold keeps the copy
-- from starting (debug). Release boot_hold only while no AXI access to
-- this slave is in flight: the copy then takes the backend over and the
-- AXI side goes back into reset. While the copy runs, AWREADY/ARREADY stay
-- low and AXI requests simply stall at the interconnect (roughly 9 s, or
-- up to roughly 35 s with retries -- see mram_boot_copy.vhd). Any PCI-side
-- completion timeout, and whether the host should instead wait for
-- boot_done before touching this window, must be handled at system level.
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
        G_CS_HIGH_WRITE_CYCLES : integer := 92;  -- tCS3 600ns at 150MHz
        G_POSTED_WRITES        : boolean := true; -- B as soon as the write data is held
        G_STREAM_WRITES        : boolean := true; -- one SPI write for contiguous AXI writes
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
    signal axi_owns, boot_fail_i, boot_held           : std_logic;

    signal boot_hold_sync, powerup_done               : std_logic;
    signal reg_cmd_valid, reg_cmd_accept, reg_cmd_done : std_logic;
    signal reg_cmd_op                                  : reg_cmd_t;
    signal reg_cmd_wdata, reg_cmd_addr                 : std_logic_vector(7 downto 0);
    signal reg_cmd_rdata                               : std_logic_vector(31 downto 0);

    signal backend_io_o, backend_io_oe, backend_io_i : std_logic_vector(3 downto 0);

begin

    boot_done <= boot_done_i;

    -- The AXI side owns the backend after the copy (pass or fail) or while
    -- the copy is held off; otherwise it stays in reset.
    axi_owns   <= boot_done_i or boot_fail_i or boot_held;
    axi_resetn <= aresetn and axi_owns;
    boot_fail  <= boot_fail_i;

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
            boot_fail    => boot_fail_i,
            boot_hold    => boot_hold_sync,
            powerup_done => powerup_done,
            boot_held    => boot_held
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

    -- Plain mux, not an arbiter: exactly one side owns the backend. The CPU
    -- and the AXI wrapper are held in reset whenever mram_boot_copy does.
    muxed_req <= axi_req when axi_owns = '1' else boot_req;

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
