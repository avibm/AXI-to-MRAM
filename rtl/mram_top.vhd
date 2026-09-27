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
-- selected mux (not an arbiter -- the CPU issues no AXI traffic while
-- held in reset, so there is never a cycle where both have a real request
-- at once), whose output passes through mram_write_guard before reaching
-- mram_qspi_backend. mram_boot_copy's own traffic is never actually
-- affected by the guard: its reads (from the protected master copy) are
-- always allowed, and its writes (to the unprotected working-copy region)
-- are always allowed -- the guard exists for AXI-side writes to the
-- protected region, which mram_boot_copy never attempts.
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
        G_ID_WIDTH : integer := C_AXI_ID_WIDTH
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

    signal backend_io_o, backend_io_oe, backend_io_i : std_logic_vector(3 downto 0);

begin

    boot_done <= boot_done_i;

    u_boot_copy : entity work.mram_boot_copy
        port map (
            aclk        => aclk,
            aresetn     => aresetn,
            core_req    => boot_req,
            core_resp   => guard_resp,
            cpu_reset_n => cpu_reset_n,
            boot_done   => boot_done_i,
            boot_fail   => boot_fail
        );

    u_axi_wrapper : entity work.axi4_slave_wrapper
        generic map (
            G_ID_WIDTH => G_ID_WIDTH
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
            core_req       => axi_req,
            core_resp      => guard_resp
        );

    -- Plain mux, not an arbiter -- the CPU (and therefore all normal AXI
    -- traffic) is held in reset for the entire time mram_boot_copy runs.
    muxed_req <= boot_req when boot_done_i = '0' else axi_req;

    u_write_guard : entity work.mram_write_guard
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
        port map (
            aclk       => aclk,
            aresetn    => aresetn,
            core_req   => guarded_req,
            core_resp  => backend_resp,
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
