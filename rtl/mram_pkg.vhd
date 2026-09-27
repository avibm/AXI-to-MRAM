--------------------------------------------------------------------------------
-- mram_pkg.vhd
--
-- Shared types and constants for the AS302G208 MRAM subsystem on RT
-- PolarFire (RTPF500TCG1509), behind the existing PF_SRAM_AHB_AXI-
-- compatible AXI4 slave interface.
--
-- AXI address/data widths are fixed system constants -- they match the
-- existing PF_SRAM_AHB_AXI interface (512-bit data, 128MB usable MRAM
-- address space) and are not meant to vary per instance.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package mram_pkg is

    ----------------------------------------------------------------------------
    -- System-fixed widths (match PF_SRAM_AHB_AXI)
    ----------------------------------------------------------------------------
    constant C_AXI_ADDR_WIDTH : integer := 27;                    -- 128MB usable MRAM space
    constant C_AXI_DATA_WIDTH : integer := 512;                   -- 512-bit AXI data bus
    constant C_AXI_STRB_WIDTH : integer := C_AXI_DATA_WIDTH / 8;  -- 64
    constant C_AXI_LEN_WIDTH  : integer := 8;                     -- AXI4 AWLEN/ARLEN
    constant C_AXI_ID_WIDTH   : integer := 6;                     -- matches the AXI interconnect

    constant C_MAX_OUTSTANDING : integer := 4; -- outstanding AXI read bursts tracked

    ----------------------------------------------------------------------------
    -- Core-side request/response interface between the AXI wrapper, the
    -- boot-copy sequencer, the write guard, and the MRAM backend. Operates
    -- at whatever granularity core_req.size specifies (1 to 64 bytes),
    -- byte-aligned to that size -- no line or beat concept is implied.
    --
    -- Handshake contract:
    --   * Requester drives req.valid and holds the record stable until
    --     resp.ready = '1' is seen in the same cycle (accepted).
    --   * resp.ready = '1' means "accepted this cycle", NOT "completed" --
    --     completion (rvalid / bvalid) may arrive on a later cycle.
    --   * resp.rvalid / resp.bvalid pulse for exactly one cycle when the
    --     previously accepted read/write completes, in the order requests
    --     were accepted (in-order completion).
    ----------------------------------------------------------------------------
    type core_req_t is record
        valid : std_logic;
        addr  : std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0); -- byte address, aligned to size
        we    : std_logic;                                        -- '1' = write, '0' = read
        size  : std_logic_vector(2 downto 0); -- AXI-style AxSIZE: 2**size bytes, 0..6 (1..64B)
        wdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        wstrb : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);
    end record;

    -- Common AxSIZE encodings for readability at call sites.
    constant SIZE_1B  : std_logic_vector(2 downto 0) := "000";
    constant SIZE_2B  : std_logic_vector(2 downto 0) := "001";
    constant SIZE_4B  : std_logic_vector(2 downto 0) := "010";
    constant SIZE_8B  : std_logic_vector(2 downto 0) := "011";
    constant SIZE_16B : std_logic_vector(2 downto 0) := "100";
    constant SIZE_32B : std_logic_vector(2 downto 0) := "101";
    constant SIZE_64B : std_logic_vector(2 downto 0) := "110"; -- full AXI beat width

    constant CORE_REQ_IDLE : core_req_t := (
        valid => '0',
        addr  => (others => '0'),
        we    => '0',
        size  => SIZE_64B,
        wdata => (others => '0'),
        wstrb => (others => '0')
    );

    type core_resp_t is record
        ready  : std_logic; -- core accepted core_req this cycle
        rvalid : std_logic; -- rdata valid, for a previously accepted read
        rdata  : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        bvalid : std_logic; -- write completed, for a previously accepted write
        error  : std_logic; -- reserved: uncorrectable MRAM / protocol error
    end record;

    constant CORE_RESP_IDLE : core_resp_t := (
        ready  => '0',
        rvalid => '0',
        rdata  => (others => '0'),
        bvalid => '0',
        error  => '0'
    );

    ----------------------------------------------------------------------------
    -- AXI4 response encodings (RRESP/BRESP)
    ----------------------------------------------------------------------------
    constant AXI_RESP_OKAY   : std_logic_vector(1 downto 0) := "00";
    constant AXI_RESP_SLVERR : std_logic_vector(1 downto 0) := "10";

    ----------------------------------------------------------------------------
    -- AXI4 burst type encodings (AWBURST/ARBURST) -- this design only
    -- implements INCR; FIXED/WRAP are flagged, not silently mishandled.
    -- See axi4_slave_wrapper.vhd.
    ----------------------------------------------------------------------------
    constant AXI_BURST_FIXED : std_logic_vector(1 downto 0) := "00";
    constant AXI_BURST_INCR  : std_logic_vector(1 downto 0) := "01";
    constant AXI_BURST_WRAP  : std_logic_vector(1 downto 0) := "10";

end package mram_pkg;
