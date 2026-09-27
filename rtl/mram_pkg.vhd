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
    -- boot-copy sequencer, the write guard, and the MRAM backend.
    --
    -- A request moves core_req.nbytes bytes (1 to 64) starting at byte
    -- address core_req.addr. The whole request must lie inside one 64-byte
    -- aligned window: addr(5:0) + nbytes <= 64. Data is lane-aligned: the
    -- byte at address A travels in byte lane A(5:0) of wdata/rdata, exactly
    -- as on the AXI data bus. There is no byte strobe on this interface --
    -- every one of the nbytes bytes is written. axi4_slave_wrapper turns
    -- AXI WSTRB into one request per contiguous run of enabled bytes.
    --
    -- Handshake contract:
    --   * Requester drives req.valid and holds the whole record stable until
    --     it sees resp.ready = '1' in the same cycle (accepted). It must not
    --     drop valid before that.
    --   * resp.ready = '1' means "accepted this cycle", NOT "completed". The
    --     responder drives ready combinationally from valid, so acceptance
    --     is visible to the requester in the cycle it is presented.
    --   * resp.rvalid / resp.bvalid pulse for exactly one cycle when the
    --     previously accepted read/write completes, in the order requests
    --     were accepted (in-order completion). resp.error qualifies the
    --     rvalid/bvalid pulse it accompanies.
    ----------------------------------------------------------------------------
    type core_req_t is record
        valid  : std_logic;
        addr   : std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0); -- first byte address
        we     : std_logic;                                        -- '1' = write, '0' = read
        nbytes : unsigned(6 downto 0);                             -- 1..64 bytes
        wdata  : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);  -- lane-aligned
    end record;

    -- AXI AxSIZE encodings, used by axi4_slave_wrapper.
    constant SIZE_1B  : std_logic_vector(2 downto 0) := "000";
    constant SIZE_2B  : std_logic_vector(2 downto 0) := "001";
    constant SIZE_4B  : std_logic_vector(2 downto 0) := "010";
    constant SIZE_8B  : std_logic_vector(2 downto 0) := "011";
    constant SIZE_16B : std_logic_vector(2 downto 0) := "100";
    constant SIZE_32B : std_logic_vector(2 downto 0) := "101";
    constant SIZE_64B : std_logic_vector(2 downto 0) := "110"; -- full AXI beat width

    constant CORE_REQ_IDLE : core_req_t := (
        valid  => '0',
        addr   => (others => '0'),
        we     => '0',
        nbytes => to_unsigned(64, 7),
        wdata  => (others => '0')
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
