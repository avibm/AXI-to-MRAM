--------------------------------------------------------------------------------
-- mram_pkg.vhd
--
-- Shared types and constants for the AS302G208 MRAM subsystem on RT
-- PolarFire (RTPF500TCG1509), behind the existing PF_SRAM_AHB_AXI-
-- compatible AXI4 slave interface.
--
-- AXI address/data/ID widths are system constants that match the AXI
-- interconnect (32-bit address, 5-bit ID). The data width
-- C_AXI_DATA_WIDTH is set here: 64 (default), 128, 256 or 512. Everything
-- else (byte lanes, largest AxSIZE, request size) follows from it. The MRAM itself
-- is addressed with C_MRAM_ADDR_WIDTH bits (128MB = one 1Gb die); the AXI
-- wrapper uses only the low C_MRAM_ADDR_WIDTH bits of AxADDR, i.e. the
-- byte offset inside this slave's 128MB window. Address decoding (which
-- upper-bit values select this slave) is the interconnect's job.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package mram_pkg is

    ----------------------------------------------------------------------------
    -- System widths
    ----------------------------------------------------------------------------
    constant C_AXI_ADDR_WIDTH : integer := 32;                    -- matches the AXI interconnect
    constant C_MRAM_ADDR_WIDTH : integer := 27;                   -- 128MB usable MRAM space
    constant C_AXI_DATA_WIDTH : integer := 64;                    -- 64, 128, 256 or 512
    constant C_AXI_STRB_WIDTH : integer := C_AXI_DATA_WIDTH / 8;
    constant C_BEAT_BYTES     : integer := C_AXI_DATA_WIDTH / 8;  -- bytes per data beat
    -- log2(C_BEAT_BYTES): byte-lane address bits, and the largest AxSIZE
    constant C_LANE_BITS      : integer := 3 * boolean'pos(C_BEAT_BYTES = 8)
                                         + 4 * boolean'pos(C_BEAT_BYTES = 16)
                                         + 5 * boolean'pos(C_BEAT_BYTES = 32)
                                         + 6 * boolean'pos(C_BEAT_BYTES = 64);
    constant C_AXI_LEN_WIDTH  : integer := 8;                     -- AXI4 AWLEN/ARLEN
    constant C_AXI_ID_WIDTH   : integer := 5;                     -- matches the AXI interconnect

    constant C_MAX_OUTSTANDING : integer := 4; -- outstanding AXI read bursts tracked

    ----------------------------------------------------------------------------
    -- Core-side request/response interface between the AXI wrapper, the
    -- boot-copy sequencer, the write guard, and the MRAM backend.
    --
    -- A request moves core_req.nbytes bytes (1 to C_BEAT_BYTES) starting at
    -- byte address core_req.addr. The whole request must lie inside one
    -- C_BEAT_BYTES-aligned window: addr(C_LANE_BITS-1:0) + nbytes <=
    -- C_BEAT_BYTES. Data is lane-aligned: the byte at address A travels in
    -- byte lane A(C_LANE_BITS-1:0) of wdata/rdata, exactly as on the AXI
    -- data bus. There is no byte strobe on this interface --
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
    --   * Requesters may present the next request before the previous one
    --     has completed (the backend then streams contiguous accesses, see
    --     mram_qspi_backend). A requester must be able to take every
    --     rvalid/bvalid it has requests outstanding for: there is no
    --     back-pressure on rvalid/bvalid.
    ----------------------------------------------------------------------------
    -- cont = '1' marks a write that directly follows the previous write
    -- (addr = previous addr + nbytes). The backend may then append its data
    -- to the write still in progress instead of starting a new SPI write;
    -- the request is accepted only while that is possible, otherwise it is
    -- served as an ordinary write once the backend is idle.
    type core_req_t is record
        valid  : std_logic;
        addr   : std_logic_vector(C_MRAM_ADDR_WIDTH - 1 downto 0); -- first byte address (MRAM)
        we     : std_logic;                                        -- '1' = write, '0' = read
        nbytes : unsigned(6 downto 0);                             -- 1..C_BEAT_BYTES
        wdata  : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);  -- lane-aligned
        cont   : std_logic;                                        -- see above
    end record;

    -- AXI AxSIZE encodings, used by axi4_slave_wrapper.
    constant SIZE_1B  : std_logic_vector(2 downto 0) := "000";
    constant SIZE_2B  : std_logic_vector(2 downto 0) := "001";
    constant SIZE_4B  : std_logic_vector(2 downto 0) := "010";
    constant SIZE_8B  : std_logic_vector(2 downto 0) := "011";
    constant SIZE_16B : std_logic_vector(2 downto 0) := "100";
    constant SIZE_32B : std_logic_vector(2 downto 0) := "101";
    constant SIZE_64B : std_logic_vector(2 downto 0) := "110";
    constant SIZE_BEAT : std_logic_vector(2 downto 0) :=
        std_logic_vector(to_unsigned(C_LANE_BITS, 3));              -- full AXI beat width

    constant CORE_REQ_IDLE : core_req_t := (
        valid  => '0',
        addr   => (others => '0'),
        we     => '0',
        nbytes => to_unsigned(C_BEAT_BYTES, 7),
        wdata  => (others => '0'),
        cont   => '0'
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
    -- MRAM register commands (single-bit SPI, datasheet Table 29), issued by
    -- mram_cmd_ctrl and executed by mram_qspi_backend between memory
    -- accesses.
    ----------------------------------------------------------------------------
    subtype reg_cmd_t is std_logic_vector(2 downto 0);
    constant REG_CMD_WREN : reg_cmd_t := "000"; -- 06h Write Enable        (1-0-0)
    constant REG_CMD_WRDI : reg_cmd_t := "001"; -- 04h Write Disable       (1-0-0)
    constant REG_CMD_RDSR : reg_cmd_t := "010"; -- 05h Read Status Reg     (1-0-1), 1 byte out
    constant REG_CMD_WRSR : reg_cmd_t := "011"; -- 01h Write Status Reg    (1-0-1), 1 byte in, needs WREN
    constant REG_CMD_RDID : reg_cmd_t := "100"; -- 9Fh Read Device ID      (1-0-1), 4 bytes out
    constant REG_CMD_RDAR : reg_cmd_t := "101"; -- 65h Read Any Register   (1-1-1), 4 addr bytes,
                                                --     CR2 latency, 1 byte out
    constant REG_CMD_WRAR : reg_cmd_t := "110"; -- 71h Write Any Register  (1-1-1), 4 addr bytes,
                                                --     1 byte in, needs WREN

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
