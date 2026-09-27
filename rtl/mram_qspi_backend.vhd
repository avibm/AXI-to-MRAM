--------------------------------------------------------------------------------
-- mram_qspi_backend.vhd
--
-- Direct MRAM backend: services core_req_t/core_resp_t (see mram_pkg)
-- by driving the AS302G208 over Quad SPI directly, one request at a time.
-- No cache, no tag RAM, no line concept -- exactly core_req.size bytes are
-- moved per request, at core_req.addr, with identity addressing (AXI
-- address equals MRAM address across the full 128MB usable range). Boot-
-- region write protection lives in mram_write_guard.vhd, upstream of this
-- file -- this backend has no protection logic of its own and moves
-- whatever address it is given.
--
-- Opcodes and framing, per the AS302G208 datasheet's instruction table:
--   Write Enable   (WREN)  06h, framing (1-0-0), 0 address/data bytes.
--     Issued as its own complete CS# low->high transaction immediately
--     before every write (S_WREN_* states).
--   Read Quad I/O  (RDQI)  EBh, framing (1-4-4): opcode on IO0 only
--     (single-bit), address AND data on all four IOs. SDR, 108MHz max,
--     4 address bytes, latency/dummy cycles required, supports XIP.
--   Write Quad I/O (4WQIO) D2h, framing (1-4-4), SDR, 108MHz max,
--     4 address bytes, requires WREN immediately before it.
-- (DDR variants DRQI/EDh and 4DWQO/D1h exist at 54MHz but are not used
--  here; this design is SDR-only.)
--
-- Not yet confirmed against the datasheet:
--   G_DUMMY_CYCLES: latency/dummy cycles for Quad I/O reads are
--   configurable via Configuration Register 2 (CR2[3:0]) and frequency-
--   dependent, not a single fixed constant. This design assumes CR2 is
--   already configured elsewhere to match G_DUMMY_CYCLES at the chosen
--   SCLK frequency; configuring CR2 itself (via Write Any Register,
--   WRAR, 71h) is not implemented here.
--   Clock polarity/phase (mode 0) and wire byte-order (lowest-address
--   byte first, MSB-first per byte) are assumed, not confirmed.
--
-- Address width note: core_req.addr is 27 bits (128MB usable), zero-
-- extended to the 4 address bytes (32 bits) RDQI/4WQIO require.
--
-- Not implemented (documented scope):
--   * CR2 (latency) configuration / any register-write initialization.
--   * Burst aggregation / prefetch across consecutive core_req calls.
--   * XIP mode (RDQI supports it per the datasheet; unused here).
--   * Error/status reporting beyond core_resp.error tied low ('0').
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity mram_qspi_backend is
    generic (
        G_DUMMY_CYCLES      : integer := 8;              -- STILL PLACEHOLDER, see header
        G_SCLK_HALF_PERIOD  : integer := 2;               -- aclk cycles per SCLK half-period
        G_CS_SETUP_CYCLES   : integer := 4;               -- aclk cycles CS# low before first edge
        G_CS_GAP_CYCLES     : integer := 4;               -- aclk cycles CS# high between WREN and write
        G_OPCODE_QUAD_READ  : std_logic_vector(7 downto 0) := x"EB"; -- RDQI, confirmed, Table 32 #18
        G_OPCODE_QUAD_WRITE : std_logic_vector(7 downto 0) := x"D2"; -- 4WQIO, confirmed, Table 32 #23
        G_OPCODE_WREN       : std_logic_vector(7 downto 0) := x"06"  -- WREN, confirmed, Table 32 #2
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        core_req  : in  core_req_t;
        core_resp : out core_resp_t;

        mram_cs_n  : out std_logic;
        mram_sclk  : out std_logic;
        -- Drive/enable/sense split rather than a single inout, so the
        -- top-level integration owns the actual tri-state pins: o/oe drive
        -- the physical pins when oe is asserted, i senses their state
        -- regardless of which side (if any) is currently driving.
        mram_io_o  : out std_logic_vector(3 downto 0);
        mram_io_oe : out std_logic_vector(3 downto 0);
        mram_io_i  : in  std_logic_vector(3 downto 0)
    );
end entity mram_qspi_backend;

architecture rtl of mram_qspi_backend is

    type state_t is (
        S_IDLE,
        S_WREN_SETUP, S_WREN_CMD, S_WREN_GAP,
        S_CS_SETUP, S_CMD, S_ADDR, S_DUMMY, S_DATA, S_CS_HOLD, S_DONE
    );
    signal state : state_t := S_IDLE;

    -- Latched request
    signal req_addr32   : std_logic_vector(31 downto 0);
    signal req_we       : std_logic;
    signal size_bytes   : integer range 1 to 64;
    signal lane_offset  : integer range 0 to 63;

    -- Shared 512-bit shift register. During single-bit phases (WREN
    -- opcode, main opcode) only bit 511 (then shifted by 1) is used.
    -- During quad phases (address, data) the top 4 bits are used and the
    -- register shifts by 4.
    signal shreg : std_logic_vector(511 downto 0);

    signal tx_build : unsigned(511 downto 0);

    signal edges_left : integer range 0 to 520;

    signal sclk_reg     : std_logic := '0';
    signal sclk_div_cnt : integer range 0 to G_SCLK_HALF_PERIOD - 1 := 0;
    signal sclk_fall    : boolean;
    signal sclk_rise    : boolean;
    signal sclk_run     : boolean := false;
    signal gap_cnt       : integer range 0 to G_CS_GAP_CYCLES := 0;
    signal setup_cnt     : integer range 0 to G_CS_SETUP_CYCLES := 0;

    -- Per-line drive/tri-state so IO2/IO3 (WP#/HOLD#) can be held high
    -- independently of whatever IO0/IO1 are doing during single-bit phases.
    signal io_drive : std_logic_vector(3 downto 0);
    signal io_oe    : std_logic_vector(3 downto 0); -- '1' per bit = drive that line

begin

    process (aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                sclk_reg     <= '0';
                sclk_div_cnt <= 0;
            elsif sclk_run then
                if sclk_div_cnt = G_SCLK_HALF_PERIOD - 1 then
                    sclk_div_cnt <= 0;
                    sclk_reg     <= not sclk_reg;
                else
                    sclk_div_cnt <= sclk_div_cnt + 1;
                end if;
            else
                sclk_div_cnt <= 0;
                sclk_reg     <= '0'; -- idle low (mode 0, assumed -- see header)
            end if;
        end if;
    end process;

    -- Two pulses, each firing once per full SCLK period (not per half-
    -- period, which would double the effective data rate on these SDR-
    -- only commands): sclk_fall for driving new output data (mode 0:
    -- change on the falling edge), sclk_rise for sampling input data
    -- (mode 0: sample on the rising edge). Every state below uses exactly
    -- one of these.
    sclk_fall <= sclk_run and (sclk_div_cnt = G_SCLK_HALF_PERIOD - 1) and (sclk_reg = '1');
    sclk_rise <= sclk_run and (sclk_div_cnt = G_SCLK_HALF_PERIOD - 1) and (sclk_reg = '0');
    mram_sclk <= sclk_reg;

    mram_io_o  <= io_drive;
    mram_io_oe <= io_oe;

    process (aclk)
        variable v_shift_amt : integer;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state     <= S_IDLE;
                mram_cs_n <= '1';
                sclk_run  <= false;
                io_oe     <= (others => '0');
                core_resp <= CORE_RESP_IDLE;
            else
                core_resp.ready  <= '0';
                core_resp.rvalid <= '0';
                core_resp.bvalid <= '0';
                core_resp.error  <= '0';

                case state is

                    ------------------------------------------------------------
                    -- Accept a request; writes detour through WREN first.
                    ------------------------------------------------------------
                    when S_IDLE =>
                        mram_cs_n <= '1';
                        sclk_run  <= false;
                        io_oe     <= (others => '0');
                        if core_req.valid = '1' then
                            core_resp.ready <= '1';

                            req_we      <= core_req.we;
                            size_bytes  <= 2 ** to_integer(unsigned(core_req.size));
                            lane_offset <= to_integer(unsigned(core_req.addr(5 downto 0)));
                            req_addr32  <= std_logic_vector(resize(unsigned(core_req.addr), 32));

                            v_shift_amt := to_integer(unsigned(core_req.addr(5 downto 0))) * 8;
                            tx_build <= shift_left(
                                            shift_right(unsigned(core_req.wdata), v_shift_amt),
                                            512 - (2 ** to_integer(unsigned(core_req.size))) * 8);

                            setup_cnt <= 0;
                            if core_req.we = '1' then
                                mram_cs_n <= '0';
                                state     <= S_WREN_SETUP;
                            else
                                mram_cs_n <= '0';
                                state     <= S_CS_SETUP;
                            end if;
                        end if;

                    ------------------------------------------------------------
                    -- WREN: its own complete CS# transaction, single-bit,
                    -- opcode only (0 address bytes, 0 data bytes per Table 32).
                    ------------------------------------------------------------
                    when S_WREN_SETUP =>
                        if setup_cnt = G_CS_SETUP_CYCLES - 1 then
                            shreg(511 downto 504) <= G_OPCODE_WREN;
                            edges_left <= 8; -- single-bit: 8 edges for 8 bits
                            io_oe      <= "1101"; -- drive IO0, IO2, IO3; IO1 unused/input
                            io_drive(2) <= '1';    -- WP# deasserted
                            io_drive(3) <= '1';    -- HOLD# deasserted
                            sclk_run   <= true;
                            state      <= S_WREN_CMD;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_WREN_CMD =>
                        if sclk_fall then
                            io_drive(0) <= shreg(511);
                            shreg       <= shreg(510 downto 0) & '0';
                            if edges_left = 1 then
                                sclk_run  <= false;
                                mram_cs_n <= '1';
                                gap_cnt   <= 0;
                                state     <= S_WREN_GAP;
                            else
                                edges_left <= edges_left - 1;
                            end if;
                        end if;

                    when S_WREN_GAP =>
                        io_oe <= (others => '0');
                        if gap_cnt = G_CS_GAP_CYCLES - 1 then
                            mram_cs_n <= '0';
                            setup_cnt <= 0;
                            state     <= S_CS_SETUP;
                        else
                            gap_cnt <= gap_cnt + 1;
                        end if;

                    ------------------------------------------------------------
                    -- Main transaction: (1-4-4) framing -- opcode single-bit
                    -- on IO0, address and data on all four IOs.
                    ------------------------------------------------------------
                    when S_CS_SETUP =>
                        if setup_cnt = G_CS_SETUP_CYCLES - 1 then
                            if req_we = '1' then
                                shreg(511 downto 504) <= G_OPCODE_QUAD_WRITE;
                            else
                                shreg(511 downto 504) <= G_OPCODE_QUAD_READ;
                            end if;
                            edges_left  <= 8; -- single-bit opcode: 8 edges
                            io_oe       <= "1101";
                            io_drive(2) <= '1';
                            io_drive(3) <= '1';
                            sclk_run    <= true;
                            state       <= S_CMD;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_CMD =>
                        if sclk_fall then
                            io_drive(0) <= shreg(511);
                            shreg       <= shreg(510 downto 0) & '0';
                            if edges_left = 1 then
                                shreg(511 downto 480) <= req_addr32;
                                edges_left <= 8; -- 32 addr bits / 4 bits-per-edge (quad)
                                io_oe      <= "1111"; -- switch to quad drive for address
                                state      <= S_ADDR;
                            else
                                edges_left <= edges_left - 1;
                            end if;
                        end if;

                    when S_ADDR =>
                        if sclk_fall then
                            io_drive <= shreg(511 downto 508);
                            shreg    <= shreg(507 downto 0) & "0000";
                            if edges_left = 1 then
                                if req_we = '1' then
                                    shreg      <= std_logic_vector(tx_build);
                                    edges_left <= size_bytes * 2;
                                    io_oe      <= "1111";
                                    state      <= S_DATA;
                                elsif G_DUMMY_CYCLES = 0 then
                                    edges_left <= size_bytes * 2;
                                    io_oe      <= (others => '0');
                                    shreg      <= (others => '0');
                                    state      <= S_DATA;
                                else
                                    edges_left <= G_DUMMY_CYCLES;
                                    io_oe      <= (others => '0'); -- release during dummy
                                    state      <= S_DUMMY;
                                end if;
                            else
                                edges_left <= edges_left - 1;
                            end if;
                        end if;

                    when S_DUMMY =>
                        if sclk_fall then
                            if edges_left = 1 then
                                edges_left <= size_bytes * 2;
                                shreg      <= (others => '0');
                                state      <= S_DATA;
                            else
                                edges_left <= edges_left - 1;
                            end if;
                        end if;

                    when S_DATA =>
                        if req_we = '1' then
                            if sclk_fall then
                                io_drive <= shreg(511 downto 508);
                                shreg    <= shreg(507 downto 0) & "0000";
                                if edges_left = 1 then
                                    sclk_run <= false;
                                    state    <= S_CS_HOLD;
                                else
                                    edges_left <= edges_left - 1;
                                end if;
                            end if;
                        else
                            if sclk_rise then
                                shreg <= shreg(507 downto 0) & mram_io_i;
                                if edges_left = 1 then
                                    sclk_run <= false;
                                    state    <= S_CS_HOLD;
                                else
                                    edges_left <= edges_left - 1;
                                end if;
                            end if;
                        end if;

                    when S_CS_HOLD =>
                        io_oe <= (others => '0');
                        if setup_cnt = G_CS_SETUP_CYCLES - 1 then
                            mram_cs_n <= '1';
                            state     <= S_DONE;
                            setup_cnt <= 0;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_DONE =>
                        if req_we = '1' then
                            core_resp.bvalid <= '1';
                        else
                            core_resp.rdata  <= std_logic_vector(
                                                    shift_left(unsigned(shreg), lane_offset * 8));
                            core_resp.rvalid <= '1';
                        end if;
                        state <= S_IDLE;

                    when others =>
                        -- Defensive default for an unreachable/SEU-corrupted
                        -- state encoding: release the bus and deassert CS#
                        -- rather than risk driving mram_io while the device
                        -- might also be driving it (bus contention). This is
                        -- a VHDL-level backstop; it does not by itself
                        -- guarantee recovery without your synthesis tool's
                        -- illegal-state-recovery ("safe FSM") support -- see
                        -- mram_boot_copy.vhd's header note.
                        mram_cs_n <= '1';
                        sclk_run  <= false;
                        io_oe     <= (others => '0');
                        state     <= S_IDLE;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
