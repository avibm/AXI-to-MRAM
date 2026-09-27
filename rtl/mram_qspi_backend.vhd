--------------------------------------------------------------------------------
-- mram_qspi_backend.vhd
--
-- Direct MRAM backend: services core_req_t/core_resp_t (see mram_pkg)
-- by driving the AS302G208 over Quad SPI directly, one request at a time.
-- No cache, no tag RAM, no line concept -- exactly core_req.nbytes bytes are
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
--   Whether RDQI carries "mode"/XIP-continuation bits in its first dummy
--   clocks is not confirmed. This design releases IO0-3 for the whole
--   dummy phase; if the device samples mode bits there, floating lines
--   could select continuous-read (XIP) mode -- check the datasheet and, if
--   needed, drive a non-XIP mode value during those clocks.
--
-- Address width note: core_req.addr is 27 bits (128MB usable), zero-
-- extended to the 4 address bytes (32 bits) RDQI/4WQIO require.
--
-- SPI timing (mode 0): every phase is counted in SCLK rising edges. The
-- first bit/nibble of a transaction is driven before the first rising
-- edge; each later bit/nibble is driven on the falling edge that follows
-- the rising edge which consumed the previous one. Read data is sampled on
-- the aclk edge that raises SCLK, i.e. half an SCLK period after the device
-- launched it on the preceding falling edge. At G_SCLK_HALF_PERIOD = 1
-- (75MHz from a 150MHz aclk) that leaves ~6.7ns for FPGA clock-to-out +
-- board round trip + device output-valid + input setup, which is unlikely
-- to close; the default of 2 (37.5MHz) leaves ~13.3ns. Close it with real
-- I/O constraints in timing analysis.
--
-- Byte lanes: the byte at address A is carried in lane A(5:0) of wdata /
-- rdata (see mram_pkg). The lowest-address byte is sent / received first.
--
-- Not implemented (documented scope):
--   * CR2 (latency) configuration / any register-write initialization.
--   * Device power-up delay before the first command (datasheet tPU) --
--     aresetn must be held until the device is ready.
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

    -- Byte-order reversal of a 512-bit word: byte k <-> byte 63-k. Pure
    -- wiring, no logic.
    function byte_rev(v : std_logic_vector(511 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(511 downto 0);
    begin
        for k in 0 to 63 loop
            r(8 * (63 - k) + 7 downto 8 * (63 - k)) := v(8 * k + 7 downto 8 * k);
        end loop;
        return r;
    end function;

    -- Latched request
    signal req_addr32   : std_logic_vector(31 downto 0);
    signal req_we       : std_logic;
    signal req_nbytes   : unsigned(6 downto 0);
    signal lane_offset  : unsigned(5 downto 0);

    -- Shared 512-bit shift register. During single-bit phases (WREN
    -- opcode, main opcode) bit 511 is on IO0 and the register shifts by 1.
    -- During quad phases (address, write data) the top 4 bits are on IO3..0
    -- and the register shifts by 4. Read data shifts in at the bottom.
    signal shreg : std_logic_vector(511 downto 0);

    -- Write data with the first byte (lane lane_offset) moved to the top.
    signal tx_build : std_logic_vector(511 downto 0);

    -- SCLK rising edges still to come in the current phase.
    signal clks_left : integer range 0 to 128;

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

    signal resp_rvalid : std_logic := '0';
    signal resp_bvalid : std_logic := '0';
    signal resp_rdata  : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);

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

    -- Single-cycle strobes on the aclk edge at which SCLK rises / falls.
    -- sclk_rise: the device samples inputs, and this design samples read
    -- data. sclk_fall: this design changes its outputs.
    sclk_fall <= sclk_run and (sclk_div_cnt = G_SCLK_HALF_PERIOD - 1) and (sclk_reg = '1');
    sclk_rise <= sclk_run and (sclk_div_cnt = G_SCLK_HALF_PERIOD - 1) and (sclk_reg = '0');
    mram_sclk <= sclk_reg;

    mram_io_o  <= io_drive;
    mram_io_oe <= io_oe;

    -- Accept combinationally whenever idle (see the mram_pkg contract).
    core_resp.ready  <= core_req.valid when state = S_IDLE else '0';
    core_resp.rvalid <= resp_rvalid;
    core_resp.bvalid <= resp_bvalid;
    core_resp.rdata  <= resp_rdata;
    core_resp.error  <= '0';

    process (aclk)
        variable v_off : natural range 0 to 63;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state       <= S_IDLE;
                mram_cs_n   <= '1';
                sclk_run    <= false;
                io_oe       <= (others => '0');
                resp_rvalid <= '0';
                resp_bvalid <= '0';
            else
                resp_rvalid <= '0';
                resp_bvalid <= '0';

                case state is

                    ------------------------------------------------------------
                    -- Accept a request; writes detour through WREN first.
                    ------------------------------------------------------------
                    when S_IDLE =>
                        mram_cs_n <= '1';
                        sclk_run  <= false;
                        io_oe     <= (others => '0');
                        if core_req.valid = '1' then
                            req_we      <= core_req.we;
                            req_nbytes  <= core_req.nbytes;
                            lane_offset <= unsigned(core_req.addr(5 downto 0));
                            req_addr32  <= std_logic_vector(resize(unsigned(core_req.addr), 32));

                            -- Byte-reverse so lane 0 is at the top, then shift
                            -- the first byte (lane addr(5:0)) up to bits 511:504.
                            v_off    := to_integer(unsigned(core_req.addr(5 downto 0)));
                            tx_build <= std_logic_vector(shift_left(
                                            unsigned(byte_rev(core_req.wdata)), 8 * v_off));

                            setup_cnt <= 0;
                            mram_cs_n <= '0';
                            if core_req.we = '1' then
                                state <= S_WREN_SETUP;
                            else
                                state <= S_CS_SETUP;
                            end if;
                        end if;

                    ------------------------------------------------------------
                    -- WREN: its own complete CS# transaction, single-bit,
                    -- opcode only (0 address bytes, 0 data bytes per Table 32).
                    ------------------------------------------------------------
                    when S_WREN_SETUP =>
                        if setup_cnt = G_CS_SETUP_CYCLES - 1 then
                            -- first opcode bit set up before the first edge
                            io_drive    <= "11" & '0' & G_OPCODE_WREN(7);
                            shreg(511 downto 504) <= G_OPCODE_WREN(6 downto 0) & '0';
                            clks_left   <= 8;
                            io_oe       <= "1101"; -- drive IO0, IO2 (WP#), IO3 (HOLD#)
                            sclk_run    <= true;
                            state       <= S_WREN_CMD;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_WREN_CMD =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                        elsif sclk_fall then
                            if clks_left = 0 then
                                sclk_run  <= false;   -- SCLK ends low on this edge
                                mram_cs_n <= '1';
                                io_oe     <= (others => '0');
                                gap_cnt   <= 0;
                                state     <= S_WREN_GAP;
                            else
                                io_drive(0) <= shreg(511);
                                shreg       <= shreg(510 downto 0) & '0';
                            end if;
                        end if;

                    when S_WREN_GAP =>
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
                                io_drive <= "11" & '0' & G_OPCODE_QUAD_WRITE(7);
                                shreg(511 downto 504) <= G_OPCODE_QUAD_WRITE(6 downto 0) & '0';
                            else
                                io_drive <= "11" & '0' & G_OPCODE_QUAD_READ(7);
                                shreg(511 downto 504) <= G_OPCODE_QUAD_READ(6 downto 0) & '0';
                            end if;
                            clks_left <= 8;
                            io_oe     <= "1101";
                            sclk_run  <= true;
                            state     <= S_CMD;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_CMD =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                        elsif sclk_fall then
                            if clks_left = 0 then
                                -- opcode done: first address nibble, quad drive
                                io_drive  <= req_addr32(31 downto 28);
                                shreg(511 downto 484) <= req_addr32(27 downto 0);
                                io_oe     <= "1111";
                                clks_left <= 8; -- 32 addr bits / 4 bits per clock
                                state     <= S_ADDR;
                            else
                                io_drive(0) <= shreg(511);
                                shreg       <= shreg(510 downto 0) & '0';
                            end if;
                        end if;

                    when S_ADDR =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                        elsif sclk_fall then
                            if clks_left = 0 then
                                if req_we = '1' then
                                    io_drive  <= tx_build(511 downto 508);
                                    shreg     <= tx_build(507 downto 0) & "0000";
                                    clks_left <= 2 * to_integer(req_nbytes);
                                    state     <= S_DATA;
                                elsif G_DUMMY_CYCLES = 0 then
                                    io_oe     <= (others => '0');
                                    clks_left <= 2 * to_integer(req_nbytes);
                                    state     <= S_DATA;
                                else
                                    io_oe     <= (others => '0'); -- release for turnaround
                                    clks_left <= G_DUMMY_CYCLES;
                                    state     <= S_DUMMY;
                                end if;
                            else
                                io_drive <= shreg(511 downto 508);
                                shreg    <= shreg(507 downto 0) & "0000";
                            end if;
                        end if;

                    when S_DUMMY =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                        elsif sclk_fall and clks_left = 0 then
                            -- device drives the first data nibble after this edge
                            clks_left <= 2 * to_integer(req_nbytes);
                            state     <= S_DATA;
                        end if;

                    when S_DATA =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                            if req_we = '0' then
                                shreg <= shreg(507 downto 0) & mram_io_i;
                            end if;
                        elsif sclk_fall then
                            if clks_left = 0 then
                                sclk_run  <= false;
                                io_oe     <= (others => '0');
                                setup_cnt <= 0;
                                state     <= S_CS_HOLD;
                            elsif req_we = '1' then
                                io_drive <= shreg(511 downto 508);
                                shreg    <= shreg(507 downto 0) & "0000";
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
                            resp_bvalid <= '1';
                        else
                            -- Received bytes sit in shreg(8*n-1:0), first byte
                            -- highest. After byte_rev the first byte is in
                            -- lane 64-n; shift it down to lane lane_offset.
                            resp_rdata  <= std_logic_vector(shift_right(
                                               unsigned(byte_rev(shreg)),
                                               8 * (64 - to_integer(req_nbytes)
                                                       - to_integer(lane_offset))));
                            resp_rvalid <= '1';
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
