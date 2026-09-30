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
-- Datasheet: Avalanche "1Gbit - 8Gbit Dual Quad SPI P-SRAM Memory"
-- (AS301G208/AS302G208/AS304G208/AS308G208), Rev. J.5. Table and figure
-- numbers below refer to that revision.
--
-- The AS302G208 package holds two independent 1Gb (128MB) quad-SPI dies,
-- each with its own CS#/CLK and IO[3:0] / IO[7:4]. This backend drives one
-- die (CS1#, CLK1, IO[3:0]); die 2 is unused and its CS2# must be held
-- high on the board.
--
-- Opcodes and framing (Table 29, Figure 19):
--   Write Enable   (WREN)  06h, (1-0-0), no address/data. Issued as its own
--     CS# transaction before every array write (CR1 default "Normal" WREN
--     mode: WREN clears when CS# rises after a write).
--   Read Quad I/O  (RDQI)  EBh, (1-4-4) SDR, 54MHz max: opcode on IO0,
--     4 address bytes on IO[3:0], 1 XIP mode byte, G_DUMMY_CYCLES latency
--     clocks, then data on IO[3:0].
--   Write Quad I/O (4WQIO) D2h, (1-4-4) SDR, 54MHz max: opcode on IO0,
--     4 address bytes, 1 XIP mode byte, then data.
--   The XIP mode byte is part of both instructions (Table 29 "XIP" column,
--   Figure 19): Axh would enter XIP (no opcode on later accesses), Fxh
--   keeps normal mode. This design always sends G_XIP_BYTE = FFh.
--
-- Latency: CR2[3:0] defaults to 8 cycles (Table 25), which is valid for
-- (1-4-4) SDR up to 54MHz (Table 26). G_DUMMY_CYCLES must equal CR2; CR2
-- is never written by this design.
--
-- Mode 0 (CPOL=0, CPHA=0), inputs latched on rising CLK, outputs change on
-- falling CLK, MSB first (Table 4, Table 7, "Instruction Description").
--
-- Device timing honoured here (Tables 10, 39, 41, 43), in aclk cycles, so
-- the defaults assume a 150MHz aclk -- rescale for another clock:
--   fCLK <= 54MHz SDR      -> G_SCLK_HALF_PERIOD >= 2 (37.5MHz at 150MHz)
--   tPU  >= 25ms           -> enforced by mram_boot_copy (G_POWERUP_CYCLES),
--                             which always issues the first request
--   tCSS >= 5ns, tCSH >= 4ns -> G_CS_SETUP_CYCLES (setup and hold)
--   tCS1 >= 20ns after read -> G_CS_HIGH_READ_CYCLES
--   tCS3 >= 600ns after array write -> G_CS_HIGH_WRITE_CYCLES
--   CS# high between WREN and the write -> G_CS_GAP_CYCLES
--
-- Address width note: core_req.addr is C_MRAM_ADDR_WIDTH = 27 bits (128MB = one 1Gb die),
-- zero-extended to the 4 address bytes RDQI/4WQIO require (Table 12).
--
-- SPI timing: every phase is counted in SCLK rising edges. The first
-- bit/nibble of a transaction is driven before the first rising edge; each
-- later one is driven on the falling edge after the rising edge that
-- consumed the previous one. Read data is sampled on the aclk edge that
-- raises SCLK, half an SCLK period after the device launched it on the
-- preceding falling edge. The device's output valid time is tCO <= 9ns
-- (Table 43); at 37.5MHz the half period is 13.3ns, leaving ~4ns for FPGA
-- clock-to-out, board delay both ways and input setup. That is tight:
-- constrain the I/O in timing analysis, or use G_SCLK_HALF_PERIOD = 3
-- (25MHz, 20ns half period) for more margin.
--
-- Byte lanes: the byte at address A is carried in lane A(5:0) of wdata /
-- rdata (see mram_pkg). The lowest-address byte is sent / received first.
--
-- Register commands (reg_cmd_* ports, driven by mram_cmd_ctrl): WREN 06h,
-- WRDI 04h, RDSR 05h, WRSR 01h, RDID 9Fh, all single-bit SPI (Table 29):
-- opcode on IO0, write data on IO0, read data from the device on IO1
-- (SO), MSB first. IO2 (WP#) and IO3 are driven high throughout. They run
-- between memory accesses and take priority over a waiting core_req.
-- RDSR and RDID are limited to 40MHz (Table 29); see the assertion below.
-- WRSR is followed by the array-write CS# high time, the others by the
-- read one.
--
-- Not implemented (documented scope):
--   * RDFSR, RDAR/WRAR (CR1/CR2 access).
--   * Driving the device RESET# pin (not on this entity's ports).
--   * Burst aggregation / prefetch across consecutive core_req calls.
--   * XIP mode (deliberately never entered: the mode byte is always Fxh).
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
        G_DUMMY_CYCLES         : integer := 8;          -- must equal CR2[3:0] (default 8, Table 25)
        G_SCLK_HALF_PERIOD     : integer := 2;          -- aclk cycles per SCLK half-period (>= 2 at 150MHz)
        G_CS_SETUP_CYCLES      : integer := 4;          -- aclk cycles CS# low before first edge / after last
        G_CS_GAP_CYCLES        : integer := 92;         -- aclk cycles CS# high between WREN and write
                                                        -- (613ns: the tCS3 value, as the datasheet
                                                        -- gives no figure after WREN)
        G_CS_HIGH_READ_CYCLES  : integer := 4;          -- min CS# high after a read  (tCS1 20ns  -> 27ns)
        G_CS_HIGH_WRITE_CYCLES : integer := 92;         -- min CS# high after a write (tCS3 600ns -> 613ns)
        G_XIP_BYTE          : std_logic_vector(7 downto 0) := x"FF"; -- XIP mode byte, Fxh = no XIP
        G_OPCODE_QUAD_READ  : std_logic_vector(7 downto 0) := x"EB"; -- RDQI,  Table 29 #19
        G_OPCODE_QUAD_WRITE : std_logic_vector(7 downto 0) := x"D2"; -- 4WQIO, Table 29 #24
        G_OPCODE_WREN       : std_logic_vector(7 downto 0) := x"06"; -- WREN,  Table 29 #2
        G_OPCODE_WRDI       : std_logic_vector(7 downto 0) := x"04"; -- WRDI,  Table 29 #3
        G_OPCODE_RDSR       : std_logic_vector(7 downto 0) := x"05"; -- RDSR,  Table 29 #6
        G_OPCODE_RDID       : std_logic_vector(7 downto 0) := x"9F"; -- RDID,  Table 29 #8
        G_OPCODE_WRSR       : std_logic_vector(7 downto 0) := x"01"; -- WRSR,  Table 29 #10
        G_ACLK_FREQ_HZ      : natural := 150_000_000   -- only used by the SCLK limit checks
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        core_req  : in  core_req_t;
        core_resp : out core_resp_t;

        -- Register command request (same valid/accept rule as core_req:
        -- hold valid and the fields until accept = '1' in the same cycle).
        reg_cmd_valid  : in  std_logic := '0';
        reg_cmd_op     : in  reg_cmd_t := REG_CMD_RDSR;
        reg_cmd_wdata  : in  std_logic_vector(7 downto 0) := (others => '0'); -- WRSR data
        reg_cmd_accept : out std_logic;
        reg_cmd_done   : out std_logic;                     -- one-cycle pulse
        reg_cmd_rdata  : out std_logic_vector(31 downto 0); -- RDSR: bits 7:0; RDID: all 32

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
        S_CS_SETUP, S_CMD, S_ADDR, S_XIP, S_DUMMY, S_DATA, S_CS_HOLD, S_DONE, S_CS_HIGH,
        S_REG_SETUP, S_REG_CMD, S_REG_WDATA, S_REG_RDATA
    );
    signal state : state_t := S_IDLE;

    function max(a, b : integer) return integer is
    begin
        if a > b then
            return a;
        end if;
        return b;
    end function;

    -- S_DONE and the accepting S_IDLE cycle also keep CS# high, so the
    -- S_CS_HIGH wait is two cycles shorter than the required high time.
    constant C_HIGH_WAIT_RD : natural := max(G_CS_HIGH_READ_CYCLES - 2, 0);
    constant C_HIGH_WAIT_WR : natural := max(G_CS_HIGH_WRITE_CYCLES - 2, 0);

    signal high_cnt    : natural range 0 to max(C_HIGH_WAIT_RD, C_HIGH_WAIT_WR) := 0;

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

    -- Register command in progress
    signal req_is_reg   : std_logic := '0';
    signal reg_wr_bits  : integer range 0 to 8;   -- data bits sent after the opcode
    signal reg_rd_bits  : integer range 0 to 32;  -- data bits received after the opcode
    signal reg_rx       : std_logic_vector(31 downto 0);
    signal reg_done_i   : std_logic := '0';

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

    assert G_ACLK_FREQ_HZ / (2 * G_SCLK_HALF_PERIOD) <= 40_000_000
        report "SCLK above 40MHz: too fast for RDSR/RDID (datasheet Table 29)" severity failure;
    assert G_ACLK_FREQ_HZ / (2 * G_SCLK_HALF_PERIOD) <= 54_000_000
        report "SCLK above 54MHz SDR maximum (datasheet Table 39)" severity failure;

    -- Register commands win over a waiting memory request.
    reg_cmd_accept <= reg_cmd_valid when state = S_IDLE else '0';
    reg_cmd_done   <= reg_done_i;
    reg_cmd_rdata  <= reg_rx;

    -- Accept combinationally whenever idle (see the mram_pkg contract).
    core_resp.ready  <= core_req.valid and not reg_cmd_valid when state = S_IDLE else '0';
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
                reg_done_i  <= '0';
                req_is_reg  <= '0';
            else
                resp_rvalid <= '0';
                resp_bvalid <= '0';
                reg_done_i  <= '0';

                case state is

                    ------------------------------------------------------------
                    -- Accept a request; writes detour through WREN first.
                    ------------------------------------------------------------
                    when S_IDLE =>
                        mram_cs_n <= '1';
                        sclk_run  <= false;
                        io_oe     <= (others => '0');
                        if reg_cmd_valid = '1' then
                            -- opcode, then (WRSR only) the data byte, MSB first
                            req_is_reg  <= '1';
                            reg_wr_bits <= 0;
                            reg_rd_bits <= 0;
                            req_we      <= '0'; -- selects the CS# high time afterwards
                            case reg_cmd_op is
                                when REG_CMD_WREN =>
                                    shreg(511 downto 504) <= G_OPCODE_WREN;
                                when REG_CMD_WRDI =>
                                    shreg(511 downto 504) <= G_OPCODE_WRDI;
                                when REG_CMD_RDSR =>
                                    shreg(511 downto 504) <= G_OPCODE_RDSR;
                                    reg_rd_bits <= 8;
                                when REG_CMD_WRSR =>
                                    shreg(511 downto 496) <= G_OPCODE_WRSR & reg_cmd_wdata;
                                    reg_wr_bits <= 8;
                                    req_we      <= '1';
                                when others => -- REG_CMD_RDID
                                    shreg(511 downto 504) <= G_OPCODE_RDID;
                                    reg_rd_bits <= 32;
                            end case;
                            reg_rx    <= (others => '0');
                            setup_cnt <= 0;
                            mram_cs_n <= '0';
                            state     <= S_REG_SETUP;
                        elsif core_req.valid = '1' then
                            req_is_reg  <= '0';
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
                                -- XIP mode byte, 2 quad clocks (Figure 19)
                                io_drive  <= G_XIP_BYTE(7 downto 4);
                                shreg(511 downto 508) <= G_XIP_BYTE(3 downto 0);
                                clks_left <= 2;
                                state     <= S_XIP;
                            else
                                io_drive <= shreg(511 downto 508);
                                shreg    <= shreg(507 downto 0) & "0000";
                            end if;
                        end if;

                    when S_XIP =>
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
                        -- Register commands keep IO0/IO2 (WP#)/IO3 driven until
                        -- CS# has been high for a while (tWPHD 20ns, Table 45);
                        -- the device never drives those lines in (1-0-x) mode.
                        if req_is_reg = '0' then
                            io_oe <= (others => '0');
                        end if;
                        if setup_cnt = G_CS_SETUP_CYCLES - 1 then
                            mram_cs_n <= '1';
                            state     <= S_DONE;
                            setup_cnt <= 0;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_DONE =>
                        if req_is_reg = '1' then
                            reg_done_i <= '1';
                        elsif req_we = '1' then
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
                        high_cnt <= 0;
                        state    <= S_CS_HIGH;

                    -- Minimum CS# high time before the next instruction:
                    -- tCS3 after an array write, tCS1 after a read.
                    when S_CS_HIGH =>
                        if (req_we = '1' and high_cnt >= C_HIGH_WAIT_WR) or
                           (req_we = '0' and high_cnt >= C_HIGH_WAIT_RD) then
                            io_oe <= (others => '0');
                            state <= S_IDLE;
                        else
                            high_cnt <= high_cnt + 1;
                        end if;

                    ------------------------------------------------------------
                    -- Register commands: single-bit SPI (1-0-0 / 1-0-1).
                    ------------------------------------------------------------
                    when S_REG_SETUP =>
                        if setup_cnt = G_CS_SETUP_CYCLES - 1 then
                            io_drive  <= "11" & '0' & shreg(511);
                            shreg     <= shreg(510 downto 0) & '0';
                            clks_left <= 8;
                            io_oe     <= "1101"; -- IO1 is the device's SO
                            sclk_run  <= true;
                            state     <= S_REG_CMD;
                        else
                            setup_cnt <= setup_cnt + 1;
                        end if;

                    when S_REG_CMD =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                        elsif sclk_fall then
                            if clks_left = 0 then
                                if reg_wr_bits /= 0 then
                                    io_drive(0) <= shreg(511);
                                    shreg       <= shreg(510 downto 0) & '0';
                                    clks_left   <= reg_wr_bits;
                                    state       <= S_REG_WDATA;
                                elsif reg_rd_bits /= 0 then
                                    -- device shifts out on IO1 from this edge on
                                    clks_left <= reg_rd_bits;
                                    state     <= S_REG_RDATA;
                                else
                                    sclk_run  <= false;
                                    setup_cnt <= 0;
                                    state     <= S_CS_HOLD;
                                end if;
                            else
                                io_drive(0) <= shreg(511);
                                shreg       <= shreg(510 downto 0) & '0';
                            end if;
                        end if;

                    when S_REG_WDATA =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                        elsif sclk_fall then
                            if clks_left = 0 then
                                sclk_run  <= false;
                                setup_cnt <= 0;
                                state     <= S_CS_HOLD;
                            else
                                io_drive(0) <= shreg(511);
                                shreg       <= shreg(510 downto 0) & '0';
                            end if;
                        end if;

                    when S_REG_RDATA =>
                        if sclk_rise then
                            clks_left <= clks_left - 1;
                            reg_rx    <= reg_rx(30 downto 0) & mram_io_i(1);
                        elsif sclk_fall and clks_left = 0 then
                            sclk_run  <= false;
                            setup_cnt <= 0;
                            state     <= S_CS_HOLD;
                        end if;

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
