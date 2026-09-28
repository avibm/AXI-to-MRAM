--------------------------------------------------------------------------------
-- mram_boot_copy.vhd
--
-- Copies the 32MB master boot image to address 0 at reset and holds the
-- CPU in reset until the copy is verified. Needed because the CPU's own
-- runtime writes to general memory would otherwise leave stale or
-- corrupted data at address 0 by the next boot; the master copy itself
-- lives elsewhere in MRAM, protected from ordinary AXI writes by
-- mram_write_guard.vhd, and is refreshed into the working copy here.
--
-- Robustness features, given a one-minute boot budget. One copy+verify
-- pass moves 4 x 32MB over the QSPI bus (read src, write dst, re-read src,
-- re-read dst) in 64-byte transactions; at 37.5MHz SCLK (150MHz aclk,
-- G_SCLK_HALF_PERIOD = 2) that is roughly 9 s per pass and roughly 35-36 s
-- if all G_MAX_RETRIES retries are used (4 passes). These are hand
-- estimates from the transaction framing, not measurements:
--   1. Read-back verification: every chunk is read back from both source
--      and destination and compared before the CPU is released. A
--      mismatch retries the whole copy, up to G_MAX_RETRIES times, rather
--      than trusting an unverified write.
--   2. A watchdog: if the FSM makes no progress for G_WATCHDOG_LIMIT
--      cycles, it is treated as a hang and forces a retry rather than
--      deadlocking forever.
--   3. After G_MAX_RETRIES failed attempts, boot_fail asserts and
--      cpu_reset_n stays low permanently -- the CPU never runs from data
--      that could not be verified. Wire boot_fail to whatever fault
--      indication (LED, telemetry bit, external watchdog that forces a
--      power-cycle) the system has; none is assumed here.
--   4. The default SCLK divider is deliberately conservative rather than
--      pushed toward the datasheet's 54MHz SDR ceiling, trading unused boot
--      time budget for timing margin.
--   5. An explicit "when others" default on the state case, alongside
--      whatever "safe FSM" / illegal-state-recovery option the synthesis
--      tool offers -- see Reliability below; this VHDL-level default does
--      not by itself guarantee safe recovery from an SEU-corrupted state
--      register without that tool support.
--
-- Reliability, quad vs. single-bit SPI:
--   * Single-bit SPI's SI is always an FPGA output and SO is always an
--     FPGA input; there is no shared bidirectional line, so pin-level bus
--     contention (both the FPGA and the MRAM driving the same line) is
--     structurally impossible. Quad I/O's four lines are bidirectional and
--     tri-stated, switching direction by phase (address/data out, dummy/
--     read-data in) -- an SEU that corrupts the output-enable control at
--     the wrong moment, or a timing/protocol mismatch, can cause
--     contention.
--   * More FSM states and control logic (tri-state control, byte-lane
--     shifting, WREN sequencing) than a minimal single-bit reader means a
--     larger SEU cross-section. The underlying AS302G208 silicon is
--     itself qualified for quad operation in this environment, so the
--     added risk is in the FPGA-side control logic around it, not in the
--     chip's quad mode itself.
--   Mitigations 1-5 above target these mechanisms (contention via safe-
--   state defaults, everything else via verify + watchdog + margin). Two
--   further items are the synthesis tool's responsibility, not
--   addressable from this file: enabling illegal-state-recovery support
--   for this FSM, and applying TMR if that is standard practice elsewhere
--   in this design.
--
-- Known limitations:
--   * Verification compares destination against source only. A corrupted
--     master copy is copied and "verified" faithfully; there is no CRC or
--     signature check of the image itself.
--   * A watchdog-forced retry does not wait for a request that is still in
--     flight in mram_qspi_backend. If the backend is merely slow (not hung)
--     its late rvalid/bvalid could be taken as the answer to the retry's
--     first request. G_WATCHDOG_LIMIT is far above a normal request's
--     duration (~650 aclk cycles for 64 bytes at the default divider), so
--     this only matters after a real fault.
--   * The copy runs on aresetn only. A CPU-only reset that does not also
--     assert aresetn does not refresh the working copy.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity mram_boot_copy is
    generic (
        G_SRC_BASE      : natural := 100663296; -- 0x6000000: pristine master copy (top 32MB)
        G_DST_BASE      : natural := 0;         -- 0x0: working copy, matches NOEL-V's fixed reset vector
        G_COPY_SIZE     : natural := 33554432;  -- 32MB
        G_CHUNK_BYTES   : natural := 64;         -- bytes per request, 1..64; must divide the
                                                 -- bases and G_COPY_SIZE (64-byte window rule)
        G_MAX_RETRIES   : natural := 3;
        G_WATCHDOG_LIMIT: natural := 1_000_000;  -- aclk cycles with no state change = hang
        G_POWERUP_CYCLES: natural := 3_750_000   -- MRAM tPU (25ms at 150MHz) before the first
                                                 -- command; the -A variant needs only 1ms
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic; -- system/global reset, NOT the gated CPU reset this drives

        core_req  : out core_req_t;  -- to mram_qspi_backend (via the mux in mram_top)
        core_resp : in  core_resp_t; -- from mram_qspi_backend

        cpu_reset_n : out std_logic; -- held low until copy AND verify succeed
        boot_done   : out std_logic; -- also used as the core_req mux select in mram_top
        boot_fail   : out std_logic  -- asserted, and stays asserted, after G_MAX_RETRIES failures
    );
end entity mram_boot_copy;

architecture rtl of mram_boot_copy is

    type state_t is (
        S_POWERUP,
        S_COPY_READ_ISSUE, S_COPY_READ_WAIT, S_COPY_WRITE_ISSUE, S_COPY_WRITE_WAIT,
        S_VERIFY_SRC_ISSUE, S_VERIFY_SRC_WAIT, S_VERIFY_DST_ISSUE, S_VERIFY_DST_WAIT,
        S_VERIFY_COMPARE,
        S_RETRY_CHECK,
        S_DONE, S_FAIL
    );
    signal state      : state_t := S_POWERUP;
    signal prev_state : state_t := S_POWERUP;
    signal powerup_cnt : natural range 0 to G_POWERUP_CYCLES := 0;

    -- 26 bits comfortably covers a 32MB (2**25) offset with headroom.
    signal byte_off   : unsigned(25 downto 0) := (others => '0');
    signal hold_data  : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal verify_src : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal mismatch   : std_logic := '0';

    signal retry_count : integer range 0 to G_MAX_RETRIES := 0;
    signal watchdog_cnt : integer range 0 to G_WATCHDOG_LIMIT := 0;
    signal watchdog_trip : std_logic;

begin

    watchdog_trip <= '1' when watchdog_cnt = G_WATCHDOG_LIMIT else '0';

    process (aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state        <= S_POWERUP;
                prev_state   <= S_POWERUP;
                powerup_cnt  <= 0;
                byte_off     <= (others => '0');
                retry_count  <= 0;
                cpu_reset_n  <= '0';
                boot_done    <= '0';
                boot_fail    <= '0';
                mismatch     <= '0';
                watchdog_cnt <= 0;
                core_req     <= CORE_REQ_IDLE;
            else
                prev_state <= state;

                -- Watchdog: reset on any state change (progress); force a
                -- retry attempt if nothing has moved for G_WATCHDOG_LIMIT
                -- cycles, rather than hanging forever.
                if state /= prev_state or state = S_POWERUP then
                    watchdog_cnt <= 0;
                elsif watchdog_cnt < G_WATCHDOG_LIMIT then
                    watchdog_cnt <= watchdog_cnt + 1;
                end if;

                if watchdog_trip = '1' and state /= S_DONE and state /= S_FAIL
                   and state /= S_POWERUP then
                    core_req.valid <= '0';
                    state          <= S_RETRY_CHECK;
                else

                case state is

                    -- Datasheet tPU: no instruction to the MRAM until it has
                    -- powered up. Assumes aresetn is released no earlier than
                    -- MRAM power-up; if not, lengthen G_POWERUP_CYCLES.
                    when S_POWERUP =>
                        if powerup_cnt >= G_POWERUP_CYCLES then
                            state <= S_COPY_READ_ISSUE;
                        else
                            powerup_cnt <= powerup_cnt + 1;
                        end if;

                    when S_COPY_READ_ISSUE =>
                        if core_req.valid = '1' and core_resp.ready = '1' then
                            core_req.valid <= '0'; -- accepted
                            state          <= S_COPY_READ_WAIT;
                        else
                            core_req.valid  <= '1'; -- hold until accepted
                            core_req.addr   <= std_logic_vector(
                                                   resize(byte_off, C_AXI_ADDR_WIDTH) + G_SRC_BASE);
                            core_req.we     <= '0';
                            core_req.nbytes <= to_unsigned(G_CHUNK_BYTES, 7);
                        end if;

                    when S_COPY_READ_WAIT =>
                        if core_resp.rvalid = '1' then
                            hold_data <= core_resp.rdata;
                            state     <= S_COPY_WRITE_ISSUE;
                        end if;

                    when S_COPY_WRITE_ISSUE =>
                        if core_req.valid = '1' and core_resp.ready = '1' then
                            core_req.valid <= '0'; -- accepted
                            state          <= S_COPY_WRITE_WAIT;
                        else
                            core_req.valid  <= '1'; -- hold until accepted
                            core_req.addr   <= std_logic_vector(
                                                   resize(byte_off, C_AXI_ADDR_WIDTH) + G_DST_BASE);
                            core_req.we     <= '1';
                            core_req.nbytes <= to_unsigned(G_CHUNK_BYTES, 7);
                            core_req.wdata  <= hold_data;
                        end if;

                    when S_COPY_WRITE_WAIT =>
                        if core_resp.bvalid = '1' then
                            if byte_off + G_CHUNK_BYTES >= G_COPY_SIZE then
                                byte_off <= (others => '0');
                                state    <= S_VERIFY_SRC_ISSUE;
                            else
                                byte_off <= byte_off + G_CHUNK_BYTES;
                                state    <= S_COPY_READ_ISSUE;
                            end if;
                        end if;

                    ------------------------------------------------------------
                    -- Verify pass: re-read both sides of every chunk and
                    -- compare before trusting the copy.
                    ------------------------------------------------------------
                    when S_VERIFY_SRC_ISSUE =>
                        if core_req.valid = '1' and core_resp.ready = '1' then
                            core_req.valid <= '0'; -- accepted
                            state          <= S_VERIFY_SRC_WAIT;
                        else
                            core_req.valid  <= '1'; -- hold until accepted
                            core_req.addr   <= std_logic_vector(
                                                   resize(byte_off, C_AXI_ADDR_WIDTH) + G_SRC_BASE);
                            core_req.we     <= '0';
                            core_req.nbytes <= to_unsigned(G_CHUNK_BYTES, 7);
                        end if;

                    when S_VERIFY_SRC_WAIT =>
                        if core_resp.rvalid = '1' then
                            verify_src <= core_resp.rdata;
                            state      <= S_VERIFY_DST_ISSUE;
                        end if;

                    when S_VERIFY_DST_ISSUE =>
                        if core_req.valid = '1' and core_resp.ready = '1' then
                            core_req.valid <= '0'; -- accepted
                            state          <= S_VERIFY_DST_WAIT;
                        else
                            core_req.valid  <= '1'; -- hold until accepted
                            core_req.addr   <= std_logic_vector(
                                                   resize(byte_off, C_AXI_ADDR_WIDTH) + G_DST_BASE);
                            core_req.we     <= '0';
                            core_req.nbytes <= to_unsigned(G_CHUNK_BYTES, 7);
                        end if;

                    when S_VERIFY_DST_WAIT =>
                        if core_resp.rvalid = '1' then
                            state <= S_VERIFY_COMPARE;
                            if core_resp.rdata /= verify_src then
                                mismatch <= '1';
                            end if;
                        end if;

                    when S_VERIFY_COMPARE =>
                        if byte_off + G_CHUNK_BYTES >= G_COPY_SIZE then
                            if mismatch = '1' then
                                state <= S_RETRY_CHECK;
                            else
                                state <= S_DONE;
                            end if;
                        else
                            byte_off <= byte_off + G_CHUNK_BYTES;
                            state    <= S_VERIFY_SRC_ISSUE;
                        end if;

                    ------------------------------------------------------------
                    -- Retry or give up
                    ------------------------------------------------------------
                    when S_RETRY_CHECK =>
                        if retry_count >= G_MAX_RETRIES then
                            state <= S_FAIL;
                        else
                            retry_count <= retry_count + 1;
                            byte_off    <= (others => '0');
                            mismatch    <= '0';
                            state       <= S_COPY_READ_ISSUE;
                        end if;

                    when S_DONE =>
                        cpu_reset_n <= '1';
                        boot_done   <= '1';

                    when S_FAIL =>
                        boot_fail   <= '1';
                        -- cpu_reset_n stays low: never run from unverified data.

                    when others =>
                        -- Defensive default for an unreachable/SEU-corrupted
                        -- state encoding: treat as a hang, not a silent
                        -- continuation from garbage state.
                        core_req.valid <= '0';
                        state          <= S_RETRY_CHECK;

                end case;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
