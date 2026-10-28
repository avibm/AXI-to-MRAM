--------------------------------------------------------------------------------
-- sram_boot_copy.vhd
--
-- Copies the firmware image from the MRAM master copy into the on-chip SRAM
-- and holds the CPU in reset until the copy is verified. Replaces
-- mram_boot_copy (MRAM -> MRAM copy). See doc/SRAM_ARCH.md.
--
--   * MRAM side: beat-sized reads through the core interface (core_req /
--     core_resp, see mram_pkg), issued back to back so the backend streams
--     them as one RDQI. Data goes into a FIFO of G_FIFO_BLOCKS blocks.
--   * SRAM side: an AXI4 master (m_axi_*), connected to the interconnect as
--     an initiator. Each G_BLOCK_BYTES block is one INCR burst of full-width
--     beats with all strobes set.
--
-- Sequence, after reset or a reload request:
--   1. Power-up wait (G_POWERUP_CYCLES, MRAM tPU; after reset only). While
--      boot_hold = '1' at the end of it, the copy waits and the AXI side of
--      the MRAM stays open (boot_held = '1').
--   2. Take the MRAM backend: mram_hold = '1' stops the AXI slave wrapper
--      from accepting new bursts; once it reports mram_idle = '1', the
--      backend is handed over (mram_own = '1'). The AXI side is never reset,
--      so a burst in flight from PCI completes normally.
--   3. Copy pass: MRAM G_SRC_BASE .. + G_COPY_SIZE -> SRAM G_DST_ADDR.
--      An AW is raised only when its whole block is in the FIFO, its W
--      beats follow without gaps, and BREADY is always '1'.
--   4. Verify pass: the MRAM source is read again; each SRAM block is read
--      back (ARVALID only when the matching MRAM block is in the FIFO,
--      RREADY always '1') and compared beat by beat (two register stages). Re-reading the MRAM
--      also catches an MRAM read error in the copy pass.
--   5. Pass: cpu_reset_n = '1', boot_done = '1', the backend goes back to
--      the AXI side. Fail (mismatch, BRESP/RRESP not OKAY, MRAM error, wrong
--      ID or RLAST): retry the whole copy, up to G_MAX_RETRIES times, then
--      boot_fail = '1' with the CPU held in reset.
--
-- Reload: a one-cycle pulse on reload, in the done or failed state, resets
-- the CPU at once (cpu_reset_n = '0', boot_done = '0') and runs steps 2-5
-- again without the power-up wait. Ignored while a copy is running and
-- while the fabric is stuck (see below).
--
-- Never abandon a handshake (doc/SRAM_ARCH.md, lesson 1). If nothing
-- progresses for G_WATCHDOG_LIMIT cycles, the engine stops issuing new
-- MRAM reads, AWs and ARs but keeps every raised VALID until it is
-- accepted, sends the remaining W beats of every accepted AW, and takes
-- every B, R and MRAM response. A retry starts only once all of them are
-- complete. If that drain does not finish within another G_WATCHDOG_LIMIT
-- cycles, the fabric is taken as stuck: boot_fail = '1' and boot_status
-- reports it (phase 7, bit 13); the MRAM goes back to the AXI side once
-- its part is quiet. If the fabric answers later and everything in flight
-- completes, the engine leaves the stuck state by itself: boot_fail
-- returns to '0' and the copy is retried (counted like any retry; with no
-- retries left it ends in the failed state). Nothing is issued while
-- stuck, so a fabric that never answers stays stuck until aresetn.
--
-- boot_status (aclk register, for a PCI status register or Identify):
--   [2:0]  phase: 0 done, 1 power-up / held, 2 waiting for the MRAM port,
--          3 copy, 4 verify, 5 drain / retry, 6 failed, 7 stuck
--   [7:4]  retries used (saturates at 15)
--   [8]    verify mismatch          [9]  BRESP not OKAY
--   [10]   RRESP not OKAY           [11] MRAM core error
--   [12]   watchdog timeout         [13] drain timeout (fabric stuck)
--   [14]   wrong BID/RID, wrong RLAST or an unexpected response
--   [15]   reserved, '0'
--   Bits 8-14 are sticky from the start of the boot or reload, so after a
--   successful boot they show what the retries were for.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity sram_boot_copy is
    generic (
        G_SRC_BASE       : natural := 16#6000000#; -- MRAM offset of the master firmware copy
        G_DST_ADDR       : natural := 0;           -- AXI address of the SRAM
        G_COPY_SIZE      : natural := 1048576;     -- bytes, multiple of G_BLOCK_BYTES
        G_BLOCK_BYTES    : natural := 64;          -- bytes per AXI burst, multiple of C_BEAT_BYTES
        G_FIFO_BLOCKS    : natural := 2;           -- FIFO depth in blocks
        G_MAX_WR_OUTST   : natural := 2;           -- AXI write bursts awaiting B
        G_MAX_RD_OUTST   : natural := 2;           -- AXI read bursts awaiting their last R
        G_MAX_RETRIES    : natural := 3;
        G_WATCHDOG_LIMIT : natural := 1_000_000;   -- aclk cycles without progress = hang
        G_POWERUP_CYCLES : natural := 3_750_000;   -- MRAM tPU, 25ms at 150MHz
        G_M_ID           : natural := 0
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        -- MRAM backend (via the mux and write guard in mram_top)
        core_req  : out core_req_t;
        core_resp : in  core_resp_t;  -- masked to idle by mram_top unless mram_own = '1'
        mram_hold : out std_logic;    -- to the AXI slave wrapper: accept no new bursts
        mram_idle : in  std_logic;    -- from the wrapper: nothing accepted or in flight
        mram_own  : out std_logic;    -- '1' = this block drives the backend

        -- AXI4 master to the SRAM (through the interconnect)
        m_axi_awid    : out std_logic_vector(C_M_AXI_ID_WIDTH - 1 downto 0);
        m_axi_awaddr  : out std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        m_axi_awlen   : out std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        m_axi_awsize  : out std_logic_vector(2 downto 0);
        m_axi_awburst : out std_logic_vector(1 downto 0);
        m_axi_awlock  : out std_logic;
        m_axi_awcache : out std_logic_vector(3 downto 0);
        m_axi_awprot  : out std_logic_vector(2 downto 0);
        m_axi_awvalid : out std_logic;
        m_axi_awready : in  std_logic;
        m_axi_wdata   : out std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        m_axi_wstrb   : out std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);
        m_axi_wlast   : out std_logic;
        m_axi_wvalid  : out std_logic;
        m_axi_wready  : in  std_logic;
        m_axi_bid     : in  std_logic_vector(C_M_AXI_ID_WIDTH - 1 downto 0);
        m_axi_bresp   : in  std_logic_vector(1 downto 0);
        m_axi_bvalid  : in  std_logic;
        m_axi_bready  : out std_logic;
        m_axi_arid    : out std_logic_vector(C_M_AXI_ID_WIDTH - 1 downto 0);
        m_axi_araddr  : out std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        m_axi_arlen   : out std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        m_axi_arsize  : out std_logic_vector(2 downto 0);
        m_axi_arburst : out std_logic_vector(1 downto 0);
        m_axi_arlock  : out std_logic;
        m_axi_arcache : out std_logic_vector(3 downto 0);
        m_axi_arprot  : out std_logic_vector(2 downto 0);
        m_axi_arvalid : out std_logic;
        m_axi_arready : in  std_logic;
        m_axi_rid     : in  std_logic_vector(C_M_AXI_ID_WIDTH - 1 downto 0);
        m_axi_rdata   : in  std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        m_axi_rresp   : in  std_logic_vector(1 downto 0);
        m_axi_rlast   : in  std_logic;
        m_axi_rvalid  : in  std_logic;
        m_axi_rready  : out std_logic;

        reload       : in  std_logic;        -- one-cycle pulse (aclk): copy again
        boot_hold    : in  std_logic := '0'; -- '1' = do not start after power-up (aclk-synchronous)
        cpu_reset_n  : out std_logic;
        boot_done    : out std_logic;
        boot_fail    : out std_logic;        -- retries exhausted, or the fabric is stuck
        powerup_done : out std_logic;        -- tPU has elapsed; MRAM may be accessed
        boot_held    : out std_logic;        -- '1' = waiting on boot_hold
        boot_status  : out std_logic_vector(15 downto 0)  -- see the header
    );
end entity sram_boot_copy;

architecture rtl of sram_boot_copy is

    constant NB     : natural := C_BEAT_BYTES;                  -- bytes per beat
    constant BPB    : natural := G_BLOCK_BYTES / NB;            -- beats per block / burst
    constant NBLK   : natural := G_COPY_SIZE / G_BLOCK_BYTES;   -- blocks
    constant NBEAT  : natural := G_COPY_SIZE / NB;              -- beats
    constant FDEPTH : natural := G_FIFO_BLOCKS * BPB;           -- FIFO depth in beats

    -- S_POWERUP   MRAM power-up wait after reset; also waits on boot_hold
    -- S_WAIT_IDLE AXI side held off, waiting for it to finish (mram_idle)
    -- S_COPY      copy pass          S_VERIFY  verify pass
    -- S_DRAIN     aborted pass, completing what is in flight
    -- S_RETRY     decide: another attempt or fail
    -- S_DONE      CPU released       S_FAIL    retries exhausted, CPU in reset
    -- S_STUCK     drain timed out; waits for the fabric (or aresetn)
    type state_t is (S_POWERUP, S_WAIT_IDLE, S_COPY, S_VERIFY, S_DRAIN, S_RETRY,
                     S_DONE, S_FAIL, S_STUCK);
    signal state : state_t := S_POWERUP;

    signal powerup_cnt : natural range 0 to G_POWERUP_CYCLES := 0;  -- tPU counter
    signal powerup_ok  : std_logic := '0';                          -- tPU elapsed
    signal hold_i, own : std_logic := '1';  -- drive mram_hold / mram_own

    -- Counters for the current pass
    signal m_iss  : natural range 0 to NBEAT := 0;   -- MRAM reads accepted
    signal iss_addr : unsigned(C_MRAM_ADDR_WIDTH - 1 downto 0);  -- next MRAM read address
    signal m_rx   : natural range 0 to NBEAT := 0;   -- MRAM read data received
    signal cons   : natural range 0 to NBEAT := 0;   -- FIFO beats consumed (W loaded / R compared)
    signal aw_n   : natural range 0 to NBLK := 0;    -- AW handshakes
    signal b_n    : natural range 0 to NBLK := 0;    -- B received
    signal ar_n   : natural range 0 to NBLK := 0;    -- AR handshakes
    signal r_n    : natural range 0 to NBEAT := 0;   -- R beats received
    signal r_blk  : natural range 0 to NBLK := 0;    -- read bursts completed (RLAST seen)
    signal r_idx  : natural range 0 to BPB - 1 := 0; -- beat index inside the current R burst
    signal w_idx  : natural range 0 to BPB - 1 := 0; -- beat index inside the current W burst

    -- FIFO, kept in flip-flops: small and wide, read by the W and compare paths
    type fifo_t is array (0 to FDEPTH - 1) of std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal fifo : fifo_t;
    attribute syn_ramstyle : string;
    attribute syn_ramstyle of fifo : signal is "registers";
    signal wptr, rptr : natural range 0 to FDEPTH - 1 := 0;

    -- Verify compare pipeline: R data and FIFO data registered (cmp_*),
    -- then the compare result registered (mis_*) before it reaches the
    -- error and state logic.
    signal cmp_v      : std_logic := '0';
    signal cmp_d, cmp_e : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal mis_v, mis : std_logic := '0';

    signal abort      : std_logic := '0';  -- stop issuing, finish what is in flight
    signal att_err    : std_logic := '0';  -- an error in the current attempt
    signal wd_cnt     : natural range 0 to G_WATCHDOG_LIMIT := 0;
    signal retries    : natural range 0 to G_MAX_RETRIES := 0;
    signal st_retries : unsigned(3 downto 0) := (others => '0');
    signal st_err     : std_logic_vector(14 downto 8) := (others => '0');

    signal req     : core_req_t := CORE_REQ_IDLE;   -- MRAM read request (core_req)
    signal awvalid, wvalid, arvalid : std_logic := '0';  -- AXI VALIDs (registers)
    signal wlast   : std_logic := '0';

    -- boot_status[2:0] encoding of the current state

    function phase_code(st : state_t) return std_logic_vector is
    begin
        case st is
            when S_DONE      => return "000";
            when S_POWERUP   => return "001";
            when S_WAIT_IDLE => return "010";
            when S_COPY      => return "011";
            when S_VERIFY    => return "100";
            when S_DRAIN | S_RETRY => return "101";
            when S_FAIL      => return "110";
            when others      => return "111";
        end case;
    end function;

begin

    assert G_BLOCK_BYTES mod NB = 0 and BPB >= 1 and BPB <= 256 and G_BLOCK_BYTES <= 4096
        report "G_BLOCK_BYTES must be a multiple of C_BEAT_BYTES, at most 256 beats and 4KB"
        severity failure;
    assert G_COPY_SIZE mod G_BLOCK_BYTES = 0 and G_COPY_SIZE > 0
        report "G_COPY_SIZE must be a multiple of G_BLOCK_BYTES" severity failure;
    assert G_DST_ADDR mod G_BLOCK_BYTES = 0 and G_SRC_BASE mod NB = 0
        report "G_DST_ADDR must be block-aligned (no 4KB crossing), G_SRC_BASE beat-aligned"
        severity failure;
    assert G_FIFO_BLOCKS >= 1 report "G_FIFO_BLOCKS must be at least 1" severity failure;

    core_req     <= req;
    mram_hold    <= hold_i;
    mram_own     <= own;
    powerup_done <= powerup_ok;
    boot_held    <= '1' when state = S_POWERUP and powerup_ok = '1' and boot_hold = '1' else '0';
    boot_status  <= '0' & st_err & std_logic_vector(st_retries) & '0'
                    & phase_code(state);

    -- Fixed burst attributes
    m_axi_awid    <= std_logic_vector(to_unsigned(G_M_ID, C_M_AXI_ID_WIDTH));
    m_axi_arid    <= std_logic_vector(to_unsigned(G_M_ID, C_M_AXI_ID_WIDTH));
    m_axi_awlen   <= std_logic_vector(to_unsigned(BPB - 1, C_AXI_LEN_WIDTH));
    m_axi_arlen   <= std_logic_vector(to_unsigned(BPB - 1, C_AXI_LEN_WIDTH));
    m_axi_awsize  <= SIZE_BEAT;
    m_axi_arsize  <= SIZE_BEAT;
    m_axi_awburst <= AXI_BURST_INCR;
    m_axi_arburst <= AXI_BURST_INCR;
    m_axi_awlock  <= '0';
    m_axi_arlock  <= '0';
    m_axi_awcache <= "0000";  -- device non-bufferable: B comes from the SRAM itself
    m_axi_arcache <= "0000";
    m_axi_awprot  <= "000";
    m_axi_arprot  <= "000";
    m_axi_wstrb   <= (others => '1');
    m_axi_bready  <= '1';     -- every response is taken at once
    m_axi_rready  <= '1';
    m_axi_awvalid <= awvalid;
    m_axi_wvalid  <= wvalid;
    m_axi_wlast   <= wlast;
    m_axi_arvalid <= arvalid;

    ----------------------------------------------------------------------------
    -- The whole engine is one clocked process: the MRAM read side, the AXI
    -- write and read channels, the error collection, the watchdog and the
    -- sequencer all run every cycle; the sequencer (state) decides which
    -- channels may start new transfers (v_issue). Handshakes already started
    -- are always completed, whatever the state.
    ----------------------------------------------------------------------------
    process (aclk)
        variable v_issue  : boolean;  -- new MRAM reads, AWs and ARs allowed
        variable v_prog   : boolean;  -- something moved this cycle
        variable v_err    : std_logic_vector(14 downto 8);  -- errors seen this cycle (boot_status bits)
        variable v_drained : boolean;  -- nothing in flight (see below)
        variable v_new_pass : boolean;  -- start a pass from a clean slate (nothing in flight)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                -- Synchronous reset: power-up wait again, CPU held in reset,
                -- AXI side of the MRAM held (hold_i = 1) until the copy can
                -- start, no request or VALID raised.
                state       <= S_POWERUP;
                powerup_cnt <= 0;
                powerup_ok  <= '0';
                hold_i      <= '1';
                own         <= '0';
                req         <= CORE_REQ_IDLE;
                awvalid     <= '0';
                wvalid      <= '0';
                wlast       <= '0';
                arvalid     <= '0';
                cmp_v       <= '0';
                mis_v       <= '0';
                att_err     <= '0';
                retries     <= 0;
                st_retries  <= (others => '0');
                st_err      <= (others => '0');
                cpu_reset_n <= '0';
                boot_done   <= '0';
                boot_fail   <= '0';
                m_iss <= 0; m_rx <= 0; cons <= 0;
                iss_addr <= to_unsigned(G_SRC_BASE, C_MRAM_ADDR_WIDTH);
                aw_n  <= 0; b_n  <= 0; ar_n <= 0; r_n <= 0; r_blk <= 0;
                r_idx <= 0; w_idx <= 0;
                wptr  <= 0; rptr <= 0;
                abort <= '0';
                wd_cnt <= 0;
            else
                -- New transfers only in a copy or verify pass, never while
                -- aborting (draining) or in any other state.
                v_issue := (state = S_COPY or state = S_VERIFY) and abort = '0';
                v_prog  := false;
                v_err   := (others => '0');
                v_new_pass := false;

                ----------------------------------------------------------------
                -- MRAM reads: hold valid until accepted; a new read only if
                -- its data has a free FIFO slot. core_resp.ready is
                -- combinational from the backend, so it only enables
                -- registers here: after each acceptance valid drops for one
                -- cycle and the next request is built from registers alone
                -- (an MRAM beat takes about 64 cycles, so the bubble costs
                -- nothing).
                ----------------------------------------------------------------
                if req.valid = '1' then
                    -- A request is up: keep it unchanged until accepted
                    -- (core interface contract, also while aborting).
                    if core_resp.ready = '1' then
                        req.valid <= '0';
                        m_iss     <= m_iss + 1;
                        iss_addr  <= iss_addr + NB;
                        v_prog    := true;
                    end if;
                elsif v_issue and m_iss < NBEAT and m_iss - cons < FDEPTH then
                    -- Next beat of the pass, if any is left and its data will
                    -- have a FIFO slot: reads accepted but not yet consumed
                    -- (m_iss - cons) must stay below the FIFO depth.
                    req.valid  <= '1';
                    req.we     <= '0';
                    req.cont   <= '0';
                    req.nbytes <= to_unsigned(NB, 7);
                    req.addr   <= std_logic_vector(iss_addr);
                end if;

                -- MRAM read data: always taken (there is no back-pressure on
                -- rvalid) into the FIFO slot reserved when it was requested.
                if core_resp.rvalid = '1' then
                    fifo(wptr) <= core_resp.rdata;
                    wptr       <= (wptr + 1) mod FDEPTH;
                    m_rx       <= m_rx + 1;
                    v_prog     := true;
                    if core_resp.error = '1' then
                        v_err(11) := '1';           -- MRAM read error (not produced by
                    end if;                         -- the current backend; kept defensively)
                end if;
                -- A write completion can only belong to someone else: the
                -- ownership mask in mram_top should make this impossible.
                if core_resp.bvalid = '1' then
                    v_err(14) := '1';               -- this block never writes the MRAM
                end if;

                ----------------------------------------------------------------
                -- Copy pass: AW per complete block in the FIFO, W beats from
                -- the FIFO, B always accepted.
                ----------------------------------------------------------------
                if awvalid = '1' then
                    -- AWVALID is up: hold it and AWADDR until AWREADY (AXI rule,
                    -- also while aborting).
                    if m_axi_awready = '1' then
                        awvalid <= '0';
                        aw_n    <= aw_n + 1;
                        v_prog  := true;
                    end if;
                elsif v_issue and state = S_COPY and aw_n < NBLK
                      and (aw_n + 1) * BPB <= m_rx and aw_n - b_n < G_MAX_WR_OUTST then
                    -- Next write burst, when: a block is left to write, the whole
                    -- block has arrived from the MRAM (so its W beats can follow
                    -- without gaps), and fewer than G_MAX_WR_OUTST bursts await B.
                    awvalid      <= '1';
                    m_axi_awaddr <= std_logic_vector(to_unsigned(G_DST_ADDR + aw_n * G_BLOCK_BYTES,
                                                                 C_AXI_ADDR_WIDTH));
                end if;

                -- W: load the next beat when the output register is free;
                -- only beats of bursts whose AW has been accepted. This also
                -- runs while aborting: every accepted AW gets all its beats.
                if wvalid = '0' or m_axi_wready = '1' then
                    -- The W register is empty or its beat is being taken now.
                    if wvalid = '1' then
                        v_prog := true;             -- a W handshake this cycle
                    end if;
                    if cons < aw_n * BPB then
                        -- More beats are owed to accepted AWs: load the next one
                        -- from the FIFO. In a verify pass aw_n = 0, so never.
                        m_axi_wdata <= fifo(rptr);
                        rptr        <= (rptr + 1) mod FDEPTH;
                        cons        <= cons + 1;
                        wvalid      <= '1';
                        if w_idx = BPB - 1 then
                            wlast <= '1';           -- last beat of the burst
                            w_idx <= 0;
                        else
                            wlast <= '0';
                            w_idx <= w_idx + 1;
                        end if;
                    else
                        wvalid <= '0';              -- nothing owed: W idle
                        wlast  <= '0';
                    end if;
                end if;

                -- B: BREADY is always 1, so every BVALID is a handshake.
                if m_axi_bvalid = '1' then
                    v_prog := true;
                    if b_n < aw_n then
                        b_n <= b_n + 1;             -- answers an accepted AW
                    else
                        v_err(14) := '1';           -- B with no write outstanding
                    end if;
                    if m_axi_bresp /= AXI_RESP_OKAY then
                        v_err(9) := '1';            -- SLVERR/DECERR: retry the copy
                    end if;
                    if unsigned(m_axi_bid) /= G_M_ID then
                        v_err(14) := '1';           -- not our ID: misrouted response
                    end if;
                end if;

                ----------------------------------------------------------------
                -- Verify pass: AR per complete block in the FIFO, R compared
                -- with the FIFO (two register stages), R always accepted.
                ----------------------------------------------------------------
                if arvalid = '1' then
                    -- ARVALID is up: hold it and ARADDR until ARREADY.
                    if m_axi_arready = '1' then
                        arvalid <= '0';
                        ar_n    <= ar_n + 1;
                        v_prog  := true;
                    end if;
                elsif v_issue and state = S_VERIFY and ar_n < NBLK
                      and (ar_n + 1) * BPB <= m_rx and ar_n - r_blk < G_MAX_RD_OUTST then
                    -- Next read-back burst, when: a block is left to check, its
                    -- MRAM data is already in the FIFO (so every R beat can be
                    -- compared on arrival with RREADY = 1), and fewer than
                    -- G_MAX_RD_OUTST bursts are still returning data.
                    arvalid      <= '1';
                    m_axi_araddr <= std_logic_vector(to_unsigned(G_DST_ADDR + ar_n * G_BLOCK_BYTES,
                                                                 C_AXI_ADDR_WIDTH));
                end if;

                -- R: RREADY is always 1, so every RVALID is a handshake.
                -- Stage 1 of the compare: register the R data and the matching
                -- MRAM data from the FIFO (cmp_d, cmp_e).
                cmp_v <= '0';
                if m_axi_rvalid = '1' then
                    v_prog := true;
                    if r_n < ar_n * BPB then
                        -- a beat of an accepted AR
                        r_n   <= r_n + 1;
                        cmp_v <= '1';
                        cmp_d <= m_axi_rdata;
                        cmp_e <= fifo(rptr);
                        rptr  <= (rptr + 1) mod FDEPTH;
                        cons  <= cons + 1;
                        if (r_idx = BPB - 1) /= (m_axi_rlast = '1') then
                            v_err(14) := '1';       -- RLAST missing or on the wrong beat
                        end if;
                        if r_idx = BPB - 1 then
                            r_idx <= 0;             -- burst complete
                            r_blk <= r_blk + 1;
                        else
                            r_idx <= r_idx + 1;
                        end if;
                    else
                        v_err(14) := '1';           -- R with no read outstanding
                    end if;
                    if m_axi_rresp /= AXI_RESP_OKAY then
                        v_err(10) := '1';           -- SLVERR/DECERR: retry the copy
                    end if;
                    if unsigned(m_axi_rid) /= G_M_ID then
                        v_err(14) := '1';           -- not our ID: misrouted response
                    end if;
                end if;
                -- Stage 2: compare the registered pair (64-bit compare between
                -- two register stages); stage 3: report the result.
                mis_v <= cmp_v;
                mis   <= '0';
                if cmp_d /= cmp_e then
                    mis <= '1';
                end if;
                if mis_v = '1' and mis = '1' then
                    v_err(8) := '1';                -- SRAM data differs from the MRAM
                end if;

                -- Collect this cycle's errors: sticky in boot_status (st_err)
                -- and for the current attempt (att_err, decides retry or pass).
                st_err <= st_err or v_err;
                if v_err /= (v_err'range => '0') then
                    att_err <= '1';
                end if;

                -- Nothing issued and not yet answered, on either side: no
                -- pending request or VALID, every W beat of every accepted AW
                -- sent (in a copy pass cons counts the W beats; in a verify
                -- pass aw_n = 0), every B and R received, compare pipeline
                -- flushed.
                v_drained := req.valid = '0' and m_iss = m_rx and awvalid = '0' and arvalid = '0'
                             and wvalid = '0' and cons >= aw_n * BPB and b_n = aw_n
                             and r_n = ar_n * BPB and cmp_v = '0' and mis_v = '0';

                ----------------------------------------------------------------
                -- Watchdog: no progress for G_WATCHDOG_LIMIT cycles in a pass;
                -- in S_DRAIN the same limit bounds the whole drain.
                ----------------------------------------------------------------
                if state = S_COPY or state = S_VERIFY then
                    -- in a pass: restart on any progress, else count up and
                    -- saturate at the limit (the sequencer acts on it)
                    if v_prog then
                        wd_cnt <= 0;
                    elsif wd_cnt < G_WATCHDOG_LIMIT then
                        wd_cnt <= wd_cnt + 1;
                    end if;
                elsif state = S_DRAIN then
                    -- draining: total time limit, progress does not restart it
                    if wd_cnt < G_WATCHDOG_LIMIT then
                        wd_cnt <= wd_cnt + 1;
                    end if;
                end if;

                ----------------------------------------------------------------
                -- Sequencer
                ----------------------------------------------------------------
                case state is

                    -- After reset: wait out the MRAM power-up time (tPU), then
                    -- start, unless boot_hold asks to wait.
                    when S_POWERUP =>
                        if powerup_cnt >= G_POWERUP_CYCLES then
                            powerup_ok <= '1';      -- MRAM usable (register commands too)
                            if boot_hold = '0' then
                                hold_i <= '1';
                                state  <= S_WAIT_IDLE;
                            else
                                hold_i <= '0';  -- held: the AXI side keeps the MRAM
                            end if;
                        else
                            powerup_cnt <= powerup_cnt + 1;
                        end if;

                    -- hold_i is '1' here, so the wrapper accepts nothing new;
                    -- its idle is computed from its own registers.
                    when S_WAIT_IDLE =>
                        -- the wrapper has finished everything it accepted:
                        -- take the backend and start the copy pass
                        if mram_idle = '1' then
                            own     <= '1';
                            att_err <= '0';
                            v_new_pass := true;
                            state   <= S_COPY;
                        end if;

                    -- Copy pass: MRAM -> SRAM.
                    when S_COPY =>
                        if wd_cnt = G_WATCHDOG_LIMIT then
                            -- no progress for the watchdog time: stop issuing
                            -- and complete what is in flight (drain)
                            st_err(12) <= '1';
                            abort  <= '1';
                            wd_cnt <= 0;
                            state  <= S_DRAIN;
                        elsif b_n = NBLK and v_drained then
                            -- every block written and acknowledged
                            if att_err = '1' or v_err /= (v_err'range => '0') then
                                state <= S_RETRY;   -- an error in this attempt
                            else
                                -- clean copy: read everything back and compare
                                v_new_pass := true;
                                state <= S_VERIFY;
                            end if;
                        end if;

                    -- Verify pass: MRAM again vs SRAM read-back.
                    when S_VERIFY =>
                        if wd_cnt = G_WATCHDOG_LIMIT then
                            -- no progress: drain, as in the copy pass
                            st_err(12) <= '1';
                            abort  <= '1';
                            wd_cnt <= 0;
                            state  <= S_DRAIN;
                        elsif r_n = NBEAT and v_drained then
                            -- every beat read back and compared
                            -- cmp_v = mis_v = '0': the last compare is in v_err already
                            if att_err = '1' or v_err /= (v_err'range => '0') then
                                state <= S_RETRY;   -- mismatch or error: copy again
                            else
                                -- verified: give the MRAM back to the AXI side
                                -- and release the CPU
                                own         <= '0';
                                hold_i      <= '0';
                                cpu_reset_n <= '1';
                                boot_done   <= '1';
                                state       <= S_DONE;
                            end if;
                        end if;

                    -- Aborted pass: nothing new is issued (abort = 1); wait
                    -- until every started transfer has completed.
                    when S_DRAIN =>
                        if v_drained then
                            state <= S_RETRY;       -- clean again: retry the copy
                        elsif wd_cnt = G_WATCHDOG_LIMIT then
                            -- the drain did not finish either: the fabric is
                            -- taken as stuck (VALIDs stay up, see S_STUCK)
                            st_err(13) <= '1';
                            boot_fail  <= '1';
                            state      <= S_STUCK;
                        end if;

                    -- Decide between another attempt and giving up. Only
                    -- reached with nothing in flight.
                    when S_RETRY =>
                        if retries >= G_MAX_RETRIES then
                            -- give up: MRAM back to the AXI side, CPU stays in
                            -- reset, boot_fail for the fault indication
                            own       <= '0';
                            hold_i    <= '0';
                            boot_fail <= '1';
                            state     <= S_FAIL;
                        else
                            -- another attempt, from the start of the image; the
                            -- backend is still owned, so straight to S_COPY
                            retries <= retries + 1;
                            if st_retries /= 15 then
                                st_retries <= st_retries + 1;   -- 4-bit status field saturates
                            end if;
                            att_err <= '0';
                            v_new_pass := true;
                            state   <= S_COPY;
                        end if;

                    -- Finished (good or failed). A reload request puts the CPU
                    -- back into reset at once and starts a new copy, without
                    -- the power-up wait; the error record starts afresh.
                    when S_DONE | S_FAIL =>
                        if reload = '1' then
                            cpu_reset_n <= '0';
                            boot_done   <= '0';
                            boot_fail   <= '0';
                            retries     <= 0;
                            st_retries  <= (others => '0');
                            st_err      <= (others => '0');
                            hold_i      <= '1';
                            state       <= S_WAIT_IDLE;
                        end if;

                    when S_STUCK =>
                        -- abort stays '1'; responses are still taken. If the
                        -- fabric answers after all and everything in flight
                        -- completes, retry like any other failed attempt
                        -- (through S_WAIT_IDLE, since the MRAM may have been
                        -- handed back meanwhile); with no retries left, fail.
                        -- Until then, once the MRAM side is quiet, give it
                        -- back to the AXI side so PCI can inspect the MRAM.
                        if v_drained then
                            -- the fabric has answered everything after all
                            if retries >= G_MAX_RETRIES then
                                own       <= '0';   -- no retry left: fail
                                hold_i    <= '0';   -- (boot_fail is already 1)
                                state     <= S_FAIL;
                            else
                                retries <= retries + 1;
                                if st_retries /= 15 then
                                    st_retries <= st_retries + 1;
                                end if;
                                boot_fail <= '0';
                                hold_i    <= '1';
                                state     <= S_WAIT_IDLE;
                            end if;
                        elsif own = '1' and req.valid = '0' and m_iss = m_rx then
                            -- still stuck, but no MRAM request in flight: hand
                            -- the MRAM back so PCI can use it meanwhile
                            own    <= '0';
                            hold_i <= '0';
                        end if;

                    when others =>
                        -- unreachable encoding (SEU): stop issuing, drain, retry
                        abort  <= '1';
                        wd_cnt <= 0;
                        state  <= S_DRAIN;

                end case;

                -- Start of a new pass. Last in the process, so it overrides
                -- the counter updates above (all of them idle at this point).
                if v_new_pass then
                    m_iss <= 0; m_rx <= 0; cons <= 0;
                    iss_addr <= to_unsigned(G_SRC_BASE, C_MRAM_ADDR_WIDTH);
                    aw_n  <= 0; b_n  <= 0; ar_n <= 0; r_n <= 0; r_blk <= 0;
                    r_idx <= 0; w_idx <= 0;
                    wptr  <= 0; rptr <= 0;
                    abort <= '0';
                    wd_cnt <= 0;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
