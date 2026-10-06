--------------------------------------------------------------------------------
-- axi4_slave_wrapper.vhd
--
-- AXI4 slave front end for the MRAM subsystem. The data width is
-- C_AXI_DATA_WIDTH from mram_pkg (default 64 bits). Translates AXI
-- transactions into single-beat core_req_t/core_resp_t requests (see
-- mram_pkg) to be serviced directly by the MRAM backend, with no
-- cache in the path.
--
-- AWSIZE/ARSIZE 0..C_LANE_BITS (1 byte .. the bus width) are supported, so
-- narrow accesses are serviced without moving a full word each time.
-- Larger sizes exceed the bus width and are rejected with SLVERR.
--
-- Writes honour WSTRB: each W beat becomes one core write per contiguous
-- run of enabled byte lanes (restricted to the lanes the beat's address
-- and size make legal), so disabled bytes are never written. A beat with
-- no strobes set writes nothing. Reads fetch the whole size-aligned
-- container of each beat and return it on the matching byte lanes.
-- Beat addresses follow the AXI INCR rule: the first beat uses AxADDR as
-- given (it may be unaligned), later beats are size-aligned.
--
-- Scope / documented simplifications for this skeleton (tighten later if
-- needed -- these are deliberate choices, not oversights):
--   * Only INCR bursts are implemented. FIXED and WRAP get SLVERR.
--   * AxSIZE is fixed for the whole burst per AXI4 semantics; this design
--     relies on that (captures size once at AW/AR acceptance and reuses it
--     for every beat's address increment in that burst).
--   * Beats are serviced strictly in order. Reads are issued ahead of the
--     R channel (see "Read streaming" below); writes up to two runs ahead.
--   * Read bursts may be outstanding at the AR-channel level, up to
--     G_MAX_OUTSTANDING, but data is returned strictly in the order the
--     bursts were accepted (in-order completion).
--   * WLAST is not checked against AWLEN on the normal write path (the beat
--     count comes from AWLEN); it is used only to drain rejected bursts.
--   * 4KB-boundary crossing is not checked (the master must not do it).
--   * Only AxADDR(C_MRAM_ADDR_WIDTH-1:0) reaches the MRAM: the upper bits
--     (this slave's base address, decoded by the interconnect) are ignored.
--   * core_req/core_resp completion is assumed in-order.
--   * AWLOCK/AWCACHE/AWQOS/ARLOCK/ARCACHE/ARQOS/AWPROT/ARPROT are present
--     on the port for interconnect compatibility but are not acted upon.
--
-- Write continuation: when the last enabled run of a W beat has been
-- accepted and more beats follow, the engine does not wait for that
-- write's completion; it fetches the next beat at once and marks its first
-- run cont = '1'. If it starts at the byte after the previous write, the
-- backend appends it to the SPI write still in progress (one WREN, opcode
-- and address for the whole run of contiguous bytes); otherwise it is
-- served as a separate write. At most two writes are outstanding.
--
-- Read streaming: an issue engine walks the accepted AR bursts and issues
-- one core read per beat (the beat's size-aligned container) as long as
-- the read-data buffer (C_RBUF beats) has room for every read in flight,
-- without waiting for the R channel. Contiguous beats, also across
-- consecutive bursts, therefore reach the backend back to back and are
-- read in one RDQI. R data is returned in order from the buffer.
--
-- Core-side arbitration: the write and read engines each hold their request
-- until accepted. Writes win when both are presented; ready is routed only
-- to the engine whose request is actually on core_req.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity axi4_slave_wrapper is
    generic (
        G_ID_WIDTH        : integer := C_AXI_ID_WIDTH;
        G_MAX_OUTSTANDING : integer := C_MAX_OUTSTANDING
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        -- Write address channel
        s_axi_awid     : in  std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_awaddr   : in  std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        s_axi_awlen    : in  std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        s_axi_awsize   : in  std_logic_vector(2 downto 0);
        s_axi_awburst  : in  std_logic_vector(1 downto 0);
        s_axi_awlock   : in  std_logic                     := '0';
        s_axi_awcache  : in  std_logic_vector(3 downto 0)  := (others => '0');
        s_axi_awprot   : in  std_logic_vector(2 downto 0)  := (others => '0');
        s_axi_awqos    : in  std_logic_vector(3 downto 0)  := (others => '0');
        s_axi_awvalid  : in  std_logic;
        s_axi_awready  : out std_logic;

        -- Write data channel
        s_axi_wdata  : in  std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        s_axi_wstrb  : in  std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);
        s_axi_wlast  : in  std_logic;
        s_axi_wvalid : in  std_logic;
        s_axi_wready : out std_logic;

        -- Write response channel
        s_axi_bid    : out std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_bresp  : out std_logic_vector(1 downto 0);
        s_axi_bvalid : out std_logic;
        s_axi_bready : in  std_logic;

        -- Read address channel
        s_axi_arid     : in  std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_araddr   : in  std_logic_vector(C_AXI_ADDR_WIDTH - 1 downto 0);
        s_axi_arlen    : in  std_logic_vector(C_AXI_LEN_WIDTH - 1 downto 0);
        s_axi_arsize   : in  std_logic_vector(2 downto 0);
        s_axi_arburst  : in  std_logic_vector(1 downto 0);
        s_axi_arlock   : in  std_logic                     := '0';
        s_axi_arcache  : in  std_logic_vector(3 downto 0)  := (others => '0');
        s_axi_arprot   : in  std_logic_vector(2 downto 0)  := (others => '0');
        s_axi_arqos    : in  std_logic_vector(3 downto 0)  := (others => '0');
        s_axi_arvalid  : in  std_logic;
        s_axi_arready  : out std_logic;

        -- Read data channel
        s_axi_rid    : out std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_rdata  : out std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        s_axi_rresp  : out std_logic_vector(1 downto 0);
        s_axi_rlast  : out std_logic;
        s_axi_rvalid : out std_logic;
        s_axi_rready : in  std_logic;

        -- Core-side interface to the MRAM backend (no cache in this build)
        core_req  : out core_req_t;
        core_resp : in  core_resp_t
    );
end entity axi4_slave_wrapper;

architecture rtl of axi4_slave_wrapper is

    constant LB : natural := C_LANE_BITS;
    constant NB : natural := C_BEAT_BYTES;

    -- 2**size bytes for an AXI-style 3-bit AxSIZE field (0..LB valid on
    -- this bus; larger sizes are rejected before any of these are used).
    function beat_bytes(sz : std_logic_vector(2 downto 0)) return natural is
    begin
        return to_integer(shift_left(to_unsigned(1, 8), to_integer(unsigned(sz))));
    end function;

    -- Next beat address of an INCR burst: align down to size, add size.
    function next_beat_addr(a : unsigned; sz : std_logic_vector(2 downto 0)) return unsigned is
        variable r : unsigned(a'length - 1 downto 0) := a;
    begin
        r(LB - 1 downto 0) := r(LB - 1 downto 0) and not to_unsigned(beat_bytes(sz) - 1, LB);
        return r + beat_bytes(sz);
    end function;

    -- Size-aligned container of an address (read address sent to the core).
    function align_down(a : unsigned; sz : std_logic_vector(2 downto 0)) return unsigned is
        variable r : unsigned(a'length - 1 downto 0) := a;
    begin
        r(LB - 1 downto 0) := r(LB - 1 downto 0) and not to_unsigned(beat_bytes(sz) - 1, LB);
        return r;
    end function;

    -- Byte lanes a write beat may touch: from lane addr(LB-1:0) up to the
    -- end of the size-aligned container.
    function beat_lanes(a6 : unsigned(LB - 1 downto 0); sz : std_logic_vector(2 downto 0))
        return std_logic_vector is
        variable m  : std_logic_vector(NB - 1 downto 0) := (others => '0');
        variable lo : natural range 0 to NB - 1;
        variable hi : natural range 1 to NB;
    begin
        lo := to_integer(a6);
        hi := to_integer(a6 and not to_unsigned(beat_bytes(sz) - 1, LB)) + beat_bytes(sz);
        for i in 0 to NB - 1 loop
            if i >= lo and i < hi then
                m(i) := '1';
            end if;
        end loop;
        return m;
    end function;

    -- Index of the lowest '1' bit (0 if none).
    function lowest_set(v : std_logic_vector) return natural is
        variable r : natural range 0 to v'length - 1 := 0;
    begin
        for i in v'length - 1 downto 0 loop
            if v(v'low + i) = '1' then
                r := i;
            end if;
        end loop;
        return r;
    end function;

    ----------------------------------------------------------------------------
    -- Write side
    ----------------------------------------------------------------------------
    type wr_state_t is (WR_IDLE, WR_WDATA, WR_BEAT, WR_WAIT_BVALID, WR_LAST, WR_RESP, WR_ERR_DRAIN);

    signal wr_state      : wr_state_t := WR_IDLE;
    signal wr_id         : std_logic_vector(G_ID_WIDTH - 1 downto 0);
    signal wr_addr       : unsigned(C_AXI_ADDR_WIDTH - 1 downto 0);
    signal wr_size       : std_logic_vector(2 downto 0) := SIZE_BEAT;
    signal wr_beats_left : unsigned(C_AXI_LEN_WIDTH downto 0); -- one extra bit of headroom
    signal wr_error      : std_logic := '0';
    signal wr_core_error : std_logic := '0';
    signal wr_core_req_o : core_req_t := CORE_REQ_IDLE;
    signal wr_ready      : std_logic;
    signal wr_outst      : integer range 0 to 3 := 0; -- accepted writes without bvalid yet
    signal wr_chain      : std_logic := '0';  -- next beat's first run may continue the last write
    signal wr_first_run  : std_logic := '0';  -- no run of the current beat issued yet
    signal wr_cand       : std_logic := '0';  -- the issued run is the beat's last, more beats follow

    -- Current W beat: data and the byte lanes still to be written.
    signal wr_data : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal wr_mask : std_logic_vector(C_AXI_STRB_WIDTH - 1 downto 0);

    -- Lowest contiguous run of wr_mask. Adding the mask's lowest set bit to
    -- the mask ripples a carry through that run: the sum has the run cleared
    -- and a '1' just above it, so the run is [run_first, run_end).
    signal run_sum   : unsigned(C_AXI_STRB_WIDTH downto 0);
    signal run_first : natural range 0 to C_AXI_STRB_WIDTH - 1;
    signal run_end   : natural range 0 to C_AXI_STRB_WIDTH;

    ----------------------------------------------------------------------------
    -- Read side: outstanding-burst FIFO (one process drives all of it).
    -- An issue engine walks the bursts and issues one core read per beat,
    -- ahead of the R channel, as long as the read-data buffer has room for
    -- every read in flight. Contiguous beats (also of consecutive bursts)
    -- reach the backend back to back and are streamed in one RDQI.
    ----------------------------------------------------------------------------
    type rd_entry_t is record
        id         : std_logic_vector(G_ID_WIDTH - 1 downto 0);
        addr       : unsigned(C_AXI_ADDR_WIDTH - 1 downto 0);
        size       : std_logic_vector(2 downto 0);
        beats_left : unsigned(C_AXI_LEN_WIDTH downto 0); -- beats still to output on R
        error      : std_logic;
        started    : std_logic;                          -- taken by the issue engine
    end record;

    type rd_fifo_array_t is array (0 to G_MAX_OUTSTANDING - 1) of rd_entry_t;

    signal rd_fifo       : rd_fifo_array_t;
    signal rd_fifo_count : integer range 0 to G_MAX_OUTSTANDING := 0;
    signal rd_head_ptr   : integer range 0 to G_MAX_OUTSTANDING - 1 := 0;
    signal rd_tail_ptr   : integer range 0 to G_MAX_OUTSTANDING - 1 := 0;

    signal rd_fifo_full  : std_logic;
    signal rd_core_req_o : core_req_t := CORE_REQ_IDLE;
    signal rd_ready      : std_logic;

    -- Issue engine
    signal is_ptr    : integer range 0 to G_MAX_OUTSTANDING - 1 := 0;
    signal is_avail  : integer range 0 to G_MAX_OUTSTANDING := 0; -- pushed, not yet taken
    signal is_active : std_logic := '0';
    signal is_addr   : unsigned(C_AXI_ADDR_WIDTH - 1 downto 0);
    signal is_size   : std_logic_vector(2 downto 0);
    signal is_left   : unsigned(C_AXI_LEN_WIDTH downto 0);       -- beats still to issue

    -- Read data buffer: core reads in flight + buffered beats <= C_RBUF
    constant C_RBUF : natural := 4;
    type rbuf_t is array (0 to C_RBUF - 1) of std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal rbuf      : rbuf_t;
    signal rbuf_err  : std_logic_vector(C_RBUF - 1 downto 0);
    signal rb_wr, rb_rd : integer range 0 to C_RBUF - 1 := 0;
    signal rb_cnt    : integer range 0 to C_RBUF := 0;
    signal rd_infl   : integer range 0 to C_RBUF := 0;

begin

    rd_fifo_full  <= '1' when rd_fifo_count = G_MAX_OUTSTANDING else '0';

    ----------------------------------------------------------------------------
    -- Core request arbiter (combinational mux, fixed write priority --
    -- writes are typically rarer/shorter-lived than reads; revisit if
    -- profiling shows write starvation). Each engine only sees ready while
    -- its own request is the one on core_req.
    ----------------------------------------------------------------------------
    core_req <= wr_core_req_o when wr_core_req_o.valid = '1' else rd_core_req_o;
    wr_ready <= core_resp.ready and wr_core_req_o.valid;
    rd_ready <= core_resp.ready and rd_core_req_o.valid and not wr_core_req_o.valid;

    run_sum   <= unsigned('0' & wr_mask) + unsigned('0' & (wr_mask and std_logic_vector(
                                                         unsigned(not wr_mask) + 1)));
    run_first <= lowest_set(wr_mask);
    run_end   <= lowest_set(std_logic_vector(run_sum));

    ----------------------------------------------------------------------------
    -- Write channel FSM
    ----------------------------------------------------------------------------
    process (aclk)
        variable v_out : integer range 0 to 4;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                wr_outst      <= 0;
                wr_chain      <= '0';
                wr_state      <= WR_IDLE;
                s_axi_awready <= '0';
                s_axi_wready  <= '0';
                s_axi_bvalid  <= '0';
                wr_core_req_o <= CORE_REQ_IDLE;
                wr_error      <= '0';
                wr_core_error <= '0';
            else
                s_axi_awready <= '0';
                s_axi_wready  <= '0';

                -- Outstanding core writes: +1 on acceptance, -1 on bvalid
                -- (every bvalid on core_resp belongs to this engine).
                v_out := wr_outst;
                if wr_core_req_o.valid = '1' and wr_ready = '1' then
                    v_out := v_out + 1;
                end if;
                if core_resp.bvalid = '1' then
                    v_out := v_out - 1;
                    wr_core_error <= wr_core_error or core_resp.error;
                end if;
                wr_outst <= v_out;

                case wr_state is

                    when WR_IDLE =>
                        if s_axi_awvalid = '1' then
                            s_axi_awready <= '1'; -- AW handshake completes next cycle
                            wr_id         <= s_axi_awid;
                            wr_addr       <= unsigned(s_axi_awaddr);
                            wr_size       <= s_axi_awsize;
                            wr_beats_left <= resize(unsigned(s_axi_awlen), wr_beats_left'length) + 1;
                            wr_core_error <= '0';
                            wr_chain      <= '0';
                            if s_axi_awburst /= AXI_BURST_INCR or unsigned(s_axi_awsize) > LB then
                                wr_error     <= '1';
                                s_axi_wready <= '1';
                                wr_state     <= WR_ERR_DRAIN;
                            else
                                wr_error <= '0';
                                wr_state <= WR_WDATA;
                            end if;
                        end if;

                    when WR_ERR_DRAIN =>
                        -- absorb the W beats of a burst this design rejects
                        if s_axi_wvalid = '1' and s_axi_wready = '1' and s_axi_wlast = '1' then
                            s_axi_bvalid <= '1';
                            wr_state     <= WR_RESP;
                        else
                            s_axi_wready <= '1';
                        end if;

                    when WR_WDATA =>
                        -- Capture the beat; wready pulses next cycle, while
                        -- wvalid is still held, to complete its handshake.
                        if s_axi_wvalid = '1' then
                            s_axi_wready <= '1';
                            wr_data      <= s_axi_wdata;
                            wr_mask      <= s_axi_wstrb and beat_lanes(wr_addr(LB - 1 downto 0), wr_size);
                            wr_first_run <= '1';
                            wr_state     <= WR_BEAT;
                        end if;

                    when WR_BEAT =>
                        if wr_core_req_o.valid = '1' then
                            if wr_ready = '1' then
                                wr_core_req_o.valid <= '0';
                                if wr_cand = '1' then
                                    -- beat finished: fetch the next one now so
                                    -- its first run can continue this write
                                    wr_addr       <= next_beat_addr(wr_addr, wr_size);
                                    wr_beats_left <= wr_beats_left - 1;
                                    wr_chain      <= '1';
                                    wr_state      <= WR_WDATA;
                                else
                                    wr_state <= WR_WAIT_BVALID;
                                end if;
                            end if;
                        elsif unsigned(wr_mask) = 0 then
                            -- every enabled byte of this beat is written
                            wr_chain      <= '0';
                            wr_addr       <= next_beat_addr(wr_addr, wr_size);
                            wr_beats_left <= wr_beats_left - 1;
                            if wr_beats_left = 1 then
                                wr_state <= WR_LAST;
                            else
                                wr_state <= WR_WDATA;
                            end if;
                        else
                            -- issue the lowest contiguous run of enabled bytes
                            wr_core_req_o.valid  <= '1';
                            wr_core_req_o.we     <= '1';
                            wr_core_req_o.addr   <= std_logic_vector(wr_addr(C_MRAM_ADDR_WIDTH - 1 downto LB))
                                                    & std_logic_vector(to_unsigned(run_first, LB));
                            wr_core_req_o.nbytes <= to_unsigned(run_end - run_first, 7);
                            wr_core_req_o.wdata  <= wr_data;
                            wr_core_req_o.cont   <= wr_chain and wr_first_run;
                            wr_first_run         <= '0';
                            wr_chain             <= '0';
                            if (wr_mask and std_logic_vector(run_sum(C_AXI_STRB_WIDTH - 1 downto 0)))
                                   = (wr_mask'range => '0') and wr_beats_left > 1 then
                                wr_cand <= '1';
                            else
                                wr_cand <= '0';
                            end if;
                            wr_mask              <= wr_mask and std_logic_vector(
                                                        run_sum(C_AXI_STRB_WIDTH - 1 downto 0));
                        end if;

                    when WR_WAIT_BVALID =>
                        -- all issued writes of this burst complete
                        if v_out = 0 then
                            wr_state <= WR_BEAT;
                        end if;

                    when WR_LAST =>
                        if v_out = 0 then
                            s_axi_bvalid <= '1';
                            wr_state     <= WR_RESP;
                        end if;

                    when WR_RESP =>
                        if s_axi_bvalid = '1' and s_axi_bready = '1' then
                            s_axi_bvalid <= '0';
                            wr_state     <= WR_IDLE;
                        end if;

                end case;
            end if;
        end if;
    end process;

    s_axi_bid   <= wr_id;
    s_axi_bresp <= AXI_RESP_SLVERR when (wr_error = '1' or wr_core_error = '1') else AXI_RESP_OKAY;

    ----------------------------------------------------------------------------
    -- Read side: one process handles AR-accept (push) and R-drain (pop) so
    -- rd_fifo / rd_fifo_count have a single driver.
    ----------------------------------------------------------------------------
    -- Gated by aresetn: mram_top holds this block in reset during the boot
    -- copy while the PCI master may already be issuing AR requests, and an
    -- AR accepted during reset would be lost.
    s_axi_arready <= not rd_fifo_full and aresetn;

    process (aclk)
        variable v_push  : boolean;  -- AR accepted into the FIFO
        variable v_pop   : boolean;  -- head burst fully output
        variable v_take  : boolean;  -- issue engine takes an entry
        variable v_infl  : integer range 0 to C_RBUF + 1;
        variable v_rbcnt : integer range 0 to C_RBUF + 1;
        variable v_load  : boolean;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rd_fifo_count <= 0;
                rd_head_ptr   <= 0;
                rd_tail_ptr   <= 0;
                is_ptr        <= 0;
                is_avail      <= 0;
                is_active     <= '0';
                rb_wr         <= 0;
                rb_rd         <= 0;
                rb_cnt        <= 0;
                rd_infl       <= 0;
                s_axi_rvalid  <= '0';
                rd_core_req_o <= CORE_REQ_IDLE;
            else
                v_push  := (s_axi_arvalid = '1') and (s_axi_arready = '1');
                v_pop   := false;
                v_take  := false;
                v_infl  := rd_infl;
                v_rbcnt := rb_cnt;

                -- AR: push a new burst at the tail
                if v_push then
                    rd_fifo(rd_tail_ptr).id      <= s_axi_arid;
                    rd_fifo(rd_tail_ptr).addr    <= unsigned(s_axi_araddr);
                    rd_fifo(rd_tail_ptr).size    <= s_axi_arsize;
                    rd_fifo(rd_tail_ptr).started <= '0';
                    rd_fifo(rd_tail_ptr).beats_left <=
                        resize(unsigned(s_axi_arlen), C_AXI_LEN_WIDTH + 1) + 1;
                    if s_axi_arburst /= AXI_BURST_INCR or unsigned(s_axi_arsize) > LB then
                        rd_fifo(rd_tail_ptr).error <= '1';
                    else
                        rd_fifo(rd_tail_ptr).error <= '0';
                    end if;
                    rd_tail_ptr <= (rd_tail_ptr + 1) mod G_MAX_OUTSTANDING;
                end if;

                -- Core read data -> buffer
                if core_resp.rvalid = '1' then
                    rbuf(rb_wr)     <= core_resp.rdata;
                    rbuf_err(rb_wr) <= core_resp.error;
                    rb_wr           <= (rb_wr + 1) mod C_RBUF;
                    v_infl  := v_infl - 1;
                    v_rbcnt := v_rbcnt + 1;
                end if;

                ----------------------------------------------------------------
                -- Issue engine
                ----------------------------------------------------------------
                if is_active = '0' then
                    if is_avail > 0 then
                        -- take the next burst; a rejected one issues nothing
                        v_take := true;
                        rd_fifo(is_ptr).started <= '1';
                        is_ptr <= (is_ptr + 1) mod G_MAX_OUTSTANDING;
                        if rd_fifo(is_ptr).error = '0' then
                            is_addr   <= rd_fifo(is_ptr).addr;
                            is_size   <= rd_fifo(is_ptr).size;
                            is_left   <= rd_fifo(is_ptr).beats_left;
                            is_active <= '1';
                        end if;
                    end if;
                elsif rd_core_req_o.valid = '1' then
                    if rd_ready = '1' then
                        rd_core_req_o.valid <= '0';
                        v_infl  := v_infl + 1;
                        is_addr <= next_beat_addr(is_addr, is_size);
                        is_left <= is_left - 1;
                        if is_left = 1 then
                            is_active <= '0';
                        end if;
                    end if;
                elsif rd_infl + rb_cnt < C_RBUF then
                    -- room for its data: issue the next beat's container
                    rd_core_req_o.valid  <= '1';
                    rd_core_req_o.we     <= '0';
                    rd_core_req_o.cont   <= '0';
                    rd_core_req_o.addr   <= std_logic_vector(resize(align_down(is_addr, is_size),
                                                                    C_MRAM_ADDR_WIDTH));
                    rd_core_req_o.nbytes <= to_unsigned(beat_bytes(is_size), 7);
                end if;

                ----------------------------------------------------------------
                -- R channel: load the next beat when the output is free
                ----------------------------------------------------------------
                if s_axi_rvalid = '1' and s_axi_rready = '1' then
                    s_axi_rvalid <= '0';
                end if;
                v_load := (s_axi_rvalid = '0' or s_axi_rready = '1') and rd_fifo_count > 0
                          and rd_fifo(rd_head_ptr).started = '1'
                          and (rd_fifo(rd_head_ptr).error = '1' or rb_cnt > 0);
                if v_load then
                    if rd_fifo(rd_head_ptr).error = '1' then
                        s_axi_rdata <= (others => '0');
                        s_axi_rresp <= AXI_RESP_SLVERR;
                    else
                        s_axi_rdata <= rbuf(rb_rd);
                        s_axi_rresp <= AXI_RESP_SLVERR when rbuf_err(rb_rd) = '1' else AXI_RESP_OKAY;
                        rb_rd   <= (rb_rd + 1) mod C_RBUF;
                        v_rbcnt := v_rbcnt - 1;
                    end if;
                    s_axi_rid    <= rd_fifo(rd_head_ptr).id;
                    s_axi_rvalid <= '1';
                    if rd_fifo(rd_head_ptr).beats_left = 1 then
                        s_axi_rlast <= '1';
                        rd_head_ptr <= (rd_head_ptr + 1) mod G_MAX_OUTSTANDING;
                        v_pop := true;
                    else
                        s_axi_rlast <= '0';
                        rd_fifo(rd_head_ptr).beats_left <= rd_fifo(rd_head_ptr).beats_left - 1;
                    end if;
                end if;

                rd_infl <= v_infl;
                rb_cnt  <= v_rbcnt;

                if v_push and not v_take then
                    is_avail <= is_avail + 1;
                elsif v_take and not v_push then
                    is_avail <= is_avail - 1;
                end if;

                if v_push and not v_pop then
                    rd_fifo_count <= rd_fifo_count + 1;
                elsif v_pop and not v_push then
                    rd_fifo_count <= rd_fifo_count - 1;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
