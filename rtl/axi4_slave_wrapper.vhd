--------------------------------------------------------------------------------
-- axi4_slave_wrapper.vhd
--
-- AXI4 slave front end for the MRAM subsystem. Presents the same AXI4 slave
-- interface as the existing PF_SRAM_AHB_AXI block (512-bit read/write data)
-- so software and the AXI interconnect require no changes. Translates AXI
-- transactions into single-beat core_req_t/core_resp_t requests (see
-- mram_pkg) to be serviced directly by the MRAM backend, with no
-- cache in the path.
--
-- AWSIZE/ARSIZE 0..6 (1..64 bytes) are supported, so narrow accesses (e.g.
-- the small random reads/writes typical of this workload) are serviced
-- without moving a full 512-bit word each time. AxSIZE = 7 (128 bytes)
-- exceeds this bus width and is rejected with SLVERR.
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
--   * Beats within a burst are serviced strictly in order, one at a time
--     (no write pipelining, no read pipelining within a burst).
--   * Read bursts may be outstanding at the AR-channel level, up to
--     G_MAX_OUTSTANDING, but data is returned strictly in the order the
--     bursts were accepted (in-order completion).
--   * WLAST is not checked against AWLEN on the normal write path (the beat
--     count comes from AWLEN); it is used only to drain rejected bursts.
--   * 4KB-boundary crossing is not checked (the master must not do it).
--   * core_req/core_resp completion is assumed in-order.
--   * AWLOCK/AWCACHE/AWQOS/ARLOCK/ARCACHE/ARQOS/AWPROT/ARPROT are present
--     on the port for interconnect compatibility but are not acted upon.
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

    -- 2**size bytes for an AXI-style 3-bit AxSIZE field (0..6 valid on this
    -- 64-byte-wide bus; 7 is rejected before any of these are used).
    function beat_bytes(sz : std_logic_vector(2 downto 0)) return natural is
    begin
        return to_integer(shift_left(to_unsigned(1, 8), to_integer(unsigned(sz))));
    end function;

    -- Next beat address of an INCR burst: align down to size, add size.
    function next_beat_addr(a : unsigned; sz : std_logic_vector(2 downto 0)) return unsigned is
        variable r : unsigned(a'length - 1 downto 0) := a;
    begin
        r(5 downto 0) := r(5 downto 0) and not to_unsigned(beat_bytes(sz) - 1, 6);
        return r + beat_bytes(sz);
    end function;

    -- Size-aligned container of an address (read address sent to the core).
    function align_down(a : unsigned; sz : std_logic_vector(2 downto 0)) return unsigned is
        variable r : unsigned(a'length - 1 downto 0) := a;
    begin
        r(5 downto 0) := r(5 downto 0) and not to_unsigned(beat_bytes(sz) - 1, 6);
        return r;
    end function;

    -- Byte lanes a write beat may touch: from lane addr(5:0) up to the end
    -- of the size-aligned container.
    function beat_lanes(a6 : unsigned(5 downto 0); sz : std_logic_vector(2 downto 0))
        return std_logic_vector is
        variable m  : std_logic_vector(63 downto 0) := (others => '0');
        variable lo : natural range 0 to 63;
        variable hi : natural range 1 to 64;
    begin
        lo := to_integer(a6);
        hi := to_integer(a6 and not to_unsigned(beat_bytes(sz) - 1, 6)) + beat_bytes(sz);
        for i in 0 to 63 loop
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
    type wr_state_t is (WR_IDLE, WR_WDATA, WR_BEAT, WR_WAIT_BVALID, WR_RESP, WR_ERR_DRAIN);

    signal wr_state      : wr_state_t := WR_IDLE;
    signal wr_id         : std_logic_vector(G_ID_WIDTH - 1 downto 0);
    signal wr_addr       : unsigned(C_AXI_ADDR_WIDTH - 1 downto 0);
    signal wr_size       : std_logic_vector(2 downto 0) := SIZE_64B;
    signal wr_beats_left : unsigned(C_AXI_LEN_WIDTH downto 0); -- one extra bit of headroom
    signal wr_error      : std_logic := '0';
    signal wr_core_error : std_logic := '0';
    signal wr_core_req_o : core_req_t := CORE_REQ_IDLE;
    signal wr_ready      : std_logic;

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
    -- Read side: small outstanding-burst tracking FIFO (single process drives
    -- all of rd_fifo / rd_fifo_count / rd_head_ptr / rd_tail_ptr to avoid
    -- multiple-driver conflicts).
    ----------------------------------------------------------------------------
    type rd_entry_t is record
        id         : std_logic_vector(G_ID_WIDTH - 1 downto 0);
        addr       : unsigned(C_AXI_ADDR_WIDTH - 1 downto 0);
        size       : std_logic_vector(2 downto 0);
        beats_left : unsigned(C_AXI_LEN_WIDTH downto 0);
        error      : std_logic;
    end record;

    type rd_fifo_array_t is array (0 to G_MAX_OUTSTANDING - 1) of rd_entry_t;

    signal rd_fifo       : rd_fifo_array_t;
    signal rd_fifo_count : integer range 0 to G_MAX_OUTSTANDING := 0;
    signal rd_head_ptr   : integer range 0 to G_MAX_OUTSTANDING - 1 := 0;
    signal rd_tail_ptr   : integer range 0 to G_MAX_OUTSTANDING - 1 := 0;

    type rd_state_t is (RD_IDLE, RD_ISSUE, RD_WAIT_RVALID, RD_OUTPUT);
    signal rd_state : rd_state_t := RD_IDLE;

    signal rd_fifo_full  : std_logic;
    signal rd_fifo_empty : std_logic;
    signal rd_core_req_o : core_req_t := CORE_REQ_IDLE;
    signal rd_ready      : std_logic;
    signal rd_beat_error : std_logic := '0';

begin

    rd_fifo_full  <= '1' when rd_fifo_count = G_MAX_OUTSTANDING else '0';
    rd_fifo_empty <= '1' when rd_fifo_count = 0                else '0';

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
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
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

                case wr_state is

                    when WR_IDLE =>
                        if s_axi_awvalid = '1' then
                            s_axi_awready <= '1'; -- AW handshake completes next cycle
                            wr_id         <= s_axi_awid;
                            wr_addr       <= unsigned(s_axi_awaddr);
                            wr_size       <= s_axi_awsize;
                            wr_beats_left <= resize(unsigned(s_axi_awlen), wr_beats_left'length) + 1;
                            wr_core_error <= '0';
                            if s_axi_awburst /= AXI_BURST_INCR or unsigned(s_axi_awsize) > 6 then
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
                            wr_mask      <= s_axi_wstrb and beat_lanes(wr_addr(5 downto 0), wr_size);
                            wr_state     <= WR_BEAT;
                        end if;

                    when WR_BEAT =>
                        if wr_core_req_o.valid = '1' then
                            if wr_ready = '1' then
                                wr_core_req_o.valid <= '0';
                                wr_state            <= WR_WAIT_BVALID;
                            end if;
                        elsif unsigned(wr_mask) = 0 then
                            -- every enabled byte of this beat is written
                            wr_addr       <= next_beat_addr(wr_addr, wr_size);
                            wr_beats_left <= wr_beats_left - 1;
                            if wr_beats_left = 1 then
                                s_axi_bvalid <= '1';
                                wr_state     <= WR_RESP;
                            else
                                wr_state <= WR_WDATA;
                            end if;
                        else
                            -- issue the lowest contiguous run of enabled bytes
                            wr_core_req_o.valid  <= '1';
                            wr_core_req_o.we     <= '1';
                            wr_core_req_o.addr   <= std_logic_vector(wr_addr(C_AXI_ADDR_WIDTH - 1 downto 6))
                                                    & std_logic_vector(to_unsigned(run_first, 6));
                            wr_core_req_o.nbytes <= to_unsigned(run_end - run_first, 7);
                            wr_core_req_o.wdata  <= wr_data;
                            wr_mask              <= wr_mask and std_logic_vector(
                                                        run_sum(C_AXI_STRB_WIDTH - 1 downto 0));
                        end if;

                    when WR_WAIT_BVALID =>
                        if core_resp.bvalid = '1' then
                            wr_core_error <= wr_core_error or core_resp.error;
                            wr_state      <= WR_BEAT;
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
        variable v_push : boolean;
        variable v_pop  : boolean;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                rd_fifo_count <= 0;
                rd_head_ptr   <= 0;
                rd_tail_ptr   <= 0;
                rd_state      <= RD_IDLE;
                s_axi_rvalid  <= '0';
                rd_core_req_o <= CORE_REQ_IDLE;
            else
                v_push := (s_axi_arvalid = '1') and (s_axi_arready = '1');
                v_pop  := (rd_state = RD_OUTPUT) and (s_axi_rvalid = '1') and (s_axi_rready = '1')
                          and (rd_fifo(rd_head_ptr).beats_left = 1);

                -- Push: accept a new AR burst into the tail of the FIFO
                if v_push then
                    rd_fifo(rd_tail_ptr).id   <= s_axi_arid;
                    rd_fifo(rd_tail_ptr).addr <= unsigned(s_axi_araddr);
                    rd_fifo(rd_tail_ptr).size <= s_axi_arsize;
                    rd_fifo(rd_tail_ptr).beats_left <=
                        resize(unsigned(s_axi_arlen), C_AXI_LEN_WIDTH + 1) + 1;
                    if s_axi_arburst /= AXI_BURST_INCR or unsigned(s_axi_arsize) > 6 then
                        rd_fifo(rd_tail_ptr).error <= '1';
                    else
                        rd_fifo(rd_tail_ptr).error <= '0';
                    end if;

                    if rd_tail_ptr = G_MAX_OUTSTANDING - 1 then
                        rd_tail_ptr <= 0;
                    else
                        rd_tail_ptr <= rd_tail_ptr + 1;
                    end if;
                end if;

                -- Pop / service: drain the head entry one beat at a time
                case rd_state is

                    when RD_IDLE =>
                        -- Only a committed entry is looked at: an entry being
                        -- pushed this cycle is not readable until the next.
                        if rd_fifo_empty = '0' then
                            if rd_fifo(rd_head_ptr).error = '1' then
                                s_axi_rdata   <= (others => '0');
                                rd_beat_error <= '1';
                                rd_state      <= RD_OUTPUT;
                            else
                                rd_state <= RD_ISSUE;
                            end if;
                        end if;

                    when RD_ISSUE =>
                        if rd_core_req_o.valid = '1' then
                            if rd_ready = '1' then
                                rd_core_req_o.valid <= '0';
                                rd_state            <= RD_WAIT_RVALID;
                            end if;
                        else
                            rd_core_req_o.valid  <= '1';
                            rd_core_req_o.we     <= '0';
                            rd_core_req_o.addr   <= std_logic_vector(align_down(
                                                        rd_fifo(rd_head_ptr).addr, rd_fifo(rd_head_ptr).size));
                            rd_core_req_o.nbytes <= to_unsigned(beat_bytes(rd_fifo(rd_head_ptr).size), 7);
                        end if;

                    when RD_WAIT_RVALID =>
                        if core_resp.rvalid = '1' then
                            s_axi_rdata   <= core_resp.rdata;
                            rd_beat_error <= core_resp.error;
                            rd_state      <= RD_OUTPUT;
                        end if;

                    when RD_OUTPUT =>
                        s_axi_rid    <= rd_fifo(rd_head_ptr).id;
                        s_axi_rresp  <= AXI_RESP_SLVERR when rd_beat_error = '1' else AXI_RESP_OKAY;
                        s_axi_rlast  <= '1' when rd_fifo(rd_head_ptr).beats_left = 1 else '0';
                        s_axi_rvalid <= '1';

                        if s_axi_rvalid = '1' and s_axi_rready = '1' then
                            s_axi_rvalid <= '0';
                            if rd_fifo(rd_head_ptr).beats_left = 1 then
                                if rd_head_ptr = G_MAX_OUTSTANDING - 1 then
                                    rd_head_ptr <= 0;
                                else
                                    rd_head_ptr <= rd_head_ptr + 1;
                                end if;
                                rd_state <= RD_IDLE;
                            else
                                rd_fifo(rd_head_ptr).beats_left <=
                                    rd_fifo(rd_head_ptr).beats_left - 1;
                                if rd_fifo(rd_head_ptr).error = '0' then
                                    rd_fifo(rd_head_ptr).addr <= next_beat_addr(
                                        rd_fifo(rd_head_ptr).addr, rd_fifo(rd_head_ptr).size);
                                    rd_state <= RD_ISSUE;
                                end if;
                                -- a rejected burst stays here: every beat SLVERR
                            end if;
                        end if;

                end case;

                -- Single point of truth for the occupancy counter
                if v_push and not v_pop then
                    rd_fifo_count <= rd_fifo_count + 1;
                elsif v_pop and not v_push then
                    rd_fifo_count <= rd_fifo_count - 1;
                end if;

            end if;
        end if;
    end process;

end architecture rtl;
