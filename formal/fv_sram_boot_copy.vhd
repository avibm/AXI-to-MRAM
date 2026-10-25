--------------------------------------------------------------------------------
-- fv_sram_boot_copy.vhd  (formal verification harness, not for synthesis)
--
-- Bounded model check of sram_boot_copy with Yosys + the GHDL plugin (see
-- formal/run_formal.sh). Every input of the block is free: the solver may
-- drive the AXI slave side (READY, B, R, IDs, RLAST, responses), the MRAM
-- backend, mram_idle, reload and boot_hold in any order, including a slave
-- that breaks the AXI rules. Only the backend contract from mram_pkg is
-- assumed: read data (rvalid) only for a read that was accepted and not yet
-- answered, and no bvalid (this block never writes the MRAM). Reset is
-- applied in the first cycle only.
--
-- Small generics keep the state space small: 2 bursts of 2 beats, a FIFO
-- of one block, 1 retry, a 6-cycle watchdog, 2 power-up cycles.
--
-- Properties (all must hold in every reachable cycle after reset):
--   A1  AWVALID, once high, stays high with AWADDR unchanged until AWREADY
--   A2  WVALID, once high, stays high with WDATA/WLAST unchanged until WREADY
--   A3  ARVALID, once high, stays high with ARADDR unchanged until ARREADY
--   A4  WLAST exactly on the 2nd beat of each burst; no W beat before its
--       AW was accepted
--   A5  core_req.valid, once high, stays high with the address unchanged
--       until accepted
--   A6  no AWVALID, WVALID, ARVALID or core request while boot_done = 1
--   A7  cpu_reset_n = 1 only together with boot_done = 1
--   A8  core requests only while the block owns the MRAM; ownership only
--       while mram_hold = 1; ownership taken only in a cycle where the
--       wrapper reported idle; ownership given up only with no MRAM read
--       outstanding
--   A10 boot_done and boot_fail never both 1
--   A11 after a good boot (boot_done = 1) and in the failed state (phase 6)
--       the MRAM is back with the AXI side: mram_hold = 0, mram_own = 0
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity fv_sram_boot_copy is
    generic (G_BLOCKS : natural := 2);   -- bursts per pass (copy size = G_BLOCKS x 2 beats)
    port (
        aclk      : in std_logic;
        -- MRAM backend (free, constrained by the assumptions below)
        i_ready   : in std_logic;
        i_rvalid  : in std_logic;
        i_rerror  : in std_logic;
        i_rdata   : in std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        i_idle    : in std_logic;
        i_reload  : in std_logic;
        i_hold    : in std_logic;
        -- AXI slave side (completely free)
        i_awready, i_wready, i_bvalid, i_arready, i_rvalid_s, i_rlast : in std_logic;
        i_bresp, i_rresp : in std_logic_vector(1 downto 0);
        i_bid, i_rid     : in std_logic_vector(C_M_AXI_ID_WIDTH - 1 downto 0);
        i_rdata_s        : in std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
        -- checked by the SAT pass: ok must be 1 in every cycle in which
        -- env_ok has been 1 so far; probe_* show that the checks are not
        -- vacuous (states that must be reachable)
        ok, env_ok       : out std_logic;
        -- for unbounded checking (PDR): 1 = a property fails while the
        -- environment contract has held in every cycle so far
        bad              : out std_logic;
        probe_done, probe_fail, probe_redone, probe_unstuck : out std_logic
    );
end entity fv_sram_boot_copy;

architecture formal of fv_sram_boot_copy is
    constant BPB : natural := 2;
    signal aresetn : std_logic;
    signal rst_q, armed : std_logic := '0';

    signal req  : core_req_t;
    signal resp : core_resp_t;
    signal hold, own : std_logic;
    signal awaddr, araddr : std_logic_vector(31 downto 0);
    signal awvalid, wvalid, wlast, arvalid, bready, rready : std_logic;
    signal wdata : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0);
    signal cpu_reset_n, boot_done, boot_fail : std_logic;
    signal status : std_logic_vector(15 downto 0);

    -- previous-cycle copies
    signal p_awvalid, p_awready, p_wvalid, p_wready, p_arvalid, p_arready : std_logic := '0';
    signal p_awaddr, p_araddr : std_logic_vector(31 downto 0) := (others => '0');
    signal p_wdata  : std_logic_vector(C_AXI_DATA_WIDTH - 1 downto 0) := (others => '0');
    signal p_wlast  : std_logic := '0';
    signal p_cvalid, p_cready : std_logic := '0';
    signal p_caddr  : std_logic_vector(C_MRAM_ADDR_WIDTH - 1 downto 0) := (others => '0');
    signal p_own, p_idle : std_logic := '0';

    -- bookkeeping
    signal outst    : unsigned(3 downto 0) := (others => '0'); -- MRAM reads not answered
    signal p_outst  : unsigned(3 downto 0) := (others => '0');
    -- W beats allowed by accepted AWs and not yet sent (+BPB per AW, -1 per
    -- W). Bounded by the design's 2 outstanding writes; 6 bits cannot wrap
    -- unless the design misbehaves, which A4 reports first.
    signal w_credit : unsigned(5 downto 0) := (others => '0');
    signal w_idx    : unsigned(1 downto 0) := (others => '0');

    signal ok_a1, ok_a2, ok_a3, ok_a4, ok_a5, ok_a6, ok_a7, ok_a8, ok_a10, ok_a11 : std_logic;
    signal done_seen, refall : std_logic := '0';
    signal env_seen : std_logic := '0';   -- the contract was broken in an earlier cycle
    signal ok_i, env_i : std_logic;
begin
    -- reset in the first cycle only
    process (aclk)
    begin
        if rising_edge(aclk) then
            rst_q <= '1';
            armed <= rst_q;
        end if;
    end process;
    aresetn <= rst_q;

    -- the backend's responses reach the block only while it owns the MRAM
    resp.ready  <= i_ready and req.valid and own;
    resp.rvalid <= i_rvalid and own;
    resp.bvalid <= '0';
    resp.rdata  <= i_rdata;
    resp.error  <= i_rerror;

    dut : entity work.sram_boot_copy
        generic map (
            G_SRC_BASE => 64, G_DST_ADDR => 0, G_COPY_SIZE => G_BLOCKS * BPB * C_BEAT_BYTES,
            G_BLOCK_BYTES => BPB * C_BEAT_BYTES, G_FIFO_BLOCKS => 1, G_MAX_WR_OUTST => 2,
            G_MAX_RD_OUTST => 2, G_MAX_RETRIES => 1, G_WATCHDOG_LIMIT => 6,
            G_POWERUP_CYCLES => 2, G_M_ID => 0)
        port map (
            aclk => aclk, aresetn => aresetn,
            core_req => req, core_resp => resp,
            mram_hold => hold, mram_idle => i_idle, mram_own => own,
            m_axi_awid => open, m_axi_awaddr => awaddr, m_axi_awlen => open,
            m_axi_awsize => open, m_axi_awburst => open, m_axi_awlock => open,
            m_axi_awcache => open, m_axi_awprot => open,
            m_axi_awvalid => awvalid, m_axi_awready => i_awready,
            m_axi_wdata => wdata, m_axi_wstrb => open, m_axi_wlast => wlast,
            m_axi_wvalid => wvalid, m_axi_wready => i_wready,
            m_axi_bid => i_bid, m_axi_bresp => i_bresp, m_axi_bvalid => i_bvalid,
            m_axi_bready => bready,
            m_axi_arid => open, m_axi_araddr => araddr, m_axi_arlen => open,
            m_axi_arsize => open, m_axi_arburst => open, m_axi_arlock => open,
            m_axi_arcache => open, m_axi_arprot => open,
            m_axi_arvalid => arvalid, m_axi_arready => i_arready,
            m_axi_rid => i_rid, m_axi_rdata => i_rdata_s, m_axi_rresp => i_rresp,
            m_axi_rlast => i_rlast, m_axi_rvalid => i_rvalid_s, m_axi_rready => rready,
            reload => i_reload, boot_hold => i_hold,
            cpu_reset_n => cpu_reset_n, boot_done => boot_done, boot_fail => boot_fail,
            powerup_done => open, boot_held => open, boot_status => status);

    process (aclk)
        variable v_out : unsigned(3 downto 0);
    begin
        if rising_edge(aclk) then
            p_awvalid <= awvalid; p_awready <= i_awready; p_awaddr <= awaddr;
            p_wvalid  <= wvalid;  p_wready  <= i_wready;  p_wdata  <= wdata; p_wlast <= wlast;
            p_arvalid <= arvalid; p_arready <= i_arready; p_araddr <= araddr;
            p_cvalid  <= req.valid; p_cready <= resp.ready; p_caddr <= req.addr;
            p_own     <= own; p_idle <= i_idle; p_outst <= outst;

            v_out := outst;
            if req.valid = '1' and resp.ready = '1' then v_out := v_out + 1; end if;
            if resp.rvalid = '1' then v_out := v_out - 1; end if;
            outst <= v_out;

            if (awvalid = '1' and i_awready = '1') and not (wvalid = '1' and i_wready = '1') then
                w_credit <= w_credit + BPB;
            elsif (awvalid = '1' and i_awready = '1') and (wvalid = '1' and i_wready = '1') then
                w_credit <= w_credit + BPB - 1;
            elsif wvalid = '1' and i_wready = '1' then
                w_credit <= w_credit - 1;
            end if;
            if wvalid = '1' and i_wready = '1' then
                w_idx <= (w_idx + 1) mod BPB;
            end if;
        end if;
    end process;

    -- Environment contract: read data only for an outstanding read
    env_i  <= '1' when not (i_rvalid = '1' and own = '1' and outst = 0) else '0';
    env_ok <= env_i;

    ok_a1 <= '0' when armed = '1' and p_awvalid = '1' and p_awready = '0'
                      and (awvalid = '0' or awaddr /= p_awaddr) else '1';
    ok_a2 <= '0' when armed = '1' and p_wvalid = '1' and p_wready = '0'
                      and (wvalid = '0' or wdata /= p_wdata or wlast /= p_wlast) else '1';
    ok_a3 <= '0' when armed = '1' and p_arvalid = '1' and p_arready = '0'
                      and (arvalid = '0' or araddr /= p_araddr) else '1';
    ok_a4 <= '0' when armed = '1' and wvalid = '1'
                      and ((wlast = '1') /= (w_idx = BPB - 1) or w_credit = 0) else '1';
    ok_a5 <= '0' when armed = '1' and p_cvalid = '1' and p_cready = '0'
                      and (req.valid = '0' or req.addr /= p_caddr) else '1';
    ok_a6 <= '0' when armed = '1' and boot_done = '1'
                      and (awvalid = '1' or wvalid = '1' or arvalid = '1' or req.valid = '1') else '1';
    ok_a7 <= '0' when armed = '1' and cpu_reset_n = '1' and boot_done = '0' else '1';
    ok_a8 <= '0' when armed = '1' and ((req.valid = '1' and own = '0')
                                     or (own = '1' and hold = '0')
                                     or (p_own = '0' and own = '1' and p_idle = '0')
                                     or (p_own = '1' and own = '0' and p_outst /= 0)) else '1';
    ok_a10 <= '0' when armed = '1' and boot_done = '1' and boot_fail = '1' else '1';
    ok_a11 <= '0' when armed = '1' and (boot_done = '1' or status(2 downto 0) = "110")
                       and (hold = '1' or own = '1') else '1';

    ok_i <= ok_a1 and ok_a2 and ok_a3 and ok_a4 and ok_a5 and ok_a6 and ok_a7 and ok_a8
            and ok_a10 and ok_a11;
    ok   <= ok_i;
    process (aclk)
    begin
        if rising_edge(aclk) then
            if env_i = '0' then env_seen <= '1'; end if;
        end if;
    end process;
    bad <= (not ok_i) and env_i and not env_seen;

    -- reachability probes
    process (aclk)
    begin
        if rising_edge(aclk) then
            if boot_done = '1' then done_seen <= '1'; end if;
            if done_seen = '1' and boot_done = '0' then refall <= '1'; end if;
        end if;
    end process;
    probe_done        <= boot_done;
    probe_fail        <= boot_fail;
    probe_redone      <= boot_done and refall;   -- boot_done again after a reload
    probe_unstuck     <= boot_done and status(13);  -- a good boot after the stuck state
end architecture formal;
