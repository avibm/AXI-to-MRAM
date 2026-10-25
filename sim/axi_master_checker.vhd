--------------------------------------------------------------------------------
-- axi_master_checker.vhd  (simulation only)
--
-- Passive AXI4 protocol checker for the master side of one AXI port (here
-- the m_axi_* port of the SRAM boot copy). Every violation is reported and
-- counted in errors.
--
--   * AW, W, AR: once VALID is high it stays high, with the payload
--     unchanged, until READY (AXI4 A3.2.1). No 'U'/'X'/'Z' on VALID/READY.
--   * AW/AR: INCR only, AxSIZE within the bus width, address aligned to
--     AxSIZE, no 4KB boundary crossing.
--   * W: beats counted against the AWLEN of the bursts in AW order; WLAST
--     exactly on the last beat; WSTRB all ones (this master writes full
--     beats). With G_W_AFTER_AW, no W beat before its AW is accepted.
--   * Responses (slave side, counted in resp_errors, not errors): no B
--     without a completed write burst, no R without an outstanding AR,
--     RLAST exactly on the last beat of each AR. A response that breaks
--     these rules is not counted as completing anything.
--   * quiet = '1' (the testbench drives it after boot_done): any VALID from
--     the master is an error.
--   * aw_outst / ar_outst: accepted AWs without B, ARs without RLAST.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi_master_checker is
    generic (
        G_ID_WIDTH  : natural := 4;
        G_DATA_W    : natural := 64;
        G_W_AFTER_AW : boolean := true;
        G_NAME      : string := "m_axi"
    );
    port (
        aclk, aresetn : in std_logic;
        awid    : in std_logic_vector(G_ID_WIDTH - 1 downto 0);
        awaddr  : in std_logic_vector(31 downto 0);
        awlen   : in std_logic_vector(7 downto 0);
        awsize  : in std_logic_vector(2 downto 0);
        awburst : in std_logic_vector(1 downto 0);
        awvalid, awready : in std_logic;
        wdata   : in std_logic_vector(G_DATA_W - 1 downto 0);
        wstrb   : in std_logic_vector(G_DATA_W / 8 - 1 downto 0);
        wlast, wvalid, wready : in std_logic;
        bvalid, bready : in std_logic;
        arid    : in std_logic_vector(G_ID_WIDTH - 1 downto 0);
        araddr  : in std_logic_vector(31 downto 0);
        arlen   : in std_logic_vector(7 downto 0);
        arsize  : in std_logic_vector(2 downto 0);
        arburst : in std_logic_vector(1 downto 0);
        arvalid, arready : in std_logic;
        rlast, rvalid, rready : in std_logic;
        quiet   : in std_logic := '0';
        errors  : out natural := 0;       -- master-side violations
        resp_errors : out natural := 0;   -- slave-side violations
        aw_outst, ar_outst : out natural := 0
    );
end entity axi_master_checker;

architecture sim of axi_master_checker is
    constant NB : natural := G_DATA_W / 8;
begin
    process (aclk)
        variable errs, rerrs : natural := 0;
        variable ar_h2, ar_c2, r_beat : natural := 0;
        -- previous cycle
        variable p_awv, p_awr, p_wv, p_wr, p_arv, p_arr : std_logic := '0';
        variable p_aw : std_logic_vector(G_ID_WIDTH + 32 + 8 + 3 + 2 - 1 downto 0);
        variable p_ar : std_logic_vector(G_ID_WIDTH + 32 + 8 + 3 + 2 - 1 downto 0);
        variable p_w  : std_logic_vector(G_DATA_W + G_DATA_W / 8 downto 0);
        -- AWLEN of accepted bursts, in order, for the W beat count
        type len_q_t is array (0 to 63) of natural;
        variable lens : len_q_t;
        variable arlens : len_q_t;
        variable l_h, l_c : natural := 0;
        variable w_beat : natural := 0;
        variable w_done, n_b, n_aw, n_ar, n_rl : natural := 0;
        variable aw_v, ar_v : std_logic_vector(p_aw'range);

        procedure bad(msg : string) is
        begin
            errs := errs + 1;
            report G_NAME & " protocol: " & msg severity error;
        end procedure;

        procedure check_addr(ch : string; addr : std_logic_vector(31 downto 0);
                             len : std_logic_vector(7 downto 0); sz, bu : std_logic_vector) is
            variable a, s, n : natural;
        begin
            a := to_integer(unsigned(addr(30 downto 0)));
            s := 2 ** to_integer(unsigned(sz));
            n := to_integer(unsigned(len)) + 1;
            if bu /= "01" then bad(ch & ": burst type " & to_string(bu) & ", only INCR expected"); end if;
            if s > NB then bad(ch & ": size above the bus width"); end if;
            if a mod s /= 0 then bad(ch & ": address not aligned to size"); end if;
            if a mod 4096 + n * s > 4096 then bad(ch & ": burst crosses a 4KB boundary"); end if;
        end procedure;

        function is01(v : std_logic) return boolean is
        begin
            return v = '0' or v = '1';
        end function;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                p_awv := '0'; p_wv := '0'; p_arv := '0';
                p_awr := '0'; p_wr := '0'; p_arr := '0';
                l_h := 0; l_c := 0; w_beat := 0; w_done := 0; n_b := 0; n_aw := 0;
                n_ar := 0; n_rl := 0; ar_h2 := 0; ar_c2 := 0; r_beat := 0;
            else
                if not (is01(awvalid) and is01(wvalid) and is01(arvalid) and is01(bready)
                        and is01(rready) and is01(awready) and is01(wready) and is01(arready)
                        and is01(bvalid) and is01(rvalid)) then
                    bad("VALID/READY not 0/1");
                end if;

                aw_v := awid & awaddr & awlen & awsize & awburst;
                ar_v := arid & araddr & arlen & arsize & arburst;

                -- VALID held, payload stable until READY
                if p_awv = '1' and p_awr = '0' then
                    if awvalid /= '1' then bad("AWVALID dropped before AWREADY"); end if;
                    if aw_v /= p_aw then bad("AW payload changed while AWVALID waited"); end if;
                end if;
                if p_arv = '1' and p_arr = '0' then
                    if arvalid /= '1' then bad("ARVALID dropped before ARREADY"); end if;
                    if ar_v /= p_ar then bad("AR payload changed while ARVALID waited"); end if;
                end if;
                if p_wv = '1' and p_wr = '0' then
                    if wvalid /= '1' then bad("WVALID dropped before WREADY"); end if;
                    if (wlast & wstrb & wdata) /= p_w then bad("W payload changed while WVALID waited"); end if;
                end if;

                if quiet = '1' and (awvalid = '1' or wvalid = '1' or arvalid = '1') then
                    bad("VALID raised while quiet (after boot_done)");
                end if;

                if awvalid = '1' and awready = '1' then
                    check_addr("AW", awaddr, awlen, awsize, awburst);
                    lens((l_h + l_c) mod 64) := to_integer(unsigned(awlen));
                    l_c  := l_c + 1;
                    n_aw := n_aw + 1;
                end if;
                if wvalid = '1' and wready = '1' then
                    if l_c = 0 then
                        if G_W_AFTER_AW then bad("W beat before its AW was accepted"); end if;
                    else
                        if wstrb /= (wstrb'range => '1') then bad("WSTRB not all ones"); end if;
                        if (wlast = '1') /= (w_beat = lens(l_h)) then
                            bad("WLAST on beat " & integer'image(w_beat) & " of a burst of "
                                & integer'image(lens(l_h) + 1));
                        end if;
                        if w_beat = lens(l_h) then
                            w_beat := 0;
                            l_h    := (l_h + 1) mod 64;
                            l_c    := l_c - 1;
                            w_done := w_done + 1;
                        else
                            w_beat := w_beat + 1;
                        end if;
                    end if;
                end if;
                if bvalid = '1' and bready = '1' then
                    if n_b >= w_done then
                        rerrs := rerrs + 1;
                        report G_NAME & " slave: B before its write burst completed" severity note;
                    else
                        n_b := n_b + 1;
                    end if;
                end if;
                if arvalid = '1' and arready = '1' then
                    check_addr("AR", araddr, arlen, arsize, arburst);
                    arlens((ar_h2 + ar_c2) mod 64) := to_integer(unsigned(arlen));
                    ar_c2 := ar_c2 + 1;
                    n_ar  := n_ar + 1;
                end if;
                if rvalid = '1' and rready = '1' then
                    if ar_c2 = 0 then
                        rerrs := rerrs + 1;
                        report G_NAME & " slave: R beat with no outstanding AR" severity note;
                    else
                        if (rlast = '1') /= (r_beat = arlens(ar_h2)) then
                            rerrs := rerrs + 1;
                            report G_NAME & " slave: RLAST on the wrong beat" severity note;
                        end if;
                        if r_beat = arlens(ar_h2) then
                            r_beat := 0;
                            ar_h2  := (ar_h2 + 1) mod 64;
                            ar_c2  := ar_c2 - 1;
                            n_rl   := n_rl + 1;
                        else
                            r_beat := r_beat + 1;
                        end if;
                    end if;
                end if;

                p_awv := awvalid; p_awr := awready; p_aw := aw_v;
                p_arv := arvalid; p_arr := arready; p_ar := ar_v;
                p_wv  := wvalid;  p_wr  := wready;  p_w  := wlast & wstrb & wdata;
            end if;
            errors   <= errs;
            resp_errors <= rerrs;
            aw_outst <= n_aw - n_b;
            ar_outst <= n_ar - n_rl;
        end if;
    end process;
end architecture sim;
