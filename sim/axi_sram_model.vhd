--------------------------------------------------------------------------------
-- axi_sram_model.vhd  (simulation only)
--
-- Behavioural AXI4 slave memory standing in for the SRAM behind the
-- interconnect. NOT a model of PF_SRAM_AHBL_AXI or CoreAXI4Interconnect:
-- it follows the AXI4 rules only, with randomised stalls, so the master
-- under test must be correct for any legal timing.
--
--   * INCR bursts (FIXED/WRAP answered with SLVERR, data ignored), any
--     AxSIZE up to the bus width, WSTRB honoured. Responses in order.
--   * W beats are taken only for an accepted AW (the master under test
--     sends W after AW; a W beat with no AW is counted as an error).
--   * Random stalls on AWREADY, WREADY, ARREADY, and random delays before
--     BVALID and each RVALID, G_STALL_PCT percent of cycles each (seeded).
--     stall_all = '1' stalls every channel for as long as it is high.
--   * Error injection, driven by the testbench:
--       inj_b    : count of B responses to answer with SLVERR (from now on)
--       inj_r    : count of R beats to answer with SLVERR
--       inj_c    : count of write bursts in which the byte at corrupt_addr
--                  is stored corrupted (XOR 5Ah)
--       corrupt_always = '1': every write to corrupt_addr is corrupted
--       stop_b   = '1': no BVALID at all (a stuck target)
--     The counts are cumulative: raise inj_b from 0 to 1 for one error,
--     later to 2 for another.
--   * Backdoor: package axi_sram_mem_pkg, shared variable sram.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package axi_sram_mem_pkg is
    type sram_pt is protected
        procedure write(addr : natural; data : std_logic_vector(7 downto 0));
        impure function read(addr : natural) return std_logic_vector;
    end protected sram_pt;
    shared variable sram : sram_pt;
    constant C_SRAM_MODEL_BYTES : natural := 2097152; -- 2MB: room to check writes past 1MB
end package axi_sram_mem_pkg;

package body axi_sram_mem_pkg is
    type sram_pt is protected body
        type arr_t is array (0 to C_SRAM_MODEL_BYTES - 1) of std_logic_vector(7 downto 0);
        variable m : arr_t := (others => x"00");
        procedure write(addr : natural; data : std_logic_vector(7 downto 0)) is
        begin
            m(addr mod C_SRAM_MODEL_BYTES) := data;
        end procedure;
        impure function read(addr : natural) return std_logic_vector is
        begin
            return m(addr mod C_SRAM_MODEL_BYTES);
        end function;
    end protected body sram_pt;
end package body axi_sram_mem_pkg;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.axi_sram_mem_pkg.all;

entity axi_sram_model is
    generic (
        G_ID_WIDTH  : natural := 4;
        G_DATA_W    : natural := 64;
        G_SEED      : natural := 1;
        G_STALL_PCT : natural := 30   -- 0 = never stall
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;
        s_axi_awid    : in  std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_awaddr  : in  std_logic_vector(31 downto 0);
        s_axi_awlen   : in  std_logic_vector(7 downto 0);
        s_axi_awsize  : in  std_logic_vector(2 downto 0);
        s_axi_awburst : in  std_logic_vector(1 downto 0);
        s_axi_awvalid : in  std_logic;
        s_axi_awready : out std_logic := '0';
        s_axi_wdata   : in  std_logic_vector(G_DATA_W - 1 downto 0);
        s_axi_wstrb   : in  std_logic_vector(G_DATA_W / 8 - 1 downto 0);
        s_axi_wlast   : in  std_logic;
        s_axi_wvalid  : in  std_logic;
        s_axi_wready  : out std_logic := '0';
        s_axi_bid     : out std_logic_vector(G_ID_WIDTH - 1 downto 0) := (others => '0');
        s_axi_bresp   : out std_logic_vector(1 downto 0) := "00";
        s_axi_bvalid  : out std_logic := '0';
        s_axi_bready  : in  std_logic;
        s_axi_arid    : in  std_logic_vector(G_ID_WIDTH - 1 downto 0);
        s_axi_araddr  : in  std_logic_vector(31 downto 0);
        s_axi_arlen   : in  std_logic_vector(7 downto 0);
        s_axi_arsize  : in  std_logic_vector(2 downto 0);
        s_axi_arburst : in  std_logic_vector(1 downto 0);
        s_axi_arvalid : in  std_logic;
        s_axi_arready : out std_logic := '0';
        s_axi_rid     : out std_logic_vector(G_ID_WIDTH - 1 downto 0) := (others => '0');
        s_axi_rdata   : out std_logic_vector(G_DATA_W - 1 downto 0) := (others => '0');
        s_axi_rresp   : out std_logic_vector(1 downto 0) := "00";
        s_axi_rlast   : out std_logic := '0';
        s_axi_rvalid  : out std_logic := '0';
        s_axi_rready  : in  std_logic;

        stall_all      : in  std_logic := '0';
        stop_b         : in  std_logic := '0';
        inj_b, inj_r, inj_c : in natural := 0;
        corrupt_addr   : in  natural := 0;
        corrupt_always : in  std_logic := '0';
        n_aw, n_ar     : out natural := 0;   -- accepted bursts
        errors         : out natural := 0    -- W beat without AW, WLAST mismatch
    );
end entity axi_sram_model;

architecture sim of axi_sram_model is
    constant NB    : natural := G_DATA_W / 8;
    constant DEPTH : natural := 8;
    type burst_t is record
        id    : std_logic_vector(G_ID_WIDTH - 1 downto 0);
        addr  : natural;
        len   : natural;   -- beats - 1
        size  : natural;   -- bytes per beat
        bad   : boolean;   -- unsupported burst type
    end record;
    type q_t is array (0 to DEPTH - 1) of burst_t;
    type bq_t is array (0 to DEPTH - 1) of std_logic_vector(G_ID_WIDTH + 1 downto 0);
begin

    process (aclk)
        variable lfsr : unsigned(31 downto 0) := to_unsigned(16#ACE1# + G_SEED * 7919, 32);
        variable awq, arq : q_t;
        variable aw_h, aw_c, ar_h, ar_c : natural := 0;   -- head index, count
        variable bq  : bq_t;
        variable b_h, b_c : natural := 0;
        variable w_beat, r_beat : natural := 0;
        variable nb_inj, nr_inj, nc_inj : natural := 0;
        variable c_this : boolean := false;               -- current W burst may corrupt
        variable c_hit  : boolean := false;               -- it did corrupt corrupt_addr
        variable errs, naw, nar : natural := 0;
        variable a : natural;
        variable bt : burst_t;
        variable rd : std_logic_vector(G_DATA_W - 1 downto 0);

        impure function stall return boolean is
        begin
            lfsr := lfsr(30 downto 0) & (lfsr(31) xor lfsr(21) xor lfsr(1) xor lfsr(0));
            return stall_all = '1' or (to_integer(lfsr(15 downto 0)) mod 100) < G_STALL_PCT;
        end function;

        function beat_addr(b : burst_t; n : natural) return natural is
        begin
            if n = 0 then
                return b.addr;
            end if;
            return (b.addr / b.size) * b.size + n * b.size;
        end function;

        function to_burst(id : std_logic_vector; ad, ln : std_logic_vector; sz, bu : std_logic_vector)
            return burst_t is
            variable r : burst_t;
        begin
            r.id   := id;
            r.addr := to_integer(unsigned(ad(30 downto 0)));
            r.len  := to_integer(unsigned(ln));
            r.size := 2 ** to_integer(unsigned(sz));
            r.bad  := bu /= "01" or r.size > NB;
            return r;
        end function;
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                aw_h := 0; aw_c := 0; ar_h := 0; ar_c := 0; b_h := 0; b_c := 0;
                w_beat := 0; r_beat := 0;
                s_axi_awready <= '0'; s_axi_wready <= '0'; s_axi_bvalid <= '0';
                s_axi_arready <= '0'; s_axi_rvalid <= '0';
            else
                -- AW
                if s_axi_awvalid = '1' and s_axi_awready = '1' then
                    awq((aw_h + aw_c) mod DEPTH) := to_burst(s_axi_awid, s_axi_awaddr, s_axi_awlen,
                                                             s_axi_awsize, s_axi_awburst);
                    aw_c := aw_c + 1;
                    naw  := naw + 1;
                end if;
                s_axi_awready <= '1' when aw_c < DEPTH - 1 and not stall else '0';

                -- W (into the oldest AW whose data is not complete)
                if s_axi_wvalid = '1' and s_axi_wready = '1' then
                    if aw_c = 0 then
                        errs := errs + 1;
                        report "SRAM model: W beat without an accepted AW" severity error;
                    else
                        bt := awq(aw_h);
                        if w_beat = 0 then
                            c_this := corrupt_always = '1' or nc_inj < inj_c;
                            c_hit  := false;
                        end if;
                        if not bt.bad then
                            a := beat_addr(bt, w_beat);
                            for i in 0 to NB - 1 loop
                                -- lanes of this beat's size-aligned container
                                if s_axi_wstrb(i) = '1' and i >= a mod NB
                                   and i < (a / bt.size) * bt.size mod NB + bt.size then
                                    if c_this and (a / NB) * NB + i = corrupt_addr then
                                        sram.write((a / NB) * NB + i,
                                                   s_axi_wdata(8 * i + 7 downto 8 * i) xor x"5A");
                                        c_hit := true;
                                    else
                                        sram.write((a / NB) * NB + i, s_axi_wdata(8 * i + 7 downto 8 * i));
                                    end if;
                                end if;
                            end loop;
                        end if;
                        if (s_axi_wlast = '1') /= (w_beat = bt.len) then
                            errs := errs + 1;
                            report "SRAM model: WLAST on beat " & integer'image(w_beat)
                                   & " of a burst of " & integer'image(bt.len + 1) severity error;
                        end if;
                        if w_beat = bt.len then
                            if c_hit and corrupt_always = '0' then
                                nc_inj := nc_inj + 1;
                            end if;
                            if bt.bad then
                                bq((b_h + b_c) mod DEPTH) := "10" & bt.id;
                            else
                                bq((b_h + b_c) mod DEPTH) := "00" & bt.id;
                            end if;
                            b_c    := b_c + 1;
                            aw_h   := (aw_h + 1) mod DEPTH;
                            aw_c   := aw_c - 1;
                            w_beat := 0;
                        else
                            w_beat := w_beat + 1;
                        end if;
                    end if;
                end if;
                s_axi_wready <= '1' when aw_c > 0 and b_c < DEPTH - 1 and not stall else '0';

                -- B
                if s_axi_bvalid = '1' and s_axi_bready = '1' then
                    s_axi_bvalid <= '0';
                    b_h := (b_h + 1) mod DEPTH;
                    b_c := b_c - 1;
                end if;
                if (s_axi_bvalid = '0' or s_axi_bready = '1') and b_c > 0 and stop_b = '0'
                   and not stall then
                    s_axi_bid <= bq(b_h)(G_ID_WIDTH - 1 downto 0);
                    if nb_inj < inj_b then
                        s_axi_bresp <= "10";
                        nb_inj := nb_inj + 1;
                    else
                        s_axi_bresp <= bq(b_h)(G_ID_WIDTH + 1 downto G_ID_WIDTH);
                    end if;
                    s_axi_bvalid <= '1';
                elsif s_axi_bvalid = '1' and s_axi_bready = '1' then
                    s_axi_bvalid <= '0';
                end if;

                -- AR
                if s_axi_arvalid = '1' and s_axi_arready = '1' then
                    arq((ar_h + ar_c) mod DEPTH) := to_burst(s_axi_arid, s_axi_araddr, s_axi_arlen,
                                                             s_axi_arsize, s_axi_arburst);
                    ar_c := ar_c + 1;
                    nar  := nar + 1;
                end if;
                s_axi_arready <= '1' when ar_c < DEPTH - 1 and not stall else '0';

                -- R
                if s_axi_rvalid = '1' and s_axi_rready = '1' then
                    s_axi_rvalid <= '0';
                    if r_beat = arq(ar_h).len then
                        ar_h   := (ar_h + 1) mod DEPTH;
                        ar_c   := ar_c - 1;
                        r_beat := 0;
                    else
                        r_beat := r_beat + 1;
                    end if;
                end if;
                if (s_axi_rvalid = '0' or s_axi_rready = '1') and ar_c > 0 and not stall then
                    bt := arq(ar_h);
                    a  := beat_addr(bt, r_beat);
                    for i in 0 to NB - 1 loop
                        rd(8 * i + 7 downto 8 * i) := sram.read((a / NB) * NB + i);
                    end loop;
                    s_axi_rid   <= bt.id;
                    s_axi_rdata <= rd;
                    s_axi_rlast <= '1' when r_beat = bt.len else '0';
                    if bt.bad then
                        s_axi_rresp <= "10";
                    elsif nr_inj < inj_r then
                        s_axi_rresp <= "10";
                        nr_inj := nr_inj + 1;
                    else
                        s_axi_rresp <= "00";
                    end if;
                    s_axi_rvalid <= '1';
                end if;
            end if;
            errors <= errs;
            n_aw   <= naw;
            n_ar   <= nar;
        end if;
    end process;

end architecture sim;
