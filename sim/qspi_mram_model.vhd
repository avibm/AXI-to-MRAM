--------------------------------------------------------------------------------
-- qspi_mram_model.vhd  (simulation only)
--
-- Behavioural model of one die of the Avalanche AS302G208 (Dual Quad SPI
-- P-SRAM datasheet Rev. J.5), just detailed enough to check the
-- mram_qspi_backend framing and timing. It is NOT a vendor model.
--
-- Supported: 06h WREN, 04h WRDI (1-0-0); 05h RDSR, 01h WRSR, 9Fh RDID
-- (1-0-1, data out on IO1 / in on IO0); EBh RDQI and D2h 4WQIO in (1-4-4) SDR with
-- the XIP mode byte after the address (Table 29, Figure 19). The mode byte
-- must be Fxh; Axh (XIP entry) is reported as an error because the RTL must
-- never enter XIP. 65h RDAR / 71h WRAR (1-1-1, 4 address bytes; RDAR has
-- G_DUMMY latency cycles before the data, assumed equal to the CR2 read
-- latency) for SR (00h), CR1 (02h) and CR2 (03h). CR1 WRENS[1:0] (Table 23)
-- is modelled: 00 normal, 01 SRAM (array writes need no WREN), 10
-- back-to-back (WREN stays set until WRDI). Register writes always need
-- WREN. Anything else is reported as an error.
--
-- Timing checks (Tables 10, 39, 41, 43): fCLK <= 54MHz, tPU before the
-- first CS# fall, CS# high >= tCS1 (20ns) after a read and >= tCS3 (600ns)
-- after an array write, tCSS >= 5ns, tCSH >= 4ns. Read data is driven
-- G_TCO after each falling SCLK edge (datasheet maximum 9ns). RDSR/RDID
-- must run at <= 40MHz.
--
-- Status register (Table 16): WP#EN (7), TBPSEL (5), BPSEL[2:0] (4:2) are
-- writable with WRSR (needs WREN; WREN clears afterwards); WREN (1) is set
-- by WREN and cleared by WRDI or a completed write. Array writes into the
-- software-protected range (Tables 17/18, 1Gb die) are silently ignored,
-- like the real device. WP# pin protection of the status register is
-- modelled (WP#EN = 1 and IO2 low blocks WRSR).
--
-- Storage is folded to keep the model small: 4 x 16KB windows selected by
-- address bits 26:25 (so 0x0000000 and 0x6000000 are different windows),
-- offset = address bits 13:0. Tests must keep offsets below 16KB.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package qspi_mram_mem_pkg is
    type mem_pt is protected
        procedure write(addr : natural; data : std_logic_vector(7 downto 0));
        impure function read(addr : natural) return std_logic_vector;
    end protected mem_pt;

    shared variable mem : mem_pt;
    function mem_index(addr : natural) return natural;
end package qspi_mram_mem_pkg;

package body qspi_mram_mem_pkg is
    function mem_index(addr : natural) return natural is
        variable a : unsigned(31 downto 0) := to_unsigned(addr, 32);
    begin
        return to_integer(a(26 downto 25) & a(13 downto 0));
    end function;

    type mem_pt is protected body
        type arr_t is array (0 to 65535) of std_logic_vector(7 downto 0);
        variable m : arr_t := (others => x"00");
        procedure write(addr : natural; data : std_logic_vector(7 downto 0)) is
        begin
            m(mem_index(addr)) := data;
        end procedure;
        impure function read(addr : natural) return std_logic_vector is
        begin
            return m(mem_index(addr));
        end function;
    end protected body mem_pt;
end package body qspi_mram_mem_pkg;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.qspi_mram_mem_pkg.all;

entity qspi_mram_model is
    generic (
        G_DUMMY : natural := 8;       -- CR2 latency (default 8)
        G_TCO   : time    := 9 ns;    -- output valid after SCLK falling edge (max)
        G_TPU   : time    := 25 ms;   -- power-up to first instruction
        G_DEVICE_ID : std_logic_vector(31 downto 0) := x"E6212801" -- illustrative, per Table 22 fields
    );
    port (
        cs_n   : in    std_logic;
        sclk   : in    std_logic;
        io     : inout std_logic_vector(3 downto 0);
        errors : out   natural := 0;
        n_rd   : out   natural := 0;  -- completed RDQI transactions
        n_wr   : out   natural := 0;  -- completed 4WQIO transactions
        n_wren : out   natural := 0   -- WREN instructions received
    );
end entity qspi_mram_model;

architecture sim of qspi_mram_model is
    signal last_was_write : boolean := false;
    signal slow_cmd       : boolean := false; -- RDSR / RDID: 40MHz limit
    signal timing_errs    : natural := 0;
    signal proto_errs     : natural := 0;
begin

    errors <= proto_errs + timing_errs;

    -- Timing monitor
    process
        variable t_cs_rise, t_cs_fall, t_sclk_rise : time := 0 ns;
        variable min_period : time := 1 sec;
        variable first : boolean := true;
        variable err   : natural := 0;
        procedure fail(msg : string) is
        begin
            err := err + 1;
            timing_errs <= err;
            report "MRAM model timing: " & msg severity error;
        end procedure;
    begin
        wait until falling_edge(cs_n);
        t_cs_fall := now;
        if now < G_TPU then
            fail("instruction before tPU (" & time'image(now) & ")");
        end if;
        if not first then
            if now - t_cs_rise < 20 ns then
                fail("CS# high " & time'image(now - t_cs_rise) & " < tCS1 20ns");
            end if;
            if last_was_write and now - t_cs_rise < 600 ns then
                fail("CS# high " & time'image(now - t_cs_rise) & " after array write < tCS3 600ns");
            end if;
        end if;
        first := false;
        t_sclk_rise := 0 ns;
        min_period  := 1 sec;
        loop
            wait until rising_edge(sclk) or rising_edge(cs_n);
            exit when cs_n = '1';
            if t_sclk_rise = 0 ns then
                if now - t_cs_fall < 5 ns then
                    fail("tCSS < 5ns");
                end if;
            else
                if now - t_sclk_rise < 18.5 ns then
                    fail("SCLK period " & time'image(now - t_sclk_rise) & " above 54MHz");
                end if;
                if now - t_sclk_rise < min_period then
                    min_period := now - t_sclk_rise;
                end if;
            end if;
            t_sclk_rise := now;
        end loop;
        t_cs_rise := now;
        if slow_cmd and min_period < 25 ns then
            fail("RDSR/RDID SCLK period " & time'image(min_period) & " above 40MHz");
        end if;
        if t_sclk_rise /= 0 ns and now - t_sclk_rise < 4 ns then
            fail("tCSH < 4ns");
        end if;
    end process;

    process
        variable op      : std_logic_vector(7 downto 0);
        variable addr    : unsigned(31 downto 0);
        variable nib     : std_logic_vector(3 downto 0);
        variable byte_v  : std_logic_vector(7 downto 0);
        variable nnib    : natural;
        variable wel     : boolean := false;
        variable aborted : boolean;
        variable err     : natural := 0;
        variable rd_cnt, wr_cnt, wren_cnt : natural := 0;
        variable sr      : std_logic_vector(7 downto 0) := x"00"; -- WREN bit kept in wel
        variable cr1     : std_logic_vector(7 downto 0) := x"60"; -- Table 23 default
        variable cr2     : std_logic_vector(7 downto 0) := x"08";
        variable rbyte   : std_logic_vector(7 downto 0);
        variable wr_ok   : boolean;
        variable out_v   : std_logic_vector(31 downto 0);
        variable nbits   : natural;
        variable bp      : natural;
        variable a_prot  : boolean;

        -- Software block protection for a 1Gb die (Tables 17/18)
        impure function sw_protected(a : natural) return boolean is
            constant C_SIZE : natural := 2 ** 27;
            variable portion : natural;
        begin
            bp := to_integer(unsigned(sr(4 downto 2)));
            if bp = 0 then
                return false;
            end if;
            portion := C_SIZE / 2 ** (7 - bp); -- 1/64 .. all
            if sr(5) = '0' then
                return a >= C_SIZE - portion;  -- top
            else
                return a < portion;            -- bottom
            end if;
        end function;

        procedure fail(msg : string) is
        begin
            err := err + 1;
            proto_errs <= err;
            report "MRAM model: " & msg severity error;
        end procedure;

        -- wait for the next SCLK rising edge; aborted if CS# rises first
        procedure next_rise is
        begin
            wait until rising_edge(sclk) or cs_n = '1';
            aborted := (cs_n = '1');
        end procedure;

        procedure check_bits(v : std_logic_vector; what : string) is
        begin
            if is_x(v) then
                fail(what & " sampled non-0/1 value " & to_string(v) & " (contention or undriven)");
            end if;
        end procedure;
    begin
        io <= (others => 'Z');
        wait until cs_n = '0';
        aborted        := false;
        last_was_write <= false;
        slow_cmd       <= false;

        -- opcode, single bit on IO0, MSB first, sampled on rising SCLK
        for i in 7 downto 0 loop
            next_rise;
            exit when aborted;
            check_bits(io(0 downto 0), "opcode bit");
            op(i) := io(0);
        end loop;

        if aborted then
            fail("CS# deasserted mid-opcode");
        elsif op = x"06" or op = x"04" then
            next_rise;
            if not aborted then
                fail("extra SCLK after WREN/WRDI opcode");
            end if;
            wel := (op = x"06");
            if op = x"06" then
                wren_cnt := wren_cnt + 1;
                n_wren   <= wren_cnt;
            end if;
        elsif op = x"05" or op = x"9F" then
            -- RDSR / RDID: shift out on IO1 after each falling edge
            slow_cmd <= true;
            if op = x"05" then
                out_v  := (others => '0');
                out_v(31 downto 24) := sr(7 downto 2) & '1' & '0' when wel
                                       else sr(7 downto 2) & '0' & '0';
                nbits  := 8;
            else
                out_v := G_DEVICE_ID;
                nbits := 32;
            end if;
            nnib := 0;
            loop
                wait until falling_edge(sclk) or cs_n = '1';
                exit when cs_n = '1';
                io <= "ZZ" & out_v(31 - (nnib mod nbits)) & 'Z' after G_TCO;
                nnib := nnib + 1;
            end loop;
        elsif op = x"01" then
            -- WRSR: 8 bits on IO0
            for i in 7 downto 0 loop
                next_rise;
                exit when aborted;
                check_bits(io(0 downto 0), "WRSR data bit");
                byte_v(i) := io(0);
            end loop;
            if aborted then
                fail("CS# deasserted during WRSR data");
            else
                next_rise;
                if not aborted then
                    fail("extra SCLK after WRSR data");
                end if;
                if not wel then
                    report "MRAM model: WRSR without WREN ignored" severity note;
                elsif sr(7) = '1' and to_X01(io(2)) = '0' then
                    report "MRAM model: WRSR blocked by WP#" severity note;
                else
                    sr(7) := byte_v(7);
                    sr(5 downto 2) := byte_v(5 downto 2);
                end if;
            end if;
            wel := false;
        elsif op = x"65" or op = x"71" then
            -- RDAR / WRAR (1-1-1): 4 address bytes on IO0
            for i in 31 downto 0 loop
                next_rise;
                exit when aborted;
                check_bits(io(0 downto 0), "RDAR/WRAR address bit");
                addr(i) := io(0);
            end loop;
            if aborted then
                fail("CS# deasserted during RDAR/WRAR address");
            elsif op = x"65" then
                case to_integer(addr) is
                    when 0      => rbyte := sr(7 downto 2) & '1' & '0' when wel
                                            else sr(7 downto 2) & "00";
                    when 2      => rbyte := cr1;
                    when 3      => rbyte := cr2;
                    when others => rbyte := x"00";
                                   fail("RDAR of unmodelled register 0x" & to_hstring(addr));
                end case;
                -- latency (CR2[3:0], assumed = G_DUMMY), then data on IO1
                for i in 1 to G_DUMMY loop
                    next_rise;
                    exit when aborted;
                end loop;
                if aborted then
                    fail("CS# deasserted during RDAR latency");
                else
                    -- the first bit goes out after the falling edge that
                    -- follows the last latency rise
                    nnib := 0;
                    loop
                        wait until falling_edge(sclk) or cs_n = '1';
                        exit when cs_n = '1';
                        io <= "ZZ" & rbyte(7 - (nnib mod 8)) & 'Z' after G_TCO;
                        nnib := nnib + 1;
                    end loop;
                end if;
            else
                for i in 7 downto 0 loop
                    next_rise;
                    exit when aborted;
                    check_bits(io(0 downto 0), "WRAR data bit");
                    byte_v(i) := io(0);
                end loop;
                if aborted then
                    fail("CS# deasserted during WRAR data");
                else
                    next_rise;
                    if not aborted then
                        fail("extra SCLK after WRAR data");
                    end if;
                    -- register writes always need WREN (Table 23 note 1)
                    if not wel then
                        report "MRAM model: WRAR without WREN ignored" severity note;
                    elsif to_integer(addr) = 2 then
                        if byte_v(1 downto 0) = "11" then
                            fail("WRAR CR1 WRENS=11 is reserved");
                        end if;
                        cr1 := byte_v(7 downto 5) & "00" & byte_v(2 downto 0);
                    elsif to_integer(addr) = 3 then
                        cr2 := byte_v;
                    elsif to_integer(addr) = 0 then
                        sr(7) := byte_v(7);
                        sr(5 downto 2) := byte_v(5 downto 2);
                    else
                        fail("WRAR to unmodelled register 0x" & to_hstring(addr));
                    end if;
                end if;
            end if;
            wel := false;
        elsif op = x"EB" or op = x"D2" then
            for i in 7 downto 0 loop
                next_rise;
                exit when aborted;
                check_bits(io, "address nibble");
                addr(4 * i + 3 downto 4 * i) := unsigned(io);
            end loop;
            -- XIP mode byte (Figure 19): Fxh = stay in normal mode
            if not aborted then
                for i in 1 downto 0 loop
                    next_rise;
                    exit when aborted;
                    -- to_X01: a pulled-up ('H') line counts as '1'
                    byte_v(4 * i + 3 downto 4 * i) := to_X01(io);
                    check_bits(byte_v(4 * i + 3 downto 4 * i), "XIP mode nibble");
                end loop;
                if not aborted then
                    if byte_v(7 downto 4) = x"A" then
                        fail("XIP mode byte " & to_hstring(byte_v) & " enters XIP mode");
                    elsif byte_v(7 downto 4) /= x"F" then
                        fail("XIP mode byte " & to_hstring(byte_v) & " is not Fxh");
                    end if;
                end if;
            end if;
            if aborted then
                fail("CS# deasserted during address / XIP byte");
            elsif op = x"EB" then
                for i in 1 to G_DUMMY loop
                    next_rise;
                    exit when aborted;
                end loop;
                if aborted then
                    fail("CS# deasserted during dummy cycles");
                else
                    -- drive nibbles after each falling edge until CS# rises
                    nnib := 0;
                    loop
                        wait until falling_edge(sclk) or cs_n = '1';
                        exit when cs_n = '1';
                        if nnib mod 2 = 0 then
                            byte_v := mem.read(to_integer(addr(26 downto 0)));
                            io <= byte_v(7 downto 4) after G_TCO;
                        else
                            io <= byte_v(3 downto 0) after G_TCO;
                            addr := addr + 1;
                        end if;
                        nnib := nnib + 1;
                    end loop;
                    rd_cnt := rd_cnt + 1;
                    n_rd   <= rd_cnt;
                end if;
            else -- D2 write
                -- CR1 WRENS: 00 normal, 01 SRAM (no WREN), 10 back-to-back
                wr_ok := wel or cr1(1 downto 0) = "01";
                if not wr_ok then
                    fail("4WQIO without preceding WREN");
                end if;
                nnib := 0;
                loop
                    next_rise;
                    exit when aborted;
                    check_bits(io, "write data nibble");
                    if nnib mod 2 = 0 then
                        byte_v(7 downto 4) := io;
                    else
                        byte_v(3 downto 0) := io;
                        if wr_ok and not sw_protected(to_integer(addr(26 downto 0))) then
                            mem.write(to_integer(addr(26 downto 0)), byte_v);
                        end if;
                        addr := addr + 1;
                    end if;
                    nnib := nnib + 1;
                end loop;
                if nnib mod 2 /= 0 then
                    fail("write ended on a half byte");
                end if;
                -- WEL only self-clears in normal mode (back-to-back keeps
                -- it until WRDI; SRAM mode ignores it)
                if cr1(1 downto 0) = "00" then
                    wel := false;
                end if;
                last_was_write <= true;
                wr_cnt := wr_cnt + 1;
                n_wr   <= wr_cnt;
            end if;
        else
            fail("unknown opcode 0x" & to_hstring(op));
        end if;

        if cs_n /= '1' then
            wait until cs_n = '1';
        end if;
        io <= (others => 'Z');
    end process;

end architecture sim;
