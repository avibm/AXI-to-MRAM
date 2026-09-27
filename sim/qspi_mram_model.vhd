--------------------------------------------------------------------------------
-- qspi_mram_model.vhd  (simulation only)
--
-- Behavioural model of a Quad-SPI MRAM, just detailed enough to check the
-- mram_qspi_backend framing. It is NOT a vendor model of the AS302G208; it
-- encodes the standard SPI-memory conventions the RTL assumes (mode 0, MSB
-- first, data bytes stored at incrementing addresses starting at the
-- transmitted address, WREN required before each write). Confirm those
-- conventions against the AS302G208 datasheet before relying on this model.
--
-- Supported: 06h WREN (1-0-0), EBh RDQI (1-4-4, G_DUMMY dummy clocks),
-- D2h 4WQIO (1-4-4). Anything else is reported as an error.
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
        G_DUMMY : natural := 8;
        G_TCO   : time    := 4 ns  -- output valid after SCLK falling edge
    );
    port (
        cs_n   : in    std_logic;
        sclk   : in    std_logic;
        io     : inout std_logic_vector(3 downto 0);
        errors : out   natural := 0;
        n_rd   : out   natural := 0;  -- completed RDQI transactions
        n_wr   : out   natural := 0   -- completed 4WQIO transactions
    );
end entity qspi_mram_model;

architecture sim of qspi_mram_model is
begin

    process
        variable op      : std_logic_vector(7 downto 0);
        variable addr    : unsigned(31 downto 0);
        variable nib     : std_logic_vector(3 downto 0);
        variable byte_v  : std_logic_vector(7 downto 0);
        variable nnib    : natural;
        variable wel     : boolean := false;
        variable aborted : boolean;
        variable err     : natural := 0;
        variable rd_cnt, wr_cnt : natural := 0;

        procedure fail(msg : string) is
        begin
            err := err + 1;
            errors <= err;
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
        aborted := false;

        -- opcode, single bit on IO0, MSB first, sampled on rising SCLK
        for i in 7 downto 0 loop
            next_rise;
            exit when aborted;
            check_bits(io(0 downto 0), "opcode bit");
            op(i) := io(0);
        end loop;

        if aborted then
            fail("CS# deasserted mid-opcode");
        elsif op = x"06" then
            next_rise;
            if not aborted then
                fail("extra SCLK after WREN opcode");
            end if;
            wel := true;
        elsif op = x"EB" or op = x"D2" then
            for i in 7 downto 0 loop
                next_rise;
                exit when aborted;
                check_bits(io, "address nibble");
                addr(4 * i + 3 downto 4 * i) := unsigned(io);
            end loop;
            if aborted then
                fail("CS# deasserted during address");
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
                if not wel then
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
                        if wel then
                            mem.write(to_integer(addr(26 downto 0)), byte_v);
                        end if;
                        addr := addr + 1;
                    end if;
                    nnib := nnib + 1;
                end loop;
                if nnib mod 2 /= 0 then
                    fail("write ended on a half byte");
                end if;
                wel    := false;
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
