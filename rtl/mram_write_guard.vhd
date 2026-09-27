--------------------------------------------------------------------------------
-- mram_write_guard.vhd
--
-- Gates writes to the protected boot region on a single unlock signal, so
-- that both AXI masters (RISC-V and PCI) can reach the full MRAM range
-- while PCI alone can reprogram the protected region when unlocked.
--
-- Behaviour:
--   * Reads to any address: always pass through, unconditionally.
--   * Writes outside [G_PROTECT_BASE, G_PROTECT_BASE+G_PROTECT_SIZE):
--     always pass through, unconditionally.
--   * Writes inside that range: pass through only while key_ok = '1';
--     otherwise answered with SLVERR and never reach mram_qspi_backend.
--
-- key_ok is a plain, continuously-presented level (not a one-shot pulse):
-- as long as it is held high, writes to the protected range succeed;
-- dropping it re-locks the region immediately, on the next write. The
-- comparison behind key_ok (password check or otherwise) is performed
-- entirely outside this file -- this guard trusts key_ok completely and
-- has no visibility into how it was derived. Its correctness (timing,
-- glitch-freedom, what unlocks it) is entirely the responsibility of
-- whatever drives it.
--
-- This gate keys on address + key_ok only, NOT on which AXI master issued
-- the request -- there is no master-ID input here. If strictly PCI-only
-- enforcement regardless of key_ok is required, that needs the requesting
-- master's AXI ID threaded through from axi4_slave_wrapper, which this
-- revision does not do.
--
-- Placement: sits between axi4_slave_wrapper and mram_qspi_backend --
--   axi4_slave_wrapper.core_req  -> core_req_in
--   core_req_out                 -> mram_qspi_backend.core_req
--   mram_qspi_backend.core_resp  -> core_resp_in
--   core_resp_out                -> axi4_slave_wrapper.core_resp
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity mram_write_guard is
    generic (
        G_PROTECT_BASE : natural := 100663296; -- 0x6000000: pristine master copy (top 32MB)
        G_PROTECT_SIZE : natural := 33554432   -- 32MB
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;

        key_ok : in std_logic; -- '1' = protected-region writes unlocked; comparison done externally

        core_req_in   : in  core_req_t;  -- from axi4_slave_wrapper
        core_resp_out : out core_resp_t; -- to axi4_slave_wrapper

        core_req_out  : out core_req_t;  -- to mram_qspi_backend
        core_resp_in  : in  core_resp_t  -- from mram_qspi_backend
    );
end entity mram_write_guard;

architecture rtl of mram_write_guard is

    signal is_protected_write : std_logic;
    signal blocked            : std_logic;

    signal deliver_error : std_logic := '0';

begin

    is_protected_write <= '1' when (core_req_in.we = '1' and
                                     unsigned(core_req_in.addr) >= G_PROTECT_BASE and
                                     unsigned(core_req_in.addr) < G_PROTECT_BASE + G_PROTECT_SIZE)
                           else '0';

    blocked <= '1' when (is_protected_write = '1' and key_ok = '0') else '0';

    -- Never let a blocked write reach the real backend. Reads and unlocked
    -- or out-of-range writes pass straight through.
    core_req_out <= CORE_REQ_IDLE when (core_req_in.valid = '1' and blocked = '1')
                    else core_req_in;

    process (core_req_in, blocked, deliver_error, core_resp_in)
    begin
        if deliver_error = '1' then
            core_resp_out.ready  <= '0';
            core_resp_out.rvalid <= '0';
            core_resp_out.bvalid <= '1'; -- blocked requests are always writes here
            core_resp_out.error  <= '1';
            core_resp_out.rdata  <= (others => '0');
        elsif core_req_in.valid = '1' and blocked = '1' then
            core_resp_out.ready  <= '1'; -- accept immediately, never stall the wrapper
            core_resp_out.rvalid <= '0';
            core_resp_out.bvalid <= '0';
            core_resp_out.error  <= '0';
            core_resp_out.rdata  <= (others => '0');
        else
            core_resp_out <= core_resp_in;
        end if;
    end process;

    process (aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                deliver_error <= '0';
            else
                if deliver_error = '1' then
                    deliver_error <= '0';
                elsif core_req_in.valid = '1' and blocked = '1' then
                    deliver_error <= '1';
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
