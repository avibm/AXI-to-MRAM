--------------------------------------------------------------------------------
-- mram_cmd_ctrl.vhd
--
-- Direct MRAM register commands for bring-up and debug, driven by PCI-side
-- registers: Write Enable, Write Disable, Read Status Register, Write
-- Status Register, Read Device ID. Each command has its own request input.
-- One command runs per request (WRSR does NOT send WREN by itself; issue
-- WREN first).
--
-- Handshake (four-phase, per command):
--   1. PCI raises exactly one cmd_* request and holds it.
--      For WRSR, cmd_wrsr_data must already be valid and stay stable
--      while the request is high.
--   2. The command runs on the MRAM; cmd_done rises.
--      cmd_rdsr_data / cmd_rdid_data are valid from then on and keep their
--      value until the same command runs again.
--   3. PCI lowers the request.
--   4. cmd_done falls once no request is high. The next command may be
--      raised after PCI has seen cmd_done low.
--   Raising more than one request at a time is not supported; if it
--   happens they are served in the order WREN, WRDI, RDSR, WRSR, RDID and
--   cmd_done only falls once all of them are low.
--
-- Clock domains: the cmd_* requests, cmd_wrsr_data and boot_hold come from
-- the PCI clock domain (33MHz) and are asynchronous to aclk.
--   * Each request and boot_hold goes through a G_SYNC_STAGES flop
--     synchronizer into aclk.
--   * cmd_wrsr_data is not synchronized bit by bit: it is sampled only
--     after the synchronized request is seen, i.e. at least G_SYNC_STAGES
--     aclk cycles after it was last allowed to change (rule 1 above).
--   * cmd_done is a register in aclk, passed through a G_SYNC_STAGES flop
--     synchronizer clocked by pci_clk, so the cmd_done port is synchronous
--     to pci_clk.
--   * cmd_rdsr_data / cmd_rdid_data are aclk registers that only change
--     while cmd_done is low; PCI must read them only after seeing
--     cmd_done = '1' (rule 2), which makes them stable in the PCI domain.
--   Constrain all of these crossings in the timing constraints (false path
--   or max-delay into the first synchronizer stage, and for the
--   data buses).
--
-- Commands are only started once mem_ready = '1' (the boot copy's tPU
-- power-up wait has elapsed); requests raised earlier wait.
--
-- Language: VHDL-2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.mram_pkg.all;

entity mram_cmd_ctrl is
    generic (
        G_SYNC_STAGES : positive := 2   -- synchronizer depth, >= 2
    );
    port (
        aclk    : in  std_logic;
        aresetn : in  std_logic;
        pci_clk : in  std_logic;

        -- PCI side (requests/data asynchronous to aclk)
        cmd_wren      : in  std_logic;
        cmd_wrdi      : in  std_logic;
        cmd_rdsr      : in  std_logic;
        cmd_wrsr      : in  std_logic;
        cmd_rdid      : in  std_logic;
        cmd_wrsr_data : in  std_logic_vector(7 downto 0);
        cmd_rdsr_data : out std_logic_vector(7 downto 0);
        cmd_rdid_data : out std_logic_vector(31 downto 0);
        cmd_done      : out std_logic;   -- synchronous to pci_clk

        boot_hold      : in  std_logic;  -- asynchronous
        boot_hold_sync : out std_logic;  -- synchronized to aclk

        mem_ready : in std_logic;        -- MRAM power-up time has elapsed

        -- To mram_qspi_backend (aclk)
        reg_cmd_valid  : out std_logic;
        reg_cmd_op     : out reg_cmd_t;
        reg_cmd_wdata  : out std_logic_vector(7 downto 0);
        reg_cmd_accept : in  std_logic;
        reg_cmd_done   : in  std_logic;
        reg_cmd_rdata  : in  std_logic_vector(31 downto 0)
    );
end entity mram_cmd_ctrl;

architecture rtl of mram_cmd_ctrl is

    -- bit 0 WREN, 1 WRDI, 2 RDSR, 3 WRSR, 4 RDID, 5 boot_hold
    type sync_t is array (0 to G_SYNC_STAGES - 1) of std_logic_vector(5 downto 0);
    signal req_meta : sync_t := (others => (others => '0'));
    signal req_s    : std_logic_vector(5 downto 0);

    type done_sync_t is array (0 to G_SYNC_STAGES - 1) of std_logic;
    signal done_meta : done_sync_t := (others => '0');

    type state_t is (C_IDLE, C_ISSUE, C_WAIT, C_DONE);
    signal state  : state_t := C_IDLE;
    signal op     : reg_cmd_t := REG_CMD_RDSR;
    signal done_a : std_logic := '0';

    signal rdsr_q : std_logic_vector(7 downto 0)  := (others => '0');
    signal rdid_q : std_logic_vector(31 downto 0) := (others => '0');

begin

    assert G_SYNC_STAGES >= 2 report "G_SYNC_STAGES must be >= 2" severity failure;

    ----------------------------------------------------------------------------
    -- PCI -> aclk synchronizers (no reset: they only ever carry levels)
    ----------------------------------------------------------------------------
    process (aclk)
    begin
        if rising_edge(aclk) then
            req_meta(0) <= boot_hold & cmd_rdid & cmd_wrsr & cmd_rdsr & cmd_wrdi & cmd_wren;
            for i in 1 to G_SYNC_STAGES - 1 loop
                req_meta(i) <= req_meta(i - 1);
            end loop;
        end if;
    end process;
    req_s          <= req_meta(G_SYNC_STAGES - 1);
    boot_hold_sync <= req_s(5);

    ----------------------------------------------------------------------------
    -- aclk -> PCI synchronizer for cmd_done
    ----------------------------------------------------------------------------
    process (pci_clk)
    begin
        if rising_edge(pci_clk) then
            done_meta(0) <= done_a;
            for i in 1 to G_SYNC_STAGES - 1 loop
                done_meta(i) <= done_meta(i - 1);
            end loop;
        end if;
    end process;
    cmd_done <= done_meta(G_SYNC_STAGES - 1);

    cmd_rdsr_data <= rdsr_q;
    cmd_rdid_data <= rdid_q;
    reg_cmd_op    <= op;

    ----------------------------------------------------------------------------
    -- Handshake FSM
    ----------------------------------------------------------------------------
    process (aclk)
    begin
        if rising_edge(aclk) then
            if aresetn = '0' then
                state         <= C_IDLE;
                done_a        <= '0';
                reg_cmd_valid <= '0';
            else
                case state is

                    when C_IDLE =>
                        if mem_ready = '1' and req_s(4 downto 0) /= "00000" then
                            if req_s(0) = '1' then
                                op <= REG_CMD_WREN;
                            elsif req_s(1) = '1' then
                                op <= REG_CMD_WRDI;
                            elsif req_s(2) = '1' then
                                op <= REG_CMD_RDSR;
                            elsif req_s(3) = '1' then
                                op <= REG_CMD_WRSR;
                            else
                                op <= REG_CMD_RDID;
                            end if;
                            -- stable for >= G_SYNC_STAGES cycles (see header)
                            reg_cmd_wdata <= cmd_wrsr_data;
                            reg_cmd_valid <= '1';
                            state         <= C_ISSUE;
                        end if;

                    when C_ISSUE =>
                        if reg_cmd_accept = '1' then
                            reg_cmd_valid <= '0';
                            state         <= C_WAIT;
                        end if;

                    when C_WAIT =>
                        if reg_cmd_done = '1' then
                            if op = REG_CMD_RDSR then
                                rdsr_q <= reg_cmd_rdata(7 downto 0);
                            elsif op = REG_CMD_RDID then
                                rdid_q <= reg_cmd_rdata;
                            end if;
                            done_a <= '1';
                            state  <= C_DONE;
                        end if;

                    when C_DONE =>
                        if req_s(4 downto 0) = "00000" then
                            done_a <= '0';
                            state  <= C_IDLE;
                        end if;

                end case;
            end if;
        end if;
    end process;

end architecture rtl;
