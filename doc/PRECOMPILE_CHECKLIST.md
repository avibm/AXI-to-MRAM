# Pre-compile checklist – SRAM boot copy (branch `mram_sram`)

A Libero run takes hours, so every item that can be checked before or early
in it is listed here. Items marked **(check in Libero)** depend on tool
menus or reports I cannot see; I do not name options I have not verified.
Full integration notes (port list, interconnect settings, constraints,
Identify probes): `doc/LIBERO_INTEGRATION.md`; constraints template:
`constraints/mram_top.sdc`.

## 1. Simulation and formal (all must pass before compiling)

| Command | What it covers | Expected |
|---------|----------------|----------|
| `sim/run_ghdl.sh` | MRAM AXI slave regression, register commands, boot through the SRAM model | `TEST PASSED` |
| `sim/run_dma.sh` | Mixed PCI/CPU traffic on the MRAM port | `errors=0 … model errors=0` |
| `sim/run_sram.sh` | SRAM boot copy, reload, all fault cases (T0–T15) | `TEST PASSED` |
| `sim/run_sram.sh -gG_STALL_PCT=0`, `=50 -gG_SEED=3`, `=80 -gG_SEED=7`, `=95 -gG_SEED=11` | Same under different SRAM stall patterns | `TEST PASSED` each |
| `sim/run_soak.sh -gG_ITER=120 -gG_SEED=n` | Randomized scenarios with background PCI traffic | `TEST PASSED` |
| `formal/run_pdr.sh`, `BLOCKS=1 formal/run_pdr.sh`, `BLOCKS=3 formal/run_pdr.sh` | AXI master and hand-over properties of `sram_boot_copy`, unbounded, for every input sequence | `PROVED (unbounded)` |
| `BLOCKS=1 formal/run_formal.sh 90 probe_redone` (and the other probes in Rev. 14) | The proof is not vacuous: boot, reload and failure are reachable | `COUNTEREXAMPLE` (= reachable) |

Results of these runs at the time of writing are recorded in
`doc/REVIEW.md`, Rev. 14.

## 2. Before starting the run

* **Data width:** `C_AXI_DATA_WIDTH` in `rtl/mram_pkg.vhd` = 64 (matches
  TARGET0 and INITIATOR2).
* **ID widths:** `mram_top` `s_axi_*id` are 6 bits (`C_AXI_ID_WIDTH` = 6,
  TARGET0 ID[5:0]); `m_axi_*id` are 4 bits (`C_M_AXI_ID_WIDTH` = 4 in `mram_pkg`, INITIATOR2
  ID[3:0]); `PF_SRAM_AHBL_AXI` "Width of ID" = 6. **(check in Libero)**
* **File list:** `rtl/mram_boot_copy.vhd` and `rtl/mram_top_oct1.vhd` are
  deleted; `rtl/sram_boot_copy.vhd` is new. Remove the old files from the
  Libero project and add the new one, compiled as VHDL-2008.
* **Generics on the `mram_top` instance:** defaults are for the board:
  `G_BOOT_SRC_BASE` = 0x7F00000, `G_PROTECT_BASE` = 0x7F00000,
  `G_PROTECT_SIZE` = 1 MB, `G_BOOT_DST_ADDR` = 0,
  `G_BOOT_COPY_SIZE` = 1 MB, `G_POWERUP_CYCLES` = 3,750,000 (25 ms). Make sure
  no testbench values (short power-up, small copy size) were set in
  SmartDesign.
* **New connections:**
  * `m_axi_*` → interconnect INITIATOR2 (clock `aclk`, 150 MHz).
    INITIATOR2 signals that `mram_top` does not have (for example QoS,
    region or user signals, if the configurator shows them) must be tied to
    their inactive value. **(check in Libero)**
  * `sram_reload` → the chosen source (for example a PCI register bit), or
    tie to 0. A level that is already 1 at reset is ignored; a reload needs
    a rising edge after the boot.
  * `boot_status[15:0]` → a PCI-readable status register and/or Identify.
  * The `s_axi_*id` ports widened to 6 bits: reconnect all 6.
* **Reset (important):** `aresetn` of `mram_top` must reset the interconnect's
  INITIATOR2 port at the same time. A reset in the middle of a copy drops
  `m_axi` VALIDs; that is only legal if the interconnect is reset too.
* **Address map:** SRAM 0x0000_0000–0x000F_FFFF (TARGET1), MRAM
  0x0010_0000–0x07FF_FFFF (TARGET0). Open item OA1: the interconnect must
  pass the full AXI address to TARGET0 (board test in `doc/SRAM_ARCH.md` §3).
* **Master firmware location:** the image is now read from MRAM
  0x7F00000 (AXI 0x07F0_0000), the last 1 MB, and only that 1 MB is
  write-protected. Program the master firmware there (PCI, with `key_ok`)
  before the first boot of the new bitstream; an image left at 0x6000000 is
  no longer used or protected. If the board straps the MRAM's hardware
  block protection (HBP0–2 / HTBSEL, REVIEW "Board checks"), make sure the
  new location is still writable when it must be reprogrammed.
* **SRAM IP:** ECC currently Disabled (open item OA2). Decide before the run,
  since changing it means another compile.

## 3. Early checks in the synthesis results (before place and route)

If the flow lets you run synthesis on its own first, these can be checked
within minutes of starting:

* No errors about VHDL-2008 constructs. The RTL uses record ports, reads of
  `out` ports and conditional signal assignments in processes (already used
  by the existing files).
* No inferred latches.
* `fifo` (in `sram_boot_copy`) and `rbuf` (in `axi4_slave_wrapper`) built
  from registers, as requested by `syn_ramstyle = "registers"`, not RAM
  blocks.
* `reload_q1`, `reload_q2` (sram_reload synchronizer in `mram_top`) kept
  (`syn_preserve`).
* FSM handling and safe-FSM / TMR settings as required by your policy
  (REVIEW O12). The state machine of `sram_boot_copy` has an explicit
  `when others` that recovers through the drain.
* Register count: roughly +900 flip-flops against the previous build
  (GHDL estimate, before TMR).

## 4. Timing (SmartTime)

Paths to look at first:

* The combinational accept path: `axi4_slave_wrapper` / `sram_boot_copy`
  request register → mux (`boot_own`) → `mram_write_guard` address compare →
  `mram_qspi_backend` ready → ownership mask → enables in the requester
  (REVIEW O11 a). The SRAM copy only uses `ready` as a register enable.
* `sram_boot_copy` FIFO read (16:1 × 64-bit mux) into `m_axi_wdata` and
  into the compare register.
* `axi4_slave_wrapper` `idle` (a wide AND of state) into `sram_boot_copy`.
* Asynchronous inputs: `sram_reload` into `reload_q1` needs a false-path or
  max-delay constraint, like the existing `boot_hold`, `key_ok` and `cmd_*`
  synchronizers.

## 5. First power-up on the board

1. Start with `boot_hold` = 1: `boot_status` phase = 1, CPU in reset, MRAM
   reachable from PCI. Do the OA1 full-address test (SLVERR on a write to
   0x07F0_0010 with `key_ok` = 0).
2. Release `boot_hold`: expect `boot_status` = 0x0000 (phase 0, no retries,
   no errors) after about 0.14 s, `boot_done` = 1, the CPU running from SRAM.
3. Read back SRAM from PCI and compare with the master image.
4. Pulse `sram_reload`: CPU reset, `boot_done` low for about 0.11 s, then
   `boot_status` = 0x0000 again.
5. If anything differs: `boot_status` bits 8–14 give the cause; trigger
   Identify on the phase field leaving 3/4 for 5.
