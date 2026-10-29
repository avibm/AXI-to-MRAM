# Second MRAM die (SPI1) – architecture (draft for approval)

Status: **draft, not approved; no RTL yet.** Requested 2026-10-29. Builds on
`doc/SRAM_ARCH.md` (SRAM boot copy) and the current `mram_top`. Every figure
marked *estimate* is a calculation, not a measurement. Items marked
**OPEN** need an answer before or during the RTL work (section 11).

## 1. Requirements and decisions

| # | Topic | Decision |
|---|-------|----------|
| R1 | Second die | Add the second die of the AS302G208 as a second Quad-SPI interface. **SPI1** = the new interface, **SPI2** = the existing one (unchanged behaviour). |
| R2 | Protection | All 128 MB of SPI1 are write-protected by a new key, **`key1_ok`**. SPI2 keeps `key_ok` for its last 1 MB (0x7F00000–0x7FFFFFF). |
| R3 | Reads | SPI1 reads are always allowed; only writes need `key1_ok` (same rule as the existing guard). |
| R4 | Status / ID to PCI | `cmd_rdsr_data` (RDSR/RDAR result) and `cmd_rdid_data` (RDID result) show the selected die through a mux: **`key1_ok` = 1 → SPI1, else SPI2.** |
| R5 | Register commands | WREN, WRDI, RDSR, WRSR, RDID, RDAR, WRAR run on the **selected die only** (same select, `key1_ok`). One `cmd_done`, same four-phase handshake as today. |
| R6 | Address map | **SPI1 is a new interconnect target**, chosen for timing (section 7): TARGET2 at AXI 0x0800_0000–0x0FFF_FFFF. SPI2 stays at 0x0010_0000–0x07FF_FFFF, so firmware, host software and the boot copy keep their addresses. **(Proposal, please confirm.)** |
| R7 | Boot copy | Unchanged: copies from SPI2 (0x7F00000) to the SRAM. SPI1 takes no part in booting. |

Consequence of R4 + R2, recorded because it was a choice: while `key1_ok`
= 1 to read SPI1's status or ID, all of SPI1 is also writable by any AXI
master (CPU or PCI). The guard keys on the address and the key only; it
does not know which master writes (as for SPI2, REVIEW O9).

## 2. Block diagram

```
                         CoreAXI4Interconnect
        TARGET0 (MRAM SPI2)      TARGET1 (SRAM)       TARGET2 (MRAM SPI1, NEW)
             |                        |                        |
+------------|------------------------|------------------------|-----------+
| mram_top   v                        |                        v           |
|  axi4_slave_wrapper (SPI2)          |       axi4_slave_wrapper (SPI1)    |
|       |   ^ hold/idle               |              ^ hold = not tPU done |
|       v   |                         |              |                     |
|  mux(boot_own) <- sram_boot_copy --m_axi--> INITIATOR2                   |
|       |                                            |                     |
|  mram_write_guard (key_ok,                 mram_write_guard (key1_ok,    |
|   0x7F00000..0x7FFFFFF)                     0x0000000..0x7FFFFFF)        |
|       |                                            |                     |
|  mram_qspi_backend (SPI2) <--+          +--> mram_qspi_backend (SPI1)   |
|       |                      |          |            |                   |
|       |              command router (select latched at command start)    |
|       |                      ^          |                                |
|       |               mram_cmd_ctrl (one, PCI handshake)                  |
|       |                      |                                           |
|       |        result registers per die -> output mux (key1_ok sync)     |
|       v                                            v                     |
|   mram_cs_n/sclk/io[3:0]                 mram1_cs_n/sclk/io[3:0]  (NEW)  |
+--------------------------------------------------------------------------+
```

## 3. Address map

| Region | AXI address | Target | Write rule |
|--------|-------------|--------|------------|
| SRAM | 0x0000_0000 – 0x000F_FFFF | TARGET1 | — |
| MRAM SPI2 | 0x0010_0000 – 0x07EF_FFFF | TARGET0 | free |
| MRAM SPI2 master firmware | 0x07F0_0000 – 0x07FF_FFFF | TARGET0 | `key_ok` |
| **MRAM SPI1** | **0x0800_0000 – 0x0FFF_FFFF** | **TARGET2 (new)** | **`key1_ok`, whole 128 MB** |

* `mram_top` uses AxADDR[26:0] for each die. TARGET2's base is 128 MB
  aligned, so for SPI1 it makes no difference whether the interconnect
  passes the full address or the offset (unlike TARGET0, open item OA1).
* The PCI window is 64 MB: the host reaches SPI1 by moving the window base
  (BAR + 0x80) into 0x0800_0000–0x0FFF_FFFF, as it does today for SPI2.
* The CPU and PCI need access to TARGET2; the boot copy (INITIATOR2) does
  not.

## 4. Data path per die

SPI1 gets its own instance of each existing block, with no new RTL in
them:
* `axi4_slave_wrapper` – the same AXI4 slave (INCR, narrow, strobes, read
  streaming, fair read/write arbitration). Its `hold` input is driven by
  "MRAM power-up time not yet elapsed", so SPI1 AXI accesses stall until
  the 25 ms tPU has passed, exactly as SPI2's do today. No boot copy owns
  SPI1, so after power-up it is never held.
* `mram_write_guard` – generics `G_PROTECT_BASE` = 0, `G_PROTECT_SIZE` =
  128 MB, `key_ok => key1_ok` (synchronized inside, as today). Every SPI1
  write without the key is answered SLVERR and never reaches the device.
* `mram_qspi_backend` – the same engine and generics (posted writes,
  streaming, linger, read sample delay, WREN-once).

The two dies are fully independent: each has its own AXI port, its own
SPI pins and SCLK, and both can transfer at the same time (≈ 18.75 MB/s
each, *estimate* from the measured stream rate).

Per-die inputs and outputs (new ports for SPI1, defaults keep today's
behaviour):

| SPI2 (existing) | SPI1 (new) | Note |
|-----------------|------------|------|
| `s_axi_*` | `s1_axi_*` | AXI4 slave, 64-bit, ID 6 bits |
| `key_ok` | `key1_ok` | write key; `key1_ok` is also the result-mux select |
| `rd_sample_dly` | `rd_sample_dly1` | read sample delay; board delays may differ per die (default "010") |
| `skip_wren` | `skip_wren1` | WREN-once, per die because CR1 is per die (default '0') |
| `mram_wr_pending`, `mram_rd_slip` | `mram1_wr_pending`, `mram1_rd_slip` | status |
| `mram_cs_n`, `mram_sclk`, `mram_io[3:0]` | `mram1_cs_n`, `mram1_sclk`, `mram1_io[3:0]` | pins (OPEN P1) |

## 5. Register commands and the result mux

* **One `mram_cmd_ctrl`**, unchanged PCI handshake. Its command goes to
  one backend only.
* **Routing select, latched per command.** `key1_ok` is synchronized into
  `aclk` (two flops, its own synchronizer). When a command starts (C_IDLE →
  C_ISSUE), the synchronized value is latched (`cmd_die`) and holds for the
  whole command: `reg_cmd_valid` goes to that backend, and only that
  backend's `accept`, `done` and `rdata` are taken. A change of `key1_ok`
  in the middle of a command therefore cannot lose the `done` and hang the
  handshake. (Taking the live select instead would allow exactly that.)
* **Result registers per die.** RDSR/RDAR and RDID results are stored per
  die (`rdsr_q1`, `rdid_q1` for SPI1; the existing `rdsr_q`, `rdid_q` for
  SPI2). `cmd_rdsr_data` / `cmd_rdid_data` = the selected die's registers,
  selected by the synchronized `key1_ok` (a registered mux, so the outputs
  stay aclk registers). The PCI side reads them as today: after its
  synchronized `cmd_done` = 1, with `key1_ok` already set to the die it
  wants, and without changing `key1_ok` until it has read them.
* **Power-up:** commands wait for tPU (as today); both dies share it.
* The backend of the unselected die is never sent a command, so its
  `reg_cmd_valid` is 0 and its memory traffic is unaffected.

Software sequence, for example "read SPI1's ID": set `key1_ok` = 1 (SPI1
writes are now open, see section 1), raise `cmd_rdid`, wait for `cmd_done`
= 1, read `cmd_rdid_data`, lower `cmd_rdid`, wait for `cmd_done` = 0, set
`key1_ok` = 0.

## 6. Clocking, reset, CDC

* Everything on `aclk` (150 MHz); SPI1 SCLK = aclk / 4 = 37.5 MHz like SPI2.
* `aresetn` resets both dies' logic (as today: a command or transfer in
  progress is cut off, REVIEW Rev. 14).
* New asynchronous inputs, each through a two-flop synchronizer:
  `key1_ok`, `rd_sample_dly1`, `skip_wren1` (the last two quasi-static, as
  their SPI2 counterparts). `key1_ok` is synchronized twice: inside the
  SPI1 `mram_write_guard` (as `key_ok` is for SPI2) and in
  `mram_cmd_ctrl` for the command routing and the result mux. The two
  copies can differ by a cycle when `key1_ok` changes; that is harmless
  because the guard decides per write and the router per command.
* SDC: the same PCI ↔ aclk false paths cover the new inputs; add MRAM1 pin
  budgets like SPI2's (`constraints/mram_top.sdc`).

## 7. Timing (why a separate target)

* SPI2's critical path (request register → guard address compare →
  backend `ready` → back to the requester, REVIEW O11 a) is untouched: no
  logic is added to it. SPI1 has its own copy of the same path, which
  meets timing if SPI2's does (same logic, though placement differs).
* The alternative of one 256 MB target would put a die-select (address
  bit 27) mux on the 64-bit request and response records, plus logic to
  return responses in AXI order from two dies, directly on that path.
* The command router adds the latched select (`cmd_die`, one register)
  and a 2:1 mux on `accept` / `done` / 32-bit `rdata`; the result mux is a
  registered 2:1 mux. All small, all between registers.

## 8. Resource estimate (all *estimates*)

* One more wrapper + guard + backend: roughly +1,500 to +2,000 register
  bits before TMR (the existing `mram_top` without the boot copy is about
  1,600–1,700 bits by the GHDL count). Only the Libero report is
  authoritative.
* Router and result registers: about 50 bits.
* Two more SPI pin sets: 6 I/O.

## 9. Implementation steps

1. `rtl/mram_cmd_ctrl.vhd`: `key1_ok` synchronizer, latched `cmd_die`,
   second set of `reg_cmd_*` ports (or a die-indexed pair), per-die result
   registers, output mux.
2. `rtl/mram_top.vhd`: second wrapper / guard / backend instance, new
   ports from section 4, SPI1 wrapper `hold` from the tPU flag, pins.
   Existing ports and generics unchanged; new SPI1 generics default to
   SPI2's values.
3. No change in `axi4_slave_wrapper`, `mram_write_guard`,
   `mram_qspi_backend`, `sram_boot_copy`.
4. Simulation: the MRAM model gets one storage per instance (today it is a
   single shared variable in a package, so two instances would share it):
   a generic selects one of two backdoor memories.
5. Docs: README, `doc/LIBERO_INTEGRATION.md` (ports, TARGET2 settings,
   SDC), checklist, REVIEW entry.

## 10. Verification plan

* `tb_mram_top` / new `tb_dual_die`: two MRAM models, both AXI ports.
  * SPI1: reads at the first and last address without the key (OKAY, data
    correct); writes without the key at the first, a middle and the last
    64 bytes → SLVERR, unchanged; with the key → written.
  * SPI2 unchanged: all existing tests, including T8/T8b (its own key and
    region), with `key1_ok` both 0 and 1 (no cross-effect between keys).
  * Concurrent traffic on both dies, data checked on each, so a fault on
    one die cannot corrupt the other.
  * Register commands: RDID / RDSR / WRSR / RDAR / WRAR on each die,
    selected by `key1_ok`; the command reaches only the selected die (each
    model counts its commands); result mux follows `key1_ok`.
  * `key1_ok` toggled in the middle of a command: the command completes
    on the die that was selected at its start, `cmd_done` behaves, no hang.
  * SPI1 accesses before tPU stall, then complete.
  * Boot copy from SPI2 unaffected by SPI1 traffic.
* Existing regressions (`run_ghdl`, `run_dma`, `run_sram`, soak, formal)
  unchanged and passing.
* Mutation checks: live (unlatched) select, guard size wrong, swapped
  result registers – each must fail.
* Coverage of the changed `mram_cmd_ctrl` and `mram_top`.

## 11. Open items

| # | Item |
|---|------|
| P1 | **Pins:** which AS302G208 pins does each interface drive? Today's design uses die 1 (CS1#, CLK1, IO[3:0]); the proposal assumes SPI1 (new) = die 2 (CS2#, CLK2, IO[7:4]) and SPI2 (existing) keeps die 1. If your naming means the other way round, only the pin assignment changes, not the RTL. The REVIEW board note "CS2# needs a pull-up because die 2 is unused" no longer applies; RESET# and the WP#/IO2 and IO3 pull-ups apply to die 2 too. |
| P2 | **Address map:** confirm SPI1 at TARGET2, 0x0800_0000–0x0FFF_FFFF (R6). Interconnect: a fourth target, 64-bit, ID 6 bits, `aclk`, reachable from INITIATOR0 and INITIATOR1. |
| P3 | `key1_ok` source: a PCI register with its own password logic, like `key_ok`? (outside this RTL) |
| P4 | Per-die `rd_sample_dly1` / `skip_wren1` (proposed) or shared with SPI2? |
| P5 | Should SPI1's status bits (`mram1_wr_pending`, `mram1_rd_slip`) also be readable through the `key1_ok` mux, or as separate PCI status bits (proposed)? |
