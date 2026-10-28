# SRAM execution memory – architecture

Status: **implemented (Phase 2).** Implementation was requested on
2026-10-25 together with the reload input (section 6). The RTL is
`rtl/sram_boot_copy.vhd` plus changes to `mram_top` and
`axi4_slave_wrapper`; simulation results are in section 11. Every figure
marked *estimate* is a hand calculation or a simulation, not a board
measurement. Items marked **OPEN** need an answer or a check in Libero;
they are collected in section 12.

## 1. Decisions taken (from the requirements discussion)

| # | Topic | Decision |
|---|-------|----------|
| 1 | Purpose | 1 MB on-chip SRAM is execution and working memory for NOEL-V. No cache in front of the MRAM. |
| 2 | Type | On-chip, Microchip `PF_SRAM_AHBL_AXI`, already connected to the CoreAXI4Interconnect as the second target (TARGET1) and tested on the board (read and write). |
| 3 | Address map | SRAM at AXI 0x0000_0000–0x000F_FFFF (the CPU reset vector is in it). MRAM continues at AXI 0x0010_0000–0x07FF_FFFF (configured and accepted by the interconnect). |
| 4 | Boot copy | Copy MRAM 0x7F00000 (master firmware, the last 1 MB; was 0x6000000 until 2026-10-28) → SRAM 0x0. The old MRAM → MRAM 0x0 copy is no longer done. |
| 4b | Protected region | The last 1 MB of the MRAM, 0x7F00000–0x7FFFFFF, holding the master firmware (was 0x6000000–0x7FFFFFF, 32 MB, until 2026-10-28). Everything below it is ordinary MRAM. |
| 5 | Copy size | Always 1 MB. |
| 6 | Persistence | Nothing in SRAM must survive power loss. No hardware write-back to MRAM. |
| 7 | PCI access | PCI may read and write SRAM through the interconnect at any time after boot. No hardware coherency; software owns shared buffers. |
| 8 | NOEL-V caches | Unknown. Only affects the software rules in section 7. |
| 9 | ECC | Use the ECC of `PF_SRAM_AHBL_AXI`. The IP has an ECC setting; it is currently **Disabled** in the configurator (**OPEN** OA2: enable it, and how errors are reported). |
| 10 | Compatibility | Existing `mram_top` ports kept where still meaningful. The MRAM → MRAM boot copy (`mram_boot_copy.vhd`) is removed and replaced by the SRAM boot copy, with no legacy switch. The diagnostic top `mram_top_oct1.vhd` is removed (done). `boot_hold` is kept, with the same meaning: the copy waits after the power-up time while it is 1, and AXI access to the MRAM stays open. |
| 11 | Branch | `mram_sram`, from `axi2mram`. |
| A | Copy engine | Hardware copy inside `mram_top`, writing the SRAM through a **new AXI4 master port** connected as **INITIATOR2**. The CPU is released only after the copy is verified. |

## 2. Block diagram

```
                        PCI host (32-bit / 33 MHz)
                               |
                        +-------------+
                        |  pci2mram   |  PCI bridge (CorePCIF), AXI master, 150 MHz
                        +-------------+
                               | INITIATOR0 (150 MHz)
                               v
  NOEL-V (50 MHz)    +-----------------------------+
  64-bit AXI ------->|     CoreAXI4Interconnect    |<------ INITIATOR2 (NEW, 150 MHz)
  INITIATOR1 (CDC    |  decode, arbitration, CDC   |        m_axi_* of mram_top
  inside the         +-----------------------------+                ^
  interconnect)         | TARGET0          | TARGET1                |
                        | (150 MHz)        | (150 MHz)              |
                        v                  v                        |
  +---------------------------------+   +--------------------+      |
  | mram_top                        |   | PF_SRAM_AHBL_AXI   |      |
  |                                 |   | 1 MB LSRAM, ECC    |      |
  |  s_axi_* -> axi4_slave_wrapper  |   +--------------------+      |
  |                 |               |                               |
  |                 v               |                               |
  |   +-------- mux (boot_own) <-------- sram_boot_copy (NEW) ------+
  |   |                             |      reads MRAM via core_req,  |
  |   v                             |      writes/reads SRAM via     |
  |  mram_write_guard               |      m_axi_* ; drives          |
  |   |                             |      cpu_reset_n, boot_done,   |
  |   v                             |      boot_fail, boot_status    |
  |  mram_qspi_backend <-- mram_cmd_ctrl (PCI register commands)    |
  |   |                             |
  +---|-----------------------------+
      v
   AS302G208 die 1 (QSPI, 37.5 MHz SCLK, 128 MB)
```

`sram_boot_copy` replaces `mram_boot_copy`, which is deleted. There is no
switch back to the MRAM → MRAM copy. `mram_top` keeps its existing ports
and gains the `m_axi_*` master port, `sram_reload` and `boot_status`.
Generics: `G_BOOT_DST_BASE` is replaced by `G_BOOT_DST_ADDR` (AXI address
of the SRAM, 0); `G_BOOT_COPY_SIZE` now defaults to 1 MB; new
`G_BOOT_FIFO_BLOCKS` (2) and `G_M_ID` (0). The master's ID width is the
package constant `C_M_AXI_ID_WIDTH` (4) in `mram_pkg`.

## 3. Address map

AXI addresses as seen by both initiators, as configured in the
CoreAXI4Interconnect:

| Region | AXI address | Size | Target | Notes |
|--------|-------------|------|--------|-------|
| SRAM | 0x0000_0000 – 0x000F_FFFF | 1 MB | TARGET1 | CPU reset vector, code, data, stack, heap |
| MRAM | 0x0010_0000 – 0x07FF_FFFF | 127 MB | TARGET0 | MRAM offsets 0x0–0xFFFFF are not reachable (shadowed by the SRAM) |
| MRAM master firmware | 0x07F0_0000 – 0x07FF_FFFF | 1 MB (the last 1 MB of the MRAM) | TARGET0 | Write-protected by `mram_write_guard` unless `key_ok` |

This map is identity addressing **only if the interconnect passes the full
AXI address to TARGET0** (not the address minus the 0x0010_0000 base).
`mram_top` uses AxADDR[26:0], and its boot source (0x7F00000) and
protected region (0x7F00000–0x7FFFFFF) are MRAM offsets.

* **Full address passed (assumed, OPEN OA1):** AXI address = MRAM offset
  for everything from 0x0010_0000 up. No RTL change, and the master copy
  is at AXI 0x07F0_0000 for the PCI host. The 1 MB of MRAM at offsets
  0x0–0xFFFFF is simply unused.
* **Offset passed:** every MRAM offset would be 1 MB lower than its AXI
  address; the guard and boot source would be misplaced relative to what
  the host expects, and the last 1 MB of MRAM, which now holds the master
  copy, would be unreachable from AXI, so PCI could not reprogram it. This
  would need a base-offset generic in `mram_top`; not planned unless the
  board test below shows it is needed.
* **Board test for OA1:** with `key_ok` = 0, write one word from PCI to
  AXI 0x07F0_0010. SLVERR → full address passed (the guard blocked it).
  OKAY → offset passed (the write went to MRAM 0x7E00010, general memory,
  not the master copy). Alternatively, probe `s_axi_awaddr` in Identify.
* The SRAM is at base 0, so for TARGET1 full address and offset are the
  same.
* **Consequences** (outside this RTL):
  * Firmware: still linked for and running at 0x0, now in SRAM. MRAM
    addresses ≥ 0x0010_0000 are unchanged. Anything the firmware kept at
    MRAM addresses below 0x0010_0000 and expects to survive power-off now
    lands in volatile SRAM and must move to 0x0010_0000 or above.
  * PCI host software: MRAM addresses unchanged (≥ 0x0010_0000); the
    first 1 MB of the window now reaches SRAM.
  * The firmware image at MRAM 0x7F00000 must be ≤ 1 MB, and code + data
    + bss + stack + heap must fit in 1 MB.

## 4. Clocking and CDC

* `sram_boot_copy`, its `m_axi_*` port and INITIATOR2 all run on `aclk`
  (150 MHz). TARGET1 is at 150 MHz. **No new clock domain crossing in
  this RTL.**
* The CPU's 50 MHz port is crossed inside the interconnect, as now.
* `cpu_reset_n` is an `aclk` register, as now. It goes into the CPU's
  50 MHz domain; the CPU-side reset synchronizer that exists today must
  stay (**OPEN**: confirm one exists, or I add a note to the SDC section
  in Phase 3).
* PCI register inputs (`boot_hold`, `cmd_*`, `key_ok`) keep their
  existing synchronizers.

## 5. Data flow per master

**Boot copy engine (INITIATOR2), after reset and on each reload, until
`boot_done` or `boot_fail`:**

1. Waits for the MRAM power-up time (25 ms, as now) and while
   `boot_hold` = 1.
2. **Copy pass:** reads MRAM 0x7F00000 … 0x7FFFFFF through the core
   interface as one continuous stream of beat reads. Data goes into a
   small FIFO (`G_BOOT_FIFO_BLOCKS` = 2 × 64-byte blocks, in flip-flops).
   Simulation shows this depth keeps the MRAM stream unbroken even with
   95 % random stalls on the SRAM side (section 11). Each full 64-byte block is written to SRAM as one AXI INCR
   burst (8 beats at 64 bits, all strobes on, AWSIZE = bus width, never
   crossing 4 KB). Because the writes no longer use the MRAM, the MRAM
   read never pauses and stays one long RDQI.
3. **Verify pass:** reads the MRAM source again (one stream) into the
   FIFO. For each block that is complete in the FIFO, it issues one AXI
   read burst of the same SRAM block and compares each R beat with the
   FIFO. Re-reading the MRAM, rather than using a checksum of the first
   pass, also catches an MRAM read error in the copy pass.
4. Pass → `boot_done` = 1, `cpu_reset_n` = 1, `m_axi_*` idle for good.
   Fail → retry the whole copy, up to `G_MAX_RETRIES` (3), then
   `boot_fail` = 1 with the CPU held in reset.

Failure causes: compare mismatch, BRESP or RRESP ≠ OKAY, core error from
the MRAM, watchdog (no progress for `G_WATCHDOG_LIMIT` cycles). The cause
and retry count go to a new `boot_status` output (for a PCI register or
Identify).

**NOEL-V (INITIATOR1):** fetches code and accesses data, stack and heap
in SRAM (TARGET1). Persistent data goes to MRAM (TARGET0) at AXI 0x0010_0000 and up,
at MRAM speed. Held in reset until `boot_done`.

**PCI bridge (INITIATOR0):** reaches SRAM and MRAM through the 64 MB
window. During the boot copy:
* MRAM accesses stall at the interconnect: the AXI wrapper accepts no new
  AW/AR while the copy owns the MRAM (it is no longer held in reset). The
  stall is now about 0.12 s instead of about 9 s (section 9).
* SRAM accesses are **not** blocked: TARGET1 is outside `mram_top`. A PCI
  write to SRAM during the copy would race it, and a PCI write after the
  verify of that block would not be detected. **Rule: the host must not
  access the SRAM before `boot_done`** (readable through whatever PCI
  status register carries it today).

## 6. Boot flow

```
aresetn released
   |
   v
power-up wait (25 ms) --boot_hold=1--> held: MRAM AXI port open, copy not started
   |
   v
copy pass:   MRAM 0x7F00000..+1MB  --> FIFO --> AXI write bursts --> SRAM 0x0..+1MB
   |
   v
verify pass: MRAM again --> FIFO ; SRAM read bursts --> compare
   |
   +-- error / mismatch / timeout --> drain outstanding --> retry (max 3) --> boot_fail
   |
   v
boot_done = 1, cpu_reset_n = 1  --> NOEL-V starts at 0x0 (SRAM)
```

* The full 1 MB is written before the CPU runs, so every ECC word in the
  SRAM is initialised. Reads of uninitialised memory cannot cause false
  ECC errors.
* **Abort and retry follow AXI rules** (lesson 1): once AWVALID, WVALID or
  ARVALID is raised it stays raised until accepted; BREADY and RREADY are
  held high during the whole boot and while draining; a retry starts only
  when every issued burst has received its B or its last R beat and every
  MRAM request has completed. This also closes REVIEW O7 for the new
  engine. If the drain itself does not finish within a timeout (the
  fabric is stuck), the engine sets `boot_fail` and starts no new traffic.
  It cannot legally withdraw a raised VALID, so it keeps waiting; if the
  fabric answers later and the drain completes, it clears `boot_fail` and
  retries (counted as a retry, requested 2026-10-25). A fabric that never
  answers needs `aresetn`.
* **Reload (`sram_reload`, implemented).** A rising edge on `sram_reload`
  (synchronized into `aclk` and edge-detected inside `mram_top`, so it may
  come straight from a PCI register bit) runs the copy again. The CPU
  watchdog itself is not connected to it; which source drives
  `sram_reload` is a system decision.
  * Acted on in the done or failed state: `cpu_reset_n` = 0 and
    `boot_done` = 0 (and `boot_fail` = 0) at once; no power-up wait; the
    same copy and verify as at boot; then the CPU is released. A reload
    from the failed state is how the host retries after reprogramming the
    master copy. Ignored while a copy is running and in the stuck state.
  * **The AXI side of the MRAM is not reset.** `sram_boot_copy` raises
    `mram_hold`; the wrapper (new `hold` input) stops accepting new AW/AR
    (AWREADY/ARREADY low, a legal stall), finishes every burst already
    accepted, and reports `idle`. Only then does the copy take the backend
    (`mram_own`). Backend responses are routed only to the owner. A PCI
    burst in flight therefore always completes. The same mechanism is
    used after power-up and after `boot_hold` is released, so releasing
    `boot_hold` with AXI accesses in flight is now safe.
  * PCI access to the SRAM during a reload races the copy, as at boot.
    The host sees `boot_done` = 0 while it runs.
  * Not controllable from this RTL: AXI transactions the CPU had
    outstanding when it was put into reset. Whether NOEL-V and the
    interconnect complete or drop those cleanly is a system question
    (lesson 1).

## 7. Coherency and persistence rules

* **Persistence:** SRAM is volatile. No hardware writes SRAM back to MRAM.
  Data that must survive power loss is written by firmware to MRAM
  (TARGET0). `mram_wr_pending` = 0 means posted MRAM writes have reached
  the device.
* **SRAM is one AXI target** reached by both masters through the
  interconnect, so the memory itself is coherent: each access sees the
  last completed write. Ordering between PCI and the CPU is software's job
  (flags, mailboxes, a write followed by a read-back before signalling).
* **NOEL-V caches (OPEN, unknown):** if the D-cache is write-back, PCI
  does not see CPU writes still in the cache and the CPU may read stale
  cached data after PCI writes. Rules for the firmware:
  * place PCI-shared buffers in a region the CPU treats as non-cacheable;
    or flush/invalidate around each transfer;
  * if PCI ever writes code into SRAM at run time, the CPU must execute
    `FENCE.I` before running it.
  At boot this does not arise: the copy finishes before the CPU leaves
  reset.
* No exclusive accesses or atomics are provided by this RTL; whether
  `PF_SRAM_AHBL_AXI` supports AXI exclusive access is a property of the
  IP (**OPEN**, only relevant if the firmware uses LR/SC or AMOs on SRAM
  through AXI).

## 8. Interconnect and IP settings (to check in Libero, not guessed)

| Item | Needed | What to check |
|------|--------|---------------|
| INITIATOR2 | New AXI4 initiator, 150 MHz (same clock as `aclk`, no CDC), data width = `C_AXI_DATA_WIDTH` (64), **ID 4 bits (ID[3:0], from the interconnect configuration)**; the engine drives a constant ID | How many initiators the configurator allows; its ID width for that port (sets `C_M_AXI_ID_WIDTH` in `mram_pkg`); whether per-initiator target visibility can restrict INITIATOR2 to TARGET1 |
| TARGET1 width | Same width as SRAM IP's AXI port. TARGET1 ID is 6 bits (ID[5:0]); the SRAM IP ID width was changed from 5 to 6 to match. TARGET0 is also ID[5:0]: `mram_top` now uses `C_AXI_ID_WIDTH` = 6 | The configured width, and REVIEW Rev. 10: the 64-bit TARGET0 configuration once lost PCI writes. The SRAM path already works on the board, so I take its current settings as given. |
| `PF_SRAM_AHBL_AXI` | AXI4 slave, 1 MB, ECC on | Configured (screenshot of the configurator, version 1.2.116): LSRAM; depth 131072 words × 64 bits = 1 MB; native interface off; read port "Pipelined (Address pipeline and Data pipeline)"; **ECC Disabled**; fabric interface AXI; data width 64; reset Asynchronous; "High reliability" on; AXI address width 32; ID width 5 (since changed to 6); write and read interfaces on, write and read timeout count 80. Still to check in the IP handbook: what ECC options exist and how errors are reported; how narrow/partial-strobe writes are handled with ECC; what "High reliability" does; **what the timeout counts do** (see below) |

**Timeout counts (OPEN, OA2).** I do not know what the IP does when the
write or read timeout count (80) expires; the configurator does not say.
If it ends a transaction early, or answers with an error, when a master
stalls WVALID or RREADY for more than 80 cycles, that would break lesson 3
(correct under any legal stall pattern) for the CPU through the
interconnect CDC and for the PCI bridge. Check the IP handbook. The copy
engine itself is not affected: it raises AWVALID only when the whole
64-byte block is already in its FIFO, sends the W beats without gaps, and
keeps BREADY and RREADY high.

The copy engine uses only: INCR bursts of 64 bytes (AWLEN/ARLEN =
64/beat bytes − 1), full-width beats, all WSTRB bits set, one AXI ID,
AxLOCK = 0, AxCACHE = 0000 (**OPEN** if the interconnect requires another
value), AxPROT = 000, no 4 KB crossing. Up to 2 write bursts and 2 read
bursts outstanding (generics `G_MAX_WR_OUTST`, `G_MAX_RD_OUTST` of
`sram_boot_copy`). AxQOS/AxREGION/AxUSER are not driven by this block; tie
them off in SmartDesign if INITIATOR2 has them (Phase 3).

## 9. Throughput estimates (all *estimates*)

| Path | Figure | Basis |
|------|--------|-------|
| Boot: power-up wait | 25 ms | existing `G_POWERUP_CYCLES` |
| Boot: copy + verify, 1 MB | **111.9 ms (simulated)** | full-size simulation: 2 MB of MRAM reads at ≈ 18.75 MB/s; SRAM writes and reads overlap |
| Boot total after reset, no retry | **≈ 0.14 s** | 25 ms power-up + 112 ms; vs ≈ 9 s for the old 32 MB MRAM → MRAM copy |
| Reload (no power-up wait) | ≈ 0.11 s | same copy + verify |
| Boot worst case, 3 retries | ≈ 0.47 s | 4 × 112 ms + 25 ms (estimate) |
| CPU ↔ SRAM peak | ≤ 400 MB/s | 64 bits × 50 MHz at the CPU port; real value depends on interconnect CDC latency, SRAM IP latency and NOEL-V. **Measure.** |
| PCI ↔ SRAM | limited by PCI | 32 × 33 MHz = 133 MB/s bus maximum; ≈ 70 MB/s seen from the bridge (REVIEW Rev. 6). The SRAM will not be the bottleneck, but the bridge still needs back-pressure for MRAM-bound writes. |
| CPU / PCI ↔ MRAM | unchanged | ≈ 18 MB/s streamed; 4–5 µs per 64-byte read |

## 10. Resource estimates (all *estimates*)

* `PF_SRAM_AHBL_AXI`, 1 MB: already in the build. **Read the real number
  from the current Libero resource report.** For scale: 8 Mbit / 20 Kbit
  ≈ 410 LSRAM blocks without ECC overhead; more with ECC and the chosen
  width.
* `sram_boot_copy`:
  * FIFO 2 blocks = 16 × 64-bit beats ≈ 1,024 flip-flops if kept in
    registers (lesson 4), or a few uSRAM blocks if inferred as RAM with a
    registered output. Proposed: registers, with a generic to change it.
  * Control, counters, AXI master, watchdog: roughly 300–600 flip-flops.
* `mram_boot_copy` (its 64-byte buffer, ≈ 512 flip-flops) is removed.
* GHDL synthesis count (Phase 2, rough, not Libero, before TMR):
  `sram_boot_copy` ≈ 1,600 register bits (575 plus the 1,024-bit FIFO);
  whole `mram_top` ≈ 3,230 against ≈ 2,340 before, so about +900. Only the
  Libero report is authoritative.
* Timing: the FIFO read port and the 64-bit compare are registered. No
  combinational path from any input VALID to an output READY.

## 11. Verification plan and results

New testbench `sim/tb_mram_sram.vhd`, run by `sim/run_sram.sh`, all
GHDL 4.1 / VHDL-2008:

* **DUT:** `mram_top`, the existing
  `qspi_mram_model`, and a new behavioural AXI4 slave SRAM model
  (`sim/axi_sram_model.vhd`) on `m_axi_*`.
* **AXI master protocol checker** on `m_axi_*`: VALID stays high and the
  payload stays stable until READY; WLAST on the right beat; beat count
  per burst; no 4 KB crossing; INCR only; one B per AW and ARLEN+1 R
  beats per AR; no outstanding transaction left at `boot_done`; no new
  transaction after `boot_done`.
* **Scoreboard:** after `boot_done`, the SRAM model contents equal the
  MRAM model's source region byte for byte; `cpu_reset_n` rises only
  after the last verify compare.
* **Stall patterns (lesson 3):** random and long stalls on AWREADY,
  WREADY, ARREADY, BVALID, RVALID, from a seeded generator; zero-stall
  run; a run with READY low for thousands of cycles.
* **Error injection:**
  * one corrupted byte stored by the SRAM model → verify fails → retry
    succeeds;
  * a permanently corrupted location → `boot_fail`, CPU held in reset;
  * BRESP / RRESP = SLVERR → retry;
  * B never returned → watchdog → drain timeout → `boot_fail`, with no
    protocol violation and no new traffic.
* **Other:** `boot_hold`; MRAM AXI port stalled during the copy and
  working after it (accesses at AXI 0x0010_0000 and up); reduced copy size for
  speed in the regression (for example 16 KB), plus one optional full
  1 MB run (long simulation).
* **Regression:** `sim/tb_mram_top.vhd` and `sim/tb_dma.vhd` currently
  rely on the MRAM → MRAM boot copy (T12 and the boot verify checks).
  They are moved to the SRAM boot (an AXI SRAM model on `m_axi_*`), their
  MRAM-copy checks replaced by the SRAM checks above; all other AXI-slave
  tests stay as they are and must still pass.
* **Mutation checks:** disable the verify compare, drop the drain, break
  WLAST: each must make the testbench fail.

Limits: the SRAM model is generic AXI4, **not** `PF_SRAM_AHBL_AXI`, and
there is no CoreAXI4Interconnect model. Board tests remain necessary;
Phase 3 lists the Identify probes for them.

**Results (Phase 2):** all of the above is implemented in
`sim/tb_mram_sram.vhd` (tests T0–T9, listed in its header) and passes, as
do the adapted `tb_mram_top` and `tb_dma`. Stall rates 0–95 %, data widths
64/128/512, six mutation checks: see `doc/REVIEW.md`, Rev. 13. The
reduced-size (16 KB) boot takes 1.749–1.752 ms of simulated time at every
stall rate. The full 1 MB boot (T0–T2) passes in 111.9 ms of simulated
time. The pre-compile round (`doc/REVIEW.md`, Rev. 14) added tests T10–T15,
a randomized soak test, code coverage and an unbounded formal proof of the
copy engine's AXI and hand-over properties.

## 12. Open items

| # | Item | Needed by |
|---|------|-----------|
| OA1 | Address map fixed (SRAM 0x0–0xFFFFF, MRAM 0x100000–0x7FFFFFF). Still to confirm: the interconnect passes the full address to TARGET0 (board test in section 3) | Before board test |
| OA2 | `PF_SRAM_AHBL_AXI`: width 64, ID 5, address 32 confirmed from the configurator. Still open: enable ECC (currently Disabled) and how errors are reported; partial-write handling with ECC; meaning of the write/read timeout counts (80) and of "High reliability" | ECC decision before the board build; the rest Phase 3 |
| OA3 | INITIATOR2: ID width 4 (ID[3:0]) confirmed. Still open: target visibility; AxCACHE requirements | Phase 3 |
| OA4 | NOEL-V caches and their policy, cacheable regions | Firmware rules only |
| OA5 | ~~CPU watchdog reset~~ – implemented as the `sram_reload` input (section 6); the watchdog itself is not connected to it (system decision) | Done |
| OA6 | CPU-side reset synchronizer for `cpu_reset_n` | Phase 3 |
| OA7 | Earlier questions still open: bridge name (`pci2mram` vs `pci2ddr4` in the README), why the 64-bit TARGET0 port lost PCI writes (REVIEW Rev. 10), uSRAM total (I recall ≈ 3.4 Mbit, not 13 Mbit; please check the datasheet) | Documentation |
| OA8 | Image check: read-back verify only. A corrupted master image is still copied and "verified" (REVIEW O6). CRC can be added later. | Later |
| OA9 | ~~ID width mismatch~~ – resolved: SRAM IP ID width set to 6 (ID[5:0]); `C_AXI_ID_WIDTH` changed to 6 for TARGET0 (REVIEW Rev. 12) | Done |
