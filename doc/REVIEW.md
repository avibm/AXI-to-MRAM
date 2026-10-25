# MRAM Subsystem – Design Review (rev. 6)

Scope: `doc/MRAM_Subsystem_IP_Specification.pdf` (Sep 24, 2026) and the six
VHDL files in `rtl/` as first committed (commit "Add MRAM subsystem RTL and IP
specification as received"). The follow-up commit contains the fixes listed
here; `git diff` between the two commits shows every change.

## How this was checked

* All files compile and elaborate with GHDL 4.1 (`--std=08`). The original
  files did too, so none of the problems below are syntax errors.
* A self-checking testbench (`sim/tb_mram_top.vhd`) drives `mram_top` through
  an AXI4 master model and a behavioural Quad-SPI MRAM model
  (`sim/qspi_mram_model.vhd`). Every write is checked byte-by-byte in the MRAM
  model, including bytes that must stay unchanged, and every read is compared
  with the model. Run it with `sim/run_ghdl.sh`.
* The corrected design passes: boot copy + verify, 20 AXI writes and 54 AXI
  reads, 0 MRAM protocol errors.
* The testbench was checked for sensitivity by re-introducing two of the
  bugs one at a time. Both made it fail.

Limits of this review. Nothing here replaces the following:

* The MRAM model is **not** a vendor model. It encodes standard SPI-memory
  conventions: mode 0, MSB first, bytes stored at incrementing addresses, WREN
  before each write. I did not have the AS302G208 datasheet. Every
  device-specific point is marked *verify against datasheet*.
* The design has **not** been synthesized, timing-closed, or run on hardware.
  Timing and resource statements are hand estimates.
* Only one simulator was used. VHDL-2008 support in the Libero/Synplify flow
  still needs checking (see O10).

## Findings in the RTL

Severity: **Critical** = the design does not work; **High** = wrong data,
deadlock, or an AXI protocol violation under realistic traffic; **Medium** =
margin, robustness, or portability.

### Critical

| # | File | Problem | Evidence | Fix |
|---|------|---------|----------|-----|
| C1 | wrapper, boot_copy, backend | **Handshake livelock.** Requesters raised `valid` for one cycle, then dropped it and re-raised it. The backend's `ready` was registered, so it arrived one cycle late, exactly when `valid` was low. `valid='1' and ready='1'` never happened. The backend kept re-executing the same request and the requester never advanced. | Simulation of the original: the same read repeats every ~4 µs and `boot_done` never asserts. | `ready` is now combinational (`valid` while idle). Requesters hold `valid` until they are accepted. The contract is written down in `mram_pkg`. |
| C2 | backend | **SPI bit timing off by one.** Each bit was first driven on a falling SCLK edge, but mode 0 starts with a rising edge. The device sampled an undriven line first, and the last bit of every phase was never clocked. | Original simulation: opcode `EBh` arrives as `75h` (and `X5h` on the first transaction). | Engine rewritten. Each phase counts rising edges, and the first bit/nibble is set up before the first rising edge. |
| C3 | backend | **Byte order reversed on the wire.** For both writes and reads the highest lane was sent first, so the byte for address `A+n-1` was stored at `A`. Accesses of the same size and address round-trip. Mixed sizes do not: a 64-byte write followed by a 4-byte read returns the wrong bytes. Data written by any other path (for example, a master image programmed externally) reads back reversed. | Analysis. The fixed design is covered by narrow-vs-wide read checks. Mutation test: reverting the fix makes boot verification fail. | Lowest address first, lane-aligned (`byte_rev` + a single barrel shift each way). |
| C4 | backend | **WSTRB ignored.** All `2**size` bytes were written whatever the strobes said, so partial writes overwrote neighbouring bytes. | Analysis. Covered by the T5 sparse-strobe test. | The core interface now carries a byte count. The wrapper splits each beat into one write per contiguous run of enabled strobes. Beats with no strobes write nothing. |
| C5 | wrapper, backend | **Unaligned AXI start address.** The backend wrote `2**size` bytes from the unaligned address, running past the beat's container and filling with zeros. Later beats of a narrow burst were not re-aligned. | Covered by the T7 unaligned narrow burst test. | AXI INCR rules: only lanes from `addr` to the end of the size-aligned container are writable, later beats are size-aligned, and reads fetch the aligned container. |
| C6 | wrapper | **Shared `ready` not steered.** The read engine took the backend's acceptance of a *write* as acceptance of its own read, then waited forever for an `rvalid` that never came. This happens whenever reads and writes overlap. | Mutation test: removing the fix hangs the testbench. | Each engine sees `ready` only while its own request is the one on `core_req`. |
| C7 | top | **PCI traffic during the boot copy.** `guard_resp` goes to both the boot copy and the wrapper. The mux comment assumes no AXI traffic while the CPU is in reset, but the **PCI master is not in reset**. A PCI access during the ~9 s copy would consume the boot copy's responses and get the boot image's data back. | Analysis. Covered by the T12 early-read test. | The wrapper is held in reset until `boot_done`, so AXI requests stall at the interconnect. See O8. |

### High

| # | File | Problem | Fix |
|---|------|---------|-----|
| H1 | wrapper | **Duplicate B response.** `WR_RESP`/`WR_ERR_RESP` tested `bready` without `bvalid`, and `bvalid` was set again on the handshake cycle. With `bready` low at first and then high, two B handshakes occurred for one write. | Handshake is `bvalid and bready`, and `bvalid` is cleared on it. The testbench counts B handshakes. |
| H2 | wrapper | **Rejected-burst W drain.** Beats were counted on `wvalid` without `wready`, and `wready` stayed high one cycle after `WLAST`. That could swallow the first W beat of the next burst (AXI allows W before AW). | Drain uses `wvalid and wready`, and `wready` drops on the last beat. |
| H3 | wrapper | `RD_IDLE` decided from the FIFO head **in the same cycle it was being written** (`or v_push`), so it used a stale `error` flag. An unsupported burst could reach the backend with size 7, which is out of range, or a good burst could be answered with SLVERR. | Only committed entries are read. |
| H4 | wrapper | After the first SLVERR beat of a rejected read burst with `ARLEN>0`, the FSM issued **real core reads with the invalid size**. | A rejected burst stays in `RD_OUTPUT` and returns SLVERR for every beat. |
| H5 | write_guard | For one cycle the pending SLVERR **replaced the whole backend response**. A backend `rvalid` for an outstanding read in that cycle was lost, and the read hung. | The error waits for a cycle with no backend completion. New requests are held back while it is pending. |
| H6 | wrapper | `ARREADY` was combinational from the FIFO count, so it was **high during reset** and accepted AR requests that were then lost. The original design had the same issue during `aresetn`, but it became reachable with the C7 fix. Found by the testbench. | `ARREADY` is gated by `aresetn`. |

### Medium

| # | Problem | Status |
|---|---------|--------|
| M1 | At the end of a read the last SCLK high phase was shortened to one aclk cycle. | Fixed: the clock always stops on a falling edge. |
| M2 | `key_ok` was used without synchronization (spec open item). | Fixed: two-flop synchronizer, generic `G_SYNC_KEY_OK` (default true). |
| M3 | Read error was sticky across the remaining beats of a burst. | Fixed: RRESP is per beat. |
| M4 | `2 ** to_integer(x)` with a variable exponent. Synthesis support varies. | Replaced with shifts. |
| M5 | Boot copy `when others` did not drop `core_req.valid`. | Fixed. |
| M6 | Boot-copy header claimed "~1.3–1.5 s" for the copy. The spec says 8.64 s. Simulation gives ≈16.6 µs per 64-byte chunk, i.e. ≈8.7 s per 32 MB pass. | Comment corrected (≈9 s per pass, ≈35 s with 3 retries). |

## Rev. 2 – hardware failure and datasheet check

**Symptom on hardware.** Identify showed `mram_boot_copy` in `S_FAIL` after
all retries. The verify pass failed at a random chunk, and during the read
data phase the IO bus sat at `F` (pull-ups): the MRAM was not driving it.

**Root cause.** The device was checked against the Avalanche datasheet "1Gbit – 8Gbit
Dual Quad SPI P-SRAM", Rev. J.5 (Table 29, Figure 19). RDQI `EBh` and 4WQIO
`D2h` both carry an **XIP mode byte** after the 4 address bytes: 2 quad clocks,
`Axh` = enter XIP, `Fxh` = normal. The RTL did not send it. Two things followed:

* **Writes:** the first data byte was taken as the mode byte. The rest of the
  data landed one byte low. Any chunk whose first byte was `Ax` switched the
  die into XIP mode, where it no longer expects an opcode. From then on it no
  longer responded to normal commands, which matches the floating bus seen on
  hardware.
* **Reads:** data was sampled 2 clocks early, one byte off.

The behavioural MRAM model in rev. 1 used the same wrong framing. It was
written from generic SPI conventions without the datasheet, so the simulation
could not catch this. The model now follows the datasheet. Run against the
rev. 1 RTL it reports the missing mode byte and the timing violations below.

| # | Finding (datasheet reference) | Fix |
|---|------------------------------|-----|
| D1 | Missing XIP mode byte in EBh/D2h (Table 29 "XIP" column, Figure 19) | `S_XIP` state sends `G_XIP_BYTE` = `FFh` after the address, for reads and writes. |
| D2 | CS# high time after a memory-array write must be ≥ 600 ns (tCS3, Table 39); the design gave ~13–80 ns | `S_CS_HIGH` wait: `G_CS_HIGH_WRITE_CYCLES` = 92 (613 ns @150 MHz). |
| D3 | CS# high time after a read must be ≥ 20 ns (tCS1); the design gave 13.3 ns | `G_CS_HIGH_READ_CYCLES` = 4 (26.7 ns). |
| D4 | No instruction before tPU = 25 ms after power-up/RESET (1 ms for the -A variant) (Tables 10/11) | Boot copy waits `G_POWERUP_CYCLES` = 3,750,000 (25 ms) before its first request. The watchdog is held off during that wait. |
| D5 | Max SCLK is **54 MHz** SDR, not 108 MHz (Tables 29, 39) | Comments corrected. `G_SCLK_HALF_PERIOD` must be ≥ 2 at 150 MHz; the default is 37.5 MHz. |
| D6 | Read latency: CR2 default is 8 cycles, valid for (1-4-4) SDR up to 54 MHz (Tables 25, 26) | `G_DUMMY_CYCLES` = 8 confirmed; closes O1 as long as CR2 is left at its default. |
| D7 | Output valid tCO ≤ 9 ns after the falling edge (Table 43). Sampling half a period later leaves ~4 ns at 37.5 MHz for FPGA and board delays | Confirmed on hardware in rev. 5: insufficient on the first board. Fixed with `rd_sample_dly`. |

### Rev. 3 – register commands for bring-up

`mram_cmd_ctrl` adds PCI-driven WREN, WRDI, RDSR, WRSR and RDID commands, a
`boot_hold` input and the PCI → aclk synchronizers. `cmd_done` is an aclk
register that the PCI side synchronizes. The README describes
the ports and handshake. Timing constraints are needed for these crossings:
treat the paths into the first synchronizer stages, and the
`cmd_wrsr_data` / `cmd_rdsr_data` / `cmd_rdid_data` buses, as false paths
or give them a max-delay constraint.

Things these commands can reveal when "writes don't apply":

* **RDID** should read back as the manufacturer ID `E6h` in bits 31:24
  (Table 22). If it doesn't, basic SPI communication is broken: check pins,
  RESET# and the clock.
* **RDSR** shows the write-protection state:
  * BPSEL[2:0] ≠ 0 means part of the array is software write-protected.
  * WP#EN only matters together with the WP# pin.
  * After WREN, bit 1 should read 1. If it doesn't, WREN is not reaching
    the device.
* The hardware protect pins **HBP0–2 / HTBSEL** do not show up in RDSR.

### Rev. 4 – bring-up aids

* AXI access to the MRAM is open while `boot_hold` = 1 and after `boot_fail`,
  not only after `boot_done`. Before this, a PCI read or write test with the
  boot copy held off, or on a board whose boot failed, simply stalled.
* The CS# high time between WREN and the write (`G_CS_GAP_CYCLES`) is 92
  cycles (613 ns) instead of 4. The datasheet gives no figure after WREN, so
  it now uses the 600 ns tCS3 value.

### Rev. 5 – read sampling one SCLK late on hardware

With CS#/SCLK connected, RDID on the board returned `0x73109480`. That is
`0` followed by `0xE6212901` (Table 22: E6 · dual-quad · 3 V · −40…125 °C ·
2 Gb · 54 MHz) shifted right by one bit. The chip works; the FPGA sampled
every bit one SCLK late. Sampling half an SCLK period (13.3 ns at 37.5 MHz)
after the chip launches a bit was not enough to cover FPGA output delay,
board delay both ways, tCO and FPGA input delay. This is D7 in practice.
The same lateness shifts quad reads by one nibble, which on its own makes
the boot verify fail.

Fix: a runtime input `rd_sample_dly` (0–7 `aclk` cycles after the SCLK
rising edge, default 2) moves the read sample point. The simulation model
reproduces the board failure (20 ns delay, setting 0) and passes with 2;
see the README table.

### Rev. 6 – write throughput

On hardware, large PCI writes left holes. Cause, on the PCI side: the
bridge `pci2ddr4` has no flow control. PCI delivers about 70 MB/s against
about 9 MB/s into the MRAM, and once its FIFO (≈16 KB) is full, data is
dropped. That bridge needs back-pressure. Two changes in this IP raise the
MRAM rate (estimates, see the README):

* **Write continuation.** A new `cont` field on `core_req` lets the
  wrapper extend a running D2h write with the next contiguous strobe run
  of the same AXI burst. The backend accepts it while still shifting data
  (`cont_ok`) and appends it without raising CS#. Each 64-byte burst at a
  0x20 offset is now 1 MRAM write instead of 2, and a 4-beat burst is 1
  instead of 4. One B response per AXI burst, as before; errors from any
  part are OR-ed into it.
* **WREN-once.** New PCI commands RDAR (65h) and WRAR (71h), plus a
  `skip_wren` input, allow CR1 WRENS = 01 (Table 23), so writes skip the
  WREN and its CS# gap. Read the warning about `skip_wren` at power-up in
  the README.

Simulation: the model now handles RDAR/WRAR on SR/CR1/CR2, the WRENS modes,
and counts WRENs. The testbench checks the write counts for merged and
split bursts, and that no WREN is sent with `skip_wren` = 1. Two
deliberate breaks were caught: with continuation disabled, 4 checks fail;
with `skip_wren` set but CR1 left at 00, the model flags writes without
WREN.

### Rev. 7 – streaming writes across AXI transactions

The Identify capture `mram_wr.vcd` (hardware, rev. 5 bitstream) shows the
PCI bridge writing 32- or 64-byte INCR bursts (32-bit, len 7/15). Each one
arrives at the MRAM port as a single 64-byte beat. The bridge issues the
next AW about 17 aclk after B, although its FIFO held up to 191 words. Each
64 bytes took about 813 aclk (5.4 µs): data 512, WREN + CS# gap ≈ 128,
opcode/address ≈ 72, CS# high after the write 92. So the MRAM sat idle for
37 % of the time because of per-transaction overhead.

Changes (backend only; the wrapper and guard are unchanged):

* **Posted B.** bvalid one cycle after a write request is accepted. A
  read or register command still waits until the write's CS# rises.
  Output `mram_wr_pending` shows posted data not yet in the MRAM.
* **Append across transactions.** A contiguous write is appended to the
  running SPI write whether or not it belongs to the same burst.
* **Linger.** SCLK is stopped low and CS# held low for up to
  `G_WR_LINGER_CYCLES` waiting for the next contiguous write. This assumes
  the device accepts a paused SCLK with CS# low, which the reviewed
  datasheet pages do not forbid but do not state either.

Testbench T16/T16b check:
* six back-to-back single-beat transactions → 1 4WQIO and 1 WREN;
* a write after CS# rose → a new 4WQIO;
* a write arriving during the linger → resumes the same 4WQIO.

Switching off each of the three generics makes T16 or T16b fail. Measured
in simulation: ≈ 18 MB/s, against ≈ 11.8 MB/s in the capture.

### Rev. 8 – 64-bit data path, read streaming

* `C_AXI_DATA_WIDTH` (mram_pkg) is now a design constant, default 64:
  * the backend shift register, write-data and read-data alignment, the
    wrapper's strobe handling and the AxSIZE limit all follow from it;
  * 128/256/512 still work.

  Rough register count (GHDL synthesis, not Libero), before TMR:
  * about 1,850 bits at 64 bits;
  * about 5,900 bits for the previous 512-bit build.
* Read streaming:
  * **Wrapper:** the read engine was rewritten. An issue engine sends beat
    reads ahead of the R channel, bounded by a 4-beat read-data buffer.
    Error bursts (WRAP/FIXED/oversize) still return SLVERR beats in order.
  * **Backend:** contiguous reads continue the running RDQI. Per-request
    data is delivered from the capture side through a small in-order queue
    of read sizes. Linger applies to reads as to writes.
* Boot copy: it moves `G_BLOCK_BYTES` (64) blocks as back-to-back
  beat-sized requests. Each block is one streamed read and one streamed
  write, so a boot takes about as long as before at any width. The verify
  pass compares each destination beat with the buffered source block.
* Testbench:
  * now width-generic: 64-byte transfers are full-width bursts;
  * new T17: six back-to-back read bursts make 1 RDQI, and one 256-byte
    burst makes 1 RDQI. Turning off `G_STREAM_READS` or
    `G_RD_LINGER_CYCLES` makes it fail;
  * passes at 64, 128 and 512 bits and with sample delays 1–3;
  * spot checks of the `rd_sample_dly` table still match.

### Rev. 9 – read capture slip on board 2

Symptom (Identify, rev. 8 bitstream): every 64-bit read returned one junk
byte (00) and then the data shifted up one lane. The MRAM bus itself was
correct: the 16 nibbles of the 8 bytes fill exactly the data phase, and
when SCLK stops the device already shows the next byte. The boot copy
failed (`s_fail`, 3 retries).

Cause: rev. 8 delivers each read request after counting its nibbles
(`cap_cnt`). The count was off by 2 (one byte), and nothing ever reset it,
so every later read stayed shifted. The 512-bit design was immune because
it took the last nibbles of each transaction. Injecting two spurious
capture strobes in simulation reproduces the board symptom exactly. What
caused the miscount on the board is not known. The bitstream had Identify
and no timing effort, so a timing failure is a plausible candidate, but
simulation never shows it.

Fix:
* The read bookkeeping (queue and `cap_cnt`) restarts with the first read
  of every RDQI transaction, so a miscount cannot outlive one transaction.
* New sticky output `mram_rd_slip` reports any transaction whose count did
  not come out even.

With the same injection, the fixed backend returns correct data and sets
`mram_rd_slip`. The testbench fails if `mram_rd_slip` is ever set.

### Rev. 10 – 512-bit board test: wrong read data inside the FPGA

* With the 64-bit interconnect port, PCI writes never reached the IP. The
  interconnect accepted them from the bridge and never forwarded them.
  With the port back at 512 bits, the same write works end to end. The
  64-bit interconnect configuration is the suspect, not the IP.
* At 512 bits, the read of address 0 was correct on the MRAM pins
  (EBh, address 0, data 12 34 56 78 93 81 81 E1 ...). The IP returned
  12 34 A6 A5 A5 81 81 A1 ...: a bit-level mix of the new data and the
  previous read, not a shift. That pattern points to a timing failure in
  the wide read-data path. The build had Identify and no timing effort,
  so this is likely but not proven.
* Change: the read-data alignment (byte reverse plus a shift by
  NB - nbytes - offset bytes) is now three register stages: amount, coarse
  shift by 8-byte groups, fine shift by 0..7 bytes. The read buffer in the
  wrapper is kept in flip-flops (syn_ramstyle = "registers"). This adds
  two aclk of read latency. Regression passes at 64, 128 and 512 bits.

### Rev. 11 – reads starved by writes (fair arbitration)

On the board, a PCI read of 0x6000000 to a file hung while the firmware
was running. The new stress test `sim/tb_dma.vhd` (PCI-style DMA read
bursts, several outstanding, RREADY stalls, interleaved with CPU-like line
reads and a steady stream of 8-byte writes) showed why: the wrapper gave
writes strict priority over reads, so while writes kept arriving, a
waiting read never got the MRAM. With posted B, each write completes
quickly on AXI, so a CPU can send writes faster and the effect is worse.
4 KB of DMA reads took 9.3 ms instead of 0.22 ms, and a longer run did not
finish within 20 ms.

Fix: when both sides wait, the side served last keeps the MRAM, so streams
stay together, unless the other side has waited `G_ARB_MAX_WAIT` cycles
(default 256, ≈ 1.7 µs). The same load now takes 3.2 ms; the remaining
time is the MRAM itself serving the writes. All data, RID, RLAST and RRESP
checks pass; no hang. The main testbench passes at 64 and 512 bits and at
sample delays 1–3.

### Rev. 12 – SRAM preparation (branch `mram_sram`)

* `rtl/mram_top_oct1.vhd` removed: the Oct 1 diagnostic top is no longer
  needed. The run script and README no longer list it.
* AXI ID width 5 → 6 (`C_AXI_ID_WIDTH` in `mram_pkg`). With the third
  initiator (the SRAM boot copy, INITIATOR2) the interconnect's target ID
  is ID[5:0]; the SRAM IP was changed to 6 as well. A target narrower than
  the interconnect's ID drops the top bit, so a response can return to the
  wrong initiator and hang it. O15 below still says 5-bit; this supersedes
  it.
* Testbench: the burst IDs now set the top ID bit on every other burst, so
  all ID bits are checked on BID and RID. Forcing the top bit of BID/RID
  to 0 in the wrapper makes the testbench fail (454 RID and 22 BID
  mismatches); restored, it passes. `sim/run_dma.sh` passes with no
  errors.
* Architecture for the SRAM execution memory: `doc/SRAM_ARCH.md`.

### Rev. 13 – SRAM execution memory: boot copy MRAM → SRAM, reload

Architecture: `doc/SRAM_ARCH.md`. The CPU now executes from the 1 MB
on-chip SRAM (`PF_SRAM_AHBL_AXI`, interconnect TARGET1 at AXI 0x0); the
MRAM is TARGET0 from AXI 0x0010_0000.

| # | File | Change | Why |
|---|------|--------|-----|
| S1 | `rtl/sram_boot_copy.vhd` (new) | Copies 1 MB from MRAM 0x6000000 to the SRAM through its own AXI4 master port, verifies it by reading both again, retries up to 3 times, then releases `cpu_reset_n`. 64-byte INCR bursts, AWVALID only when the whole block is buffered, W beats without gaps, BREADY/RREADY always 1. Watchdog → stop issuing, complete every started handshake, retry; drain timeout → "stuck" (`boot_fail`). `boot_status` reports phase, retries and the cause of each failure. | The MRAM is too slow for execution. Lessons 1–3: never abandon a handshake, never depend on the other side's timing. |
| S2 | `rtl/mram_boot_copy.vhd` (deleted) | Replaced by S1. No legacy switch. | Requested. |
| S3 | `rtl/axi4_slave_wrapper.vhd` | New `hold` input: no new AW/AR accepted (READY low). New `idle` output: nothing accepted or in flight. | The backend changes hands for a reload without resetting the AXI side, so a PCI burst in flight always completes. Before, the wrapper was held in reset during the boot copy, which is only safe at power-up. |
| S4 | `rtl/mram_top.vhd` | New ports `m_axi_*` (INITIATOR2), `sram_reload` (synchronized, rising edge), `boot_status`. Ownership mux driven by the copy engine; backend responses are masked to the side that does not own it. New generics `G_BOOT_DST_ADDR`, `G_BOOT_FIFO_BLOCKS`, `G_M_ID_WIDTH`, `G_M_ID`; `G_BOOT_DST_BASE` removed; `G_BOOT_COPY_SIZE` default 1 MB. | S1, S3. |
| S5 | `rtl/mram_qspi_backend.vhd` | Comments only (references to the deleted file). | — |

Consequences:
* Releasing `boot_hold` with AXI accesses in flight is now safe (S3); the
  README caveat is removed.
* The old open items O7 (watchdog retry without waiting for the request
  in flight) and O8 (≈ 9 s PCI stall during boot) are addressed for the
  new engine: the drain waits for every response, and the stall is now
  about 0.12 s for 1 MB (estimate from simulation, not measured).
* O13 (copy only on `aresetn`): a reload can now be requested with
  `sram_reload`.
* The response mask adds one AND gate to the combinational accept path
  wrapper → mux → guard → backend → back (O11 a). Check it in SmartTime.
* Known limitation: if a PCI write burst was accepted by the wrapper and
  the bridge never sends the rest of its W data, the reload waits in phase
  2 ("waiting for the MRAM port") until it does. It cannot take the MRAM
  without breaking that burst.

Verification (GHDL 4.1, all at 64 bits unless noted):
* New `sim/tb_mram_sram.vhd` (`sim/run_sram.sh`), with a behavioural AXI4
  SRAM model (`sim/axi_sram_model.vhd`, random stalls and fault injection)
  and an AXI master protocol checker (`sim/axi_master_checker.vhd`) on
  `m_axi_*`. Tests T0–T9: `boot_hold`; boot with byte-for-byte scoreboard;
  MRAM access after boot; reload while PCI-style MRAM reads stream (all
  reads correct, no AW/AR accepted on the MRAM port while the copy owns
  it); one corrupted byte, one BRESP SLVERR, one RRESP SLVERR (one retry
  each); permanent corruption → `boot_fail`, then reload; all SRAM channels
  stalled past the watchdog → drain → retry; B never returned → stuck, no
  new traffic, MRAM handed back, `aresetn` recovers.
* Passes with SRAM stall rates 0 %, 30 %, 50 %, 80 % and 95 % (several
  seeds), and at 128 and 512 bits. With a 16 KB copy the boot takes
  1.749–1.752 ms of simulated time at every stall rate, so the MRAM stream
  is the bottleneck, not the SRAM side.
* Mutation checks, each made the testbench fail: verify compare removed;
  drain skipped; WLAST on the wrong beat; CPU released after the copy pass
  without verify; wrapper ignoring `hold` on AR (caught after adding a
  direct check — the first version only checked the outcome, which the
  response mask already kept correct); response mask removed.
* `sim/tb_mram_top.vhd` and `sim/tb_dma.vhd` now boot through the SRAM
  model; their MRAM-copy checks became SRAM checks. Both pass at 64, 128
  and 512 bits. `tb_dma` gives the same 3.2017 ms as before the change.
* The MRAM model's backdoor storage grew from 4 × 16 KB to 4 × 1 MB
  windows so a full 1 MB copy can be scoreboarded. Full-size run
  (`-gG_COPY=1048576 -gG_ONLY_BOOT=true`, T0–T2): passes; copy + verify of
  1 MB took 111.9 ms of simulated time (≈ 18.75 MB/s MRAM read rate).
* Rough register count (GHDL synthesis, not Libero, before TMR):
  `mram_top` ≈ 3,230 bits including the 1,024-bit copy FIFO and the
  260-bit read buffer, against ≈ 2,340 before; `sram_boot_copy` ≈ 1,600.

Limits: the SRAM model is generic AXI4, not `PF_SRAM_AHBL_AXI`, and there
is no interconnect model. Not synthesized in Libero, not timing-closed,
not run on hardware.

### Board checks (from the datasheet – cannot be fixed in RTL)

* **Recover a die left in XIP mode.** Before testing the new bitstream,
  **power-cycle the MRAM or pulse RESET#** (ball J9). XIP mode is not
  necessarily cleared by reloading the FPGA.
* **RESET# (J9)** must be high in normal operation. The datasheet gives no
  minimum pulse width in the pages reviewed.
* **Unused die:** the AS302G208 is two 1 Gb dies with separate CS1#/CLK1/IO[3:0]
  and CS2#/CLK2/IO[7:4]. This design uses die 1 only (128 MB, as in the spec).
  CS2# needs a pull-up (the datasheet recommends 10 kΩ on CS#).
* **CS1#:** 10 kΩ pull-up recommended so the die is deselected during power-up.
* **WP1#/IO2** has no internal pull-up and "cannot be left floating". IO3
  also needs a defined level. The design releases IO0–3 between
  transactions, so external pull-ups are required.
* **HBP0–2 / HTBSEL** hardware block protection. These have internal
  pull-downs, so no protection if unconnected. If the board straps them,
  writes to the protected range are silently ignored. For example, HBP =
  H-L-H with HTBSEL = L protects 0x6000000–0x7FFFFFF on a 1 Gb die.

## Open items (not fixed – need the datasheet or a system decision)

| # | Item |
|---|------|
| O1 | ~~Dummy cycles~~ – resolved by D6. CR2 is still never written, so it must stay at its default of 8. |
| O2 | ~~XIP mode bits~~ – resolved by D1. |
| O3 | ~~Power-up delay~~ – resolved by D4. It assumes `aresetn` is not released before the MRAM supply is up. |
| O4 | IO2/IO3 float between transactions and need board pull-ups (see "Board checks"). |
| O5 | I/O timing: see D7. Needs I/O constraints and timing analysis. `G_SCLK_HALF_PERIOD` = 1 (75 MHz) exceeds the device maximum of 54 MHz and must not be used. |
| O6 | Boot verify compares the copy with the source only. A corrupted master image is copied and "verified". Consider a CRC or signature over the image. |
| O7 | A watchdog-forced retry does not wait for a request still in flight in the backend. This only matters after a real fault; documented in `mram_boot_copy.vhd`. |
| O8 | During the ~9 s (up to ~35 s) boot copy, PCI accesses to this window stall. A PCIe completion timeout will fire long before that. The system must make PCI wait for `boot_done`, or the design needs a "respond SLVERR while booting" mode. |
| O9 | Protection is by address and `key_ok` only, not by master ID (spec open item). The spec says "only PCI can write the password register". That register is outside this design, so the claim cannot be checked here. |
| O10 | VHDL-2008 constructs used: records on ports, reading `out` ports, `when`/`else` inside processes. Check that the Libero synthesis version in use supports them. |
| O11 | Synthesis is not done. Watch two areas: (a) the combinational accept path wrapper → mux → guard address compare → backend → back, and (b) the 512-bit byte barrel shifters in the backend plus the strobe-run logic in the wrapper, which are the largest logic blocks. There are roughly 4–5k data flip-flops across the 512-bit registers; this is an estimate, not a synthesis result. |
| O12 | Safe-FSM / TMR settings are synthesis-tool options, not RTL (spec already says so). |
| O13 | The copy runs on `aresetn` only. A CPU-only reset does not refresh address 0. |
| O14 | Only 128 MB of the 2 Gb (256 MB) device is addressable. Confirm this is intended. |
| O15 | AXI port widths now follow the interconnect: 32-bit address, 5-bit ID, 512-bit data. Only the low 27 address bits (the offset in the 128 MB window) reach the MRAM. That is correct whether the interconnect passes the full system address or the window offset, provided the window base is 128 MB-aligned. |

## Specification errata

| Page | Statement | Issue |
|------|-----------|-------|
| 4–5 | Write FSM "INCR & full-width"; scope list "Full 512-bit (64-byte) beats only; a narrower AWSIZE/ARSIZE gets SLVERR" | Contradicts the RTL, the backend section (p6, narrow requests supported) and the workload (4-byte accesses). The RTL supports sizes 1–64 B. |
| 4 | Wrapper state diagrams | Outdated after the fixes. Write side: `WR_IDLE → WR_WDATA → WR_BEAT ⇄ WR_WAIT_BVALID → WR_RESP`, plus `WR_ERR_DRAIN → WR_RESP`. |
| 2, 3 | "Mux … the CPU issues no AXI traffic while held in reset" | The PCI master can issue traffic during the copy (C7). |
| 6 | "both mram_boot_copy and mram_qspi_backend force the bus released and CS# deasserted" on an illegal state | Only the backend owns the pins. The boot copy retries and drops its request. |
| 1, 6 | "108MHz" SDR / "~50-54MB/s" peak | This datasheet (Rev. J.5) gives 54 MHz SDR / 40 MHz DDR for the quad modes, so the raw peak is 27 MB/s per die. |
| 6 | Framing "(1-4-4) … address and data on all four IOs" | Omits the XIP mode byte between address and data (D1). |
| 7 | "a 4-byte request costs ~16 overhead cycles + 8 data cycles — about 0.24 µs at 100 MHz"; 64-byte "~1.44 µs" | Overhead is 8 opcode + 8 address + 2 XIP + 8 latency = **26** SCLK cycles for reads, 18 for writes plus a WREN transaction and 600 ns of CS# high time. That gives 32 cycles = 0.32 µs and 152 cycles = 1.52 µs at 100 MHz. The design cannot run at 100 MHz (O5). At the default 37.5 MHz, a 4-byte read takes ≈0.9 µs and a 64-byte read ≈4.1 µs. |
| 7 | "~32KB sequential … ~1 ms at ~40 MB/s", "consistent with the channel bandwidth" | Each 64-byte beat is a separate SPI transaction. The limit is ≈15 MB/s at 37.5 MHz (≈30 MB/s at 75 MHz if timing closes), so 32 KB takes ≈2.1 ms. 40 MB/s needs burst aggregation (not implemented). Hand calculation; confirm on hardware. |
| 2 | "gates writes to the protected region on a password match" | The hardware gates on `key_ok`; the password logic is external. |
| 1 | MRAM interface "CS#, SCLK, IO0-3, WP#" | WP# is IO2 in quad mode. It is not a separate pin in this RTL. |
| 2 | Design priority "Minimize miss latency" | There is no cache. Presumably this means access latency. |
| 7 | RTPF500T resources (~1,520 LSRAM, ~4,440 uSRAM, ~481K LE) | Not verified here. Check against the Microchip datasheet. |
| 8 | "No VHDL toolchain … none of it has been compiled or simulated" | Now compiled and simulated with GHDL (functional only). |
