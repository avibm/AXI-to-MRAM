# MRAM Subsystem – Design Review (rev. 2)

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
| D7 | Output valid tCO ≤ 9 ns after the falling edge (Table 43). Sampling half a period later leaves ~4 ns at 37.5 MHz for FPGA and board delays | Documented. Use `G_SCLK_HALF_PERIOD` = 3 (25 MHz) if the I/O timing does not close. The model now uses tCO = 9 ns. |

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
| O15 | "Same interface as PF_SRAM_AHB_AXI" (512-bit data, 27-bit address). I could not verify that Microchip core's configurable widths. Check against its handbook. |

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
