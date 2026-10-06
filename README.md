# AXI-to-MRAM

AXI4 slave front end for the Avalanche AS302G208 Quad-SPI MRAM on RT PolarFire
(RTPF500TCG1509), shared by the NOEL-V CPU and a PCI master through the AXI
interconnect. Includes a boot-copy sequencer, which refreshes the working boot
image at address 0 from a protected master copy at 0x6000000, and a write
guard for the protected region.

| Path | Contents |
|------|----------|
| `rtl/mram_pkg.vhd` | Shared constants and the internal `core_req` / `core_resp` interface |
| `rtl/axi4_slave_wrapper.vhd` | AXI4 slave: bursts → single core requests (INCR, 1–64 B beats, WSTRB) |
| `rtl/mram_boot_copy.vhd` | Copies and verifies the boot image, holds the CPU in reset until done |
| `rtl/mram_cmd_ctrl.vhd` | PCI-driven MRAM register commands (WREN, WRDI, RDSR, WRSR, RDID, RDAR, WRAR) with clock-domain crossing |
| `rtl/mram_write_guard.vhd` | Blocks writes to the protected region unless `key_ok` |
| `rtl/mram_qspi_backend.vhd` | Quad-SPI master (WREN / EBh read / D2h write, 1-4-4 SDR; contiguous writes merged) |
| `rtl/mram_top.vhd` | Top level |
| `sim/` | Behavioural QSPI MRAM model, self-checking testbench, GHDL run script |
| `doc/MRAM_Subsystem_IP_Specification.pdf` | IP specification |
| `doc/REVIEW.md` | Design review: bugs found and fixed, open items, spec errata |

## MRAM register commands (bring-up / debug)

Ports on `mram_top`, driven from PCI registers (the requests, `cmd_wrsr_data`
and `boot_hold` are synchronized into `aclk` inside):

| Port | Dir | Meaning |
|------|-----|---------|
| `cmd_wren`, `cmd_wrdi`, `cmd_rdsr`, `cmd_wrsr`, `cmd_rdid` | in | One request per command (06h, 04h, 05h, 01h, 9Fh) |
| `cmd_rdar`, `cmd_wrar` | in | Read / Write Any Register (65h, 71h) |
| `cmd_reg_addr[7:0]` | in | Register address for RDAR/WRAR (Table 13: SR 00h, CR1 02h, CR2 03h). Same rules as `cmd_wrsr_data` |
| `cmd_wrsr_data[7:0]` | in | Data for WRSR and WRAR; set it with or before the request and keep it stable while the request is high |
| `cmd_rdsr_data[7:0]`, `cmd_rdid_data[31:0]` | out | Results (`cmd_rdsr_data` also holds the RDAR result); valid once `cmd_done` = 1, held until the next RDSR/RDAR (or RDID) |
| `skip_wren` | in | 1 = array writes are sent without a WREN first. Only for CR1 WRENS = 01, see below. Quasi-static; tie to 0 if unused |
| `cmd_done` | out | `aclk` register, not synchronized: the PCI side must synchronize it |
| `boot_hold` | in | 1 = the boot copy waits after the 25 ms power-up time, so the device can be inspected first. AXI/PCI access to the MRAM window is open while held |

Handshake for each command:
1. Raise one request and hold it.
2. Wait for (the synchronized) `cmd_done` = 1, then read the result.
3. Lower the request.
4. Wait for `cmd_done` = 0 before the next command.

AXI access to the MRAM window is open after `boot_done`, after `boot_fail`
(the CPU stays in reset), and while `boot_hold` = 1. Release `boot_hold` only
when no AXI access to the MRAM is in flight; the boot copy then starts and
AXI stalls until it finishes. `boot_hold` is sampled when the 25 ms power-up
wait ends, so for bring-up its source should already be 1 at reset.

One command runs per request, and WRSR does not send WREN itself. Commands
wait for the 25 ms power-up time and run between memory accesses.

**Caution:** every AXI or boot-copy write sends its own WREN, and the device
clears the WREN bit when a write completes. A PCI WREN → WRSR sequence
therefore only works while no memory writes are running: for example with
`boot_hold` = 1, or with AXI writers idle. Check with RDSR afterwards.

### Write speed: streaming, merged writes and WREN-once mode

**Streaming across AXI transactions (default on).** Contiguous writes go
to the MRAM as **one** long D2h write, even when each arrives as a separate
AXI transaction. On the board, the PCI bridge sends one 64-byte write at a
time and waits for B before the next AW, so before this every 64 bytes paid
for WREN, opcode, address and two 600 ns CS# high times. Three parts make
it work:

* **Posted B** (`G_POSTED_WRITES`): B is returned as soon as the write data
  is held in the backend, not when it reaches the MRAM. The bridge then
  sends the next write while the current one is still being clocked out.
  Order is kept: a later read or register command starts only after the
  write has finished. The new output `mram_wr_pending` is 1 while posted
  data has not reached the MRAM yet. Check it is 0 before powering down or
  resetting the MRAM. It can be left unconnected.
* **Append** (`G_STREAM_WRITES`): a write starting at the next byte after
  the one being clocked out continues the same SPI write.
* **Linger** (`G_WR_LINGER_CYCLES`, default 256 = 1.7 µs): if the next write
  is late, SCLK stops low and CS# stays low for up to that long. A read, a
  register command or a non-contiguous write ends it at once.
  **Assumption:** the datasheet pages I reviewed give only minimum SCLK
  high/low times and no maximum CS# low time, so I assume pausing SCLK
  inside a write is allowed. If in doubt, set the generic to 0. Streaming
  still works whenever the next write arrives before the current one has
  finished.

Measured in simulation, back-to-back 64-byte single-beat writes: 384 bytes
in 21.3 µs, ≈ 18 MB/s. The Identify capture of the current hardware
(`mram_wr.vcd`) shows ≈ 5.4 µs per 64 bytes, ≈ 11.8 MB/s. The ceiling at
37.5 MHz SCLK is 18.75 MB/s (2 SCLK per byte); going higher needs a
faster SCLK, i.e. an aclk other than 150 MHz, since 150 / 4 = 37.5 MHz and
150 / 2 = 75 MHz is above the 54 MHz limit.

**Merged writes within a burst.** The contiguous strobe runs of one AXI
write burst also go out as one D2h write. For example, a 64-byte burst at a
0x20 offset arrives as 2 beats with 32 strobes each and becomes one 64-byte
4WQIO. A gap in the strobes or a FIXED burst ends the MRAM write.

**WREN-once mode** saves the WREN instruction and the 600 ns CS# high
time after it on every write. Datasheet Table 23: CR1[1:0] WRENS = 01
("SRAM") means WREN is not needed for array writes. Register writes still
always need WREN. Sequence, with AXI writes idle:

1. RDAR 02h → current CR1 (default 60h: ODSEL = 011).
2. WREN, then WRAR 02h with `(CR1 & FCh) | 01h`, which keeps ODSEL/MAPLK.
3. RDAR 02h → check that it reads back with WRENS = 01.
4. Set `skip_wren` = 1.

To undo: set `skip_wren` = 0 first, then WREN plus WRAR 02h with WRENS = 00.

Notes:
* `skip_wren` must be 0 whenever CR1 might be at WRENS = 00, i.e. after
  power-up/reset: the boot copy runs before PCI can set CR1. With
  `skip_wren` = 1 and WRENS = 00, every write is **silently dropped** by
  the device.
* The datasheet pages I reviewed do not say whether CR1 is volatile, so
  re-check with RDAR after every power cycle.
* With WRENS = 01 the WREN bit no longer protects the array, so any stray
  write lands. The software block protection (BPSEL) and the write guard
  still apply.
* The RDAR latency is taken as `G_DUMMY_CYCLES` (8, the CR2 default). Table
  28 gives 8–15 cycles, and that the value follows CR2 is my reading of it,
  not something I could confirm in the datasheet text. If CR2 is changed,
  `G_DUMMY_CYCLES` must match.

With streaming, WREN-once saves only one WREN per stream, so it matters
much less than before.

Even 18 MB/s is well below what PCI can deliver: the capture shows the
bridge FIFO reaching 191 words while the MRAM was writing. **The PCI
bridge (`pci2ddr4`) still needs back-pressure** (target wait states or
retry when its FIFO is full). Without it, data is still lost on large
transfers, only later.

Status register (datasheet Table 16):

| Bit(s) | Field |
|--------|-------|
| 7 | WP#EN |
| 5 | TBPSEL |
| 4:2 | BPSEL[2:0] — any non-zero value write-protects part of the array, and writes there are silently ignored |
| 1 | WREN |

## Read sample delay (`rd_sample_dly`)

Read data from the MRAM is sampled `rd_sample_dly` `aclk` cycles after each
SCLK rising edge (0–7, default 2). On the first board, with 0, every bit was
sampled one SCLK late: RDID read `0x73109480` instead of `0xE6212901`,
because the board's SCLK → MRAM → FPGA delay exceeded half an SCLK period.

The input is quasi-static, for example from a PCI register. Change it only
while the MRAM is idle (`boot_hold` = 1, no access in flight).

To tune on hardware, run RDID for each value. The ID must read `E6212901`
for an AS302G208; pick the middle of the range of values that work.

Simulated with the default `G_SCLK_HALF_PERIOD` = 2 (37.5 MHz). "Delay" is
from SCLK falling at the FPGA to the data arriving back at the FPGA:

| delay | 0 | 1 | 2 | 3 |
|-------|---|---|---|---|
| 6 ns  | ✓ | ✓ | ✓ | ✗ |
| 12 ns | ✓ | ✓ | ✓ | ✓ |
| 16 ns | ✗ | ✓ | ✓ | ✓ |
| 20 ns | ✗ | ✓ | ✓ | ✓ |
| 24 ns | ✗ | ✗ | ✓ | ✓ |
| 28 ns | ✗ | ✗ | ✗ | ✗ |

## Simulation

```sh
sim/run_ghdl.sh          # GHDL >= 4.x, VHDL-2008; prints TEST PASSED / FAILED
sim/run_ghdl.sh --wave   # also writes sim/build/tb_mram_top.ghw
sim/run_ghdl.sh -gG_MODEL_TCO_PS=20000 -gG_SAMPLE_DLY=0   # slow board, no delay: fails like the first board
```

## Status

Functionally simulated against a behavioural model written from the
AS302G208 datasheet (Rev. J.5): instruction framing including the XIP mode
byte, CS# high times, power-up time, and the 54 MHz clock limit. Not yet
timing-closed or re-verified on hardware after the rev. 2 fixes. Before
testing a new bitstream, power-cycle the MRAM or pulse RESET#. See
`doc/REVIEW.md`, "Rev. 2" and "Board checks".
