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
| `rtl/mram_cmd_ctrl.vhd` | PCI-driven MRAM register commands (WREN, WRDI, RDSR, WRSR, RDID) with clock-domain crossing |
| `rtl/mram_write_guard.vhd` | Blocks writes to the protected region unless `key_ok` |
| `rtl/mram_qspi_backend.vhd` | Quad-SPI master (WREN / EBh read / D2h write, 1-4-4 SDR) |
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
| `cmd_wrsr_data[7:0]` | in | Status register value for WRSR; set it with or before the request and keep it stable while the request is high |
| `cmd_rdsr_data[7:0]`, `cmd_rdid_data[31:0]` | out | Results; valid once `cmd_done` = 1, held until the same command runs again |
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

Status register (datasheet Table 16):

| Bit(s) | Field |
|--------|-------|
| 7 | WP#EN |
| 5 | TBPSEL |
| 4:2 | BPSEL[2:0] — any non-zero value write-protects part of the array, and writes there are silently ignored |
| 1 | WREN |

## Simulation

```sh
sim/run_ghdl.sh          # GHDL >= 4.x, VHDL-2008; prints TEST PASSED / FAILED
sim/run_ghdl.sh --wave   # also writes sim/build/tb_mram_top.ghw
```

## Status

Functionally simulated against a behavioural model written from the
AS302G208 datasheet (Rev. J.5): instruction framing including the XIP mode
byte, CS# high times, power-up time, and the 54 MHz clock limit. Not yet
timing-closed or re-verified on hardware after the rev. 2 fixes. Before
testing a new bitstream, power-cycle the MRAM or pulse RESET#. See
`doc/REVIEW.md`, "Rev. 2" and "Board checks".
