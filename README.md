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
| `rtl/mram_write_guard.vhd` | Blocks writes to the protected region unless `key_ok` |
| `rtl/mram_qspi_backend.vhd` | Quad-SPI master (WREN / EBh read / D2h write, 1-4-4 SDR) |
| `rtl/mram_top.vhd` | Top level |
| `sim/` | Behavioural QSPI MRAM model, self-checking testbench, GHDL run script |
| `doc/MRAM_Subsystem_IP_Specification.pdf` | IP specification |
| `doc/REVIEW.md` | Design review: bugs found and fixed, open items, spec errata |

## Simulation

```sh
sim/run_ghdl.sh          # GHDL >= 4.x, VHDL-2008; prints TEST PASSED / FAILED
sim/run_ghdl.sh --wave   # also writes sim/build/tb_mram_top.ghw
```

## Status

Functionally simulated only. The design has not been synthesized or
timing-closed, and it has not been checked against the AS302G208 datasheet
(dummy cycles/CR2, mode bits, power-up timing). See `doc/REVIEW.md`, "Open
items".
