# Libero integration notes – mram_top with the SRAM boot copy

Branch `mram_sram`. Architecture: `doc/SRAM_ARCH.md`. Verification record:
`doc/REVIEW.md` Rev. 13–15. Short pre-run list: `doc/PRECOMPILE_CHECKLIST.md`.

Where these notes depend on Microchip IP or tool behaviour, they say what
is needed and where to check it. I do not name configurator options, IP
ports or tool commands I have not verified; such places are marked
**(check in Libero)**. Every number marked *estimate* is a calculation or
a simulation, not a measurement on this board.

## 1. Files

| File | Change |
|------|--------|
| `rtl/mram_pkg.vhd` | `C_AXI_ID_WIDTH` = 6 (was 5) |
| `rtl/sram_boot_copy.vhd` | **new** |
| `rtl/axi4_slave_wrapper.vhd` | `hold` input, `idle` output |
| `rtl/mram_top.vhd` | new ports, generics, boot copy replaced |
| `rtl/mram_qspi_backend.vhd`, `rtl/mram_write_guard.vhd`, `rtl/mram_cmd_ctrl.vhd` | unchanged (backend: comments only) |
| `rtl/mram_boot_copy.vhd`, `rtl/mram_top_oct1.vhd` | **deleted**: remove them from the project |
| `constraints/mram_top.sdc` | **new** template (section 5) |

All RTL files are VHDL-2008, library `work`, `mram_pkg.vhd` first. The
testbench and formal files (`sim/`, `formal/`) are not for synthesis.

## 2. Port list and connections

Clock domain "aclk" means synchronous to the 150 MHz `aclk`.

### Clock and reset

| Port | Dir | Width | Connect to |
|------|-----|-------|------------|
| `aclk` | in | 1 | 150 MHz fabric clock, the same clock as interconnect TARGET0 and INITIATOR2 |
| `aresetn` | in | 1 | System reset, active low, synchronous release in `aclk`. **Must also reset the interconnect's TARGET0 and INITIATOR2 ports** (a reset in the middle of a copy drops `m_axi` VALIDs, which AXI only allows if both sides are reset). Not gated by `cpu_reset_n` |

### AXI4 slave (TARGET0, MRAM) – aclk

| Ports | Width | Note |
|-------|-------|------|
| `s_axi_aw{id,addr,len,size,burst,valid,ready}` | id 6, addr 32, len 8, size 3, burst 2 | |
| `s_axi_w{data,strb,last,valid,ready}` | data 64, strb 8 | |
| `s_axi_b{id,resp,valid,ready}` | id 6, resp 2 | |
| `s_axi_ar{id,addr,len,size,burst,valid,ready}` | as AW | |
| `s_axi_r{id,data,resp,last,valid,ready}` | id 6, data 64 | |

The ID width changed from 5 to 6 bits (TARGET0 ID[5:0]): reconnect all six
bits. `mram_top` has no AxLOCK, AxCACHE, AxPROT, AxQOS, AxREGION or user
ports; leave those TARGET0 outputs unconnected. **(check in Libero** that
SmartDesign accepts unconnected outputs of the interconnect.)

### AXI4 master (INITIATOR2, boot copy → SRAM) – aclk, new

| Ports | Width | Driven value / note |
|-------|-------|---------------------|
| `m_axi_aw{id,addr,len,size,burst,lock,cache,prot,valid,ready}` | id 4, addr 32, len 8 | id = `G_M_ID` (0), len = 7, size = 3 (8 bytes), burst = INCR, lock 0, cache 0000, prot 000 |
| `m_axi_w{data,strb,last,valid,ready}` | data 64, strb 8 | strb all ones |
| `m_axi_b{id,resp,valid,ready}` | id 4 | `bready` always 1 |
| `m_axi_ar{id,addr,len,size,burst,lock,cache,prot,valid,ready}` | as AW | |
| `m_axi_r{id,data,resp,last,valid,ready}` | id 4, data 64 | `rready` always 1 |

Bursts: 64 bytes, never crossing 4 KB, at most 2 write and 2 read bursts
outstanding, only between reset/reload and `boot_done`/`boot_fail`; idle
otherwise. INITIATOR2 inputs that `mram_top` does not drive (QoS, region,
user, if the configurator shows them) must be tied to their inactive value
**(check in Libero)**.

### Control and status

| Port | Dir | Width | Domain | Connect to |
|------|-----|-------|--------|------------|
| `key_ok` | in | 1 | async (synchronized) | Unlock for writes to the master copy (existing) |
| `cmd_wren` … `cmd_wrar` | in | 1 each | async (synchronized) | PCI register commands (existing) |
| `cmd_wrsr_data`, `cmd_reg_addr` | in | 8 | handshake | PCI registers (existing) |
| `cmd_rdsr_data`, `cmd_rdid_data`, `cmd_done` | out | 8, 32, 1 | aclk | PCI registers; `cmd_done` synchronized on the PCI side (existing) |
| `boot_hold` | in | 1 | async (synchronized) | PCI register; 1 at reset for bring-up, 0 for normal boot |
| `sram_reload` | in | 1 | async (synchronized, rising edge) | **new**: PCI register bit (or other source), or tie to 0 |
| `rd_sample_dly` | in | 3 | quasi-static | PCI register, default "010" (existing) |
| `skip_wren` | in | 1 | quasi-static | PCI register or 0 (existing) |
| `mram_wr_pending`, `mram_rd_slip` | out | 1 | aclk | status (existing) |
| `cpu_reset_n` | out | 1 | aclk | NOEL-V reset, **through a reset synchronizer in the CPU clock domain** (open item OA6) |
| `boot_done`, `boot_fail` | out | 1 | aclk | PCI status / fault indicator |
| `boot_status` | out | 16 | aclk | **new**: PCI status register and Identify. Bit layout in section 7 |
| `mram_cs_n`, `mram_sclk`, `mram_io[3:0]` | out/inout | 1, 1, 4 | aclk | MRAM pins, die 1 (existing) |

## 3. Generics for the board build

All defaults are the board values; set none of them in SmartDesign unless
the board differs.

| Generic | Default | Meaning |
|---------|---------|---------|
| `G_ID_WIDTH` | 6 (`C_AXI_ID_WIDTH`) | TARGET0 ID width |
| `G_BOOT_SRC_BASE` | 0x6000000 | MRAM offset of the master firmware |
| `G_BOOT_DST_ADDR` | 0 | AXI address of the SRAM |
| `G_BOOT_COPY_SIZE` | 1048576 | Bytes copied |
| `G_BOOT_FIFO_BLOCKS` | 2 | Copy FIFO depth in 64-byte blocks |
| `G_BOOT_MAX_RETRIES` | 3 | Retries before `boot_fail` |
| `G_BOOT_WATCHDOG` | 1,000,000 | Cycles without progress before the drain (6.7 ms); the drain gets as long again before "stuck" |
| `G_M_ID_WIDTH` | 4 | INITIATOR2 ID width |
| `G_M_ID` | 0 | AXI ID used by the boot copy |
| `G_POWERUP_CYCLES` | 3,750,000 | MRAM tPU, 25 ms |
| others | unchanged | MRAM backend, write guard (see README) |

The testbenches use much smaller values for `G_POWERUP_CYCLES`,
`G_BOOT_COPY_SIZE` and `G_BOOT_WATCHDOG`; make sure none of them reached the
SmartDesign instance.

## 4. Interconnect and SRAM IP

What the design needs; the configurator names are yours to map.
**(check in Libero)** for every row.

| Item | Required |
|------|----------|
| Initiators | 3: INITIATOR0 PCI bridge, INITIATOR1 NOEL-V, INITIATOR2 boot copy (new) |
| INITIATOR2 | AXI4, 64-bit data, ID 4 bits, clocked by `aclk` (no CDC needed), may reach TARGET1 (SRAM). It needs no access to TARGET0 |
| TARGET0 (MRAM) | 0x0010_0000–0x07FF_FFFF, 64-bit, ID 6 bits, `aclk`. The full AXI address must reach `mram_top` (not the address minus the base), open item OA1 |
| TARGET1 (SRAM) | 0x0000_0000–0x000F_FFFF, 64-bit, ID 6 bits, `aclk` |
| `PF_SRAM_AHBL_AXI` | LSRAM, 131072 × 64 bit, AXI, 64-bit, ID width 6, address 32. Open: ECC (currently Disabled), meaning of the write/read timeout counts (80) and of "High reliability" (open item OA2) |

If the configurator offers per-initiator target visibility, restrict
INITIATOR2 to TARGET1: the boot copy never addresses anything else, and a
fault in it then cannot reach the MRAM through the fabric.

## 5. Timing constraints (`constraints/mram_top.sdc`)

The file is a template with placeholders (`<MRAM>`, `<ACLK>`, `<PCI_CLK>`,
`<CPU_RST_SYNC>`); its header says how to fill and check it. In short:

1. **PCI clock ↔ aclk:** false path both ways. Every crossing into
   `mram_top` goes through a 2-flop synchronizer (`u_cmd_ctrl/req_meta`,
   `u_write_guard/key_ok_meta`, `u_backend/dly_q1`, `u_backend/skip_q1`,
   `reload_q1`) or is sampled only under the command handshake
   (`cmd_wrsr_data`, `cmd_reg_addr`). Outputs read on the PCI side are
   synchronized there (`cmd_done`) or read under the handshake / as
   status.
2. **cpu_reset_n → CPU domain:** false path to the first stage of the CPU
   reset synchronizer only. Do not cut aclk ↔ CPU clock as a whole: the
   interconnect's CDC sits there and needs the vendor's constraints.
3. **MRAM pins:** max-delay budgets (7 ns, *proposed*) on the register →
   pad and pad → register paths so that SCLK, CS# and IO have short and
   similar pad delays. Read timing is tuned on the board with
   `rd_sample_dly`. Whether to pack these registers into I/O registers is
   a synthesis choice **(check in the Synplify/Libero documentation for
   your version)**; it would make the delays more uniform.
4. Everything else, including the new `m_axi_*` port, is synchronous to
   `aclk` and needs no exception.

Paths to look at first in SmartTime (all in `aclk`):
* The combinational accept path: request register (`u_axi_wrapper` or
  `u_boot_copy`) → ownership mux → `u_write_guard` address compare →
  `u_backend` ready → ownership mask → register enables (REVIEW O11 a).
* `u_boot_copy` FIFO read (16:1 × 64 bit) into `m_axi_wdata` and `cmp_e`.
* `u_axi_wrapper` `idle` (wide AND) into `u_boot_copy`.
* The interconnect itself on the new INITIATOR2 → TARGET1 path.

## 6. Expected resources and throughput (estimates)

| Item | Figure | Basis |
|------|--------|-------|
| Registers, `mram_top` | ≈ 3,230 bits (≈ 2,340 before), before TMR | GHDL synthesis count, not Synplify |
| of which `sram_boot_copy` | ≈ 1,600 bits, including the 1,024-bit FIFO | same |
| LSRAM for the SRAM | from the Libero report (already in the build) | 8 Mbit / 20 Kbit ≈ 410 blocks without ECC overhead, as a lower bound |
| Boot after reset | ≈ 0.14 s: 25 ms power-up + 112 ms copy and verify | full 1 MB simulation (111.9 ms), MRAM read ≈ 18.75 MB/s |
| Reload | ≈ 0.11 s | same, without the power-up wait |
| Worst case, 3 retries | ≈ 0.47 s | 4 × 112 ms + 25 ms |
| MRAM access stall during a copy (PCI, CPU) | up to ≈ 0.11 s | the copy owns the MRAM |
| CPU ↔ SRAM | ≤ 400 MB/s peak | 64 bit × 50 MHz at the CPU port; measure |
| PCI ↔ SRAM | limited by PCI (≈ 70 MB/s seen at the bridge, REVIEW Rev. 6) | |
| PCI/CPU ↔ MRAM | unchanged, ≈ 18 MB/s streamed | |

TMR multiplies the register count by about 3 plus voters (*estimate*).

## 7. Identify probe list

Probe registered signals only and keep the list short: earlier board
failures (REVIEW Rev. 9, 10) appeared in builds with Identify and no
timing effort. Re-run timing with the instrumented design. Names are the
VHDL names under the `mram_top` instance (`<MRAM>`); Identify shows them
by RTL hierarchy.

| Group | Signals | Why |
|-------|---------|-----|
| Status | `<MRAM>/boot_status[15:0]` (or `u_boot_copy/state`), `boot_done`, `boot_fail`, `cpu_reset_n` | Phase, retries, failure cause |
| Hand-over | `<MRAM>/boot_own`, `boot_hold_axi`, `axi_idle`, `reload_pulse` | Who owns the MRAM; a reload waiting in phase 2 shows `axi_idle` = 0 |
| SRAM AXI | `m_axi_awvalid`, `m_axi_awready`, `m_axi_awaddr[19:6]`, `m_axi_wvalid`, `m_axi_wready`, `m_axi_wlast`, `m_axi_bvalid`, `m_axi_bresp`, `m_axi_bid`, `m_axi_arvalid`, `m_axi_arready`, `m_axi_rvalid`, `m_axi_rresp`, `m_axi_rlast`, `m_axi_rid` | Handshakes, responses |
| Verify | `u_boot_copy/mis_v`, `u_boot_copy/mis`, `u_boot_copy/cmp_d[63:0]`, `u_boot_copy/cmp_e[63:0]`, `m_axi_araddr[19:6]` | The data of a mismatch |
| MRAM side | `u_boot_copy/req.valid`, `guard_resp.ready`, `guard_resp.rvalid`, `mram_cs_n`, `mram_sclk`, `mram_io[3:0]`, `mram_rd_slip` | The MRAM read stream |
| PCI side | `s_axi_awvalid`, `s_axi_awready`, `s_axi_arvalid`, `s_axi_arready`, `s_axi_bvalid`, `s_axi_rvalid`, `u_axi_wrapper/wr_state` | Stalls during a copy |

Triggers:
1. `boot_status[2:0]` changes to 5 (drain/retry), 6 (failed) or 7 (stuck).
2. `u_boot_copy/mis_v` = 1 and `u_boot_copy/mis` = 1: a verify mismatch;
   `cmp_d` is the SRAM data, `cmp_e` the MRAM data.
3. `m_axi_bvalid` = 1 and `m_axi_bresp` ≠ 00, or `m_axi_rvalid` = 1 and
   `m_axi_rresp` ≠ 00.
4. `reload_pulse` = 1, to watch a whole reload hand-over.

A whole copy is about 112 ms, far longer than any capture buffer; capture
around the triggers.

`boot_status` bits: [2:0] phase (0 done, 1 power-up / held, 2 waiting for
the MRAM port, 3 copy, 4 verify, 5 drain / retry, 6 failed, 7 stuck),
[7:4] retries, [8] mismatch, [9] BRESP error, [10] RRESP error, [11] MRAM
error, [12] watchdog, [13] drain timeout (stuck), [14] wrong ID, RLAST or
unexpected response. Bits 8–14 are sticky from the start of a boot or
reload.

## 8. Bring-up order on the board

See `doc/PRECOMPILE_CHECKLIST.md`, section 5: boot with `boot_hold` = 1,
the full-address test (OA1), release, check `boot_status` = 0x0000 and the
SRAM contents, then a reload.

## 9. Open items affecting integration

| # | Item |
|---|------|
| OA1 | Interconnect passes the full address to TARGET0 (board test) |
| OA2 | SRAM IP: ECC on or off; timeout counts; "High reliability" |
| OA3 | INITIATOR2: target visibility; AxCACHE requirements of the interconnect |
| OA6 | Reset synchronizer for `cpu_reset_n` in the CPU clock domain |
| — | `sram_reload` source |
| — | AS302G208: an MRAM command cut by `aresetn` (CS# high mid-command) is harmless (datasheet check, REVIEW Rev. 14) |
