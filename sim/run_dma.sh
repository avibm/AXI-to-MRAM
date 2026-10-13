#!/usr/bin/env bash
# Mixed-traffic stress test (PCI DMA reads + CPU-like reads/writes), GHDL 4.x.
# Usage: sim/run_dma.sh [-gG_SZ=2 -gG_LEN=15 -gG_NBURST=64 -gG_SEED=7 -gG_CPU=true ...]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ROOT/sim/build_dma"
mkdir -p "$WORK"
cd "$WORK"
FLAGS="--std=08 --workdir=$WORK"
for f in rtl/mram_pkg rtl/mram_write_guard rtl/mram_cmd_ctrl rtl/mram_qspi_backend rtl/mram_boot_copy \
         rtl/axi4_slave_wrapper rtl/mram_top sim/qspi_mram_model sim/tb_dma; do
    ghdl -a $FLAGS "$ROOT/$f.vhd"
done
ghdl -e $FLAGS tb_dma
ghdl -r $FLAGS tb_dma "$@"
