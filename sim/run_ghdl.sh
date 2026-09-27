#!/usr/bin/env bash
# Compile and run the self-checking testbench with GHDL (tested with 4.1).
# Usage: sim/run_ghdl.sh [--wave]   (--wave writes build/tb_mram_top.ghw)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ROOT/sim/build"
mkdir -p "$WORK"
cd "$WORK"
FLAGS="--std=08 --workdir=$WORK"
for f in rtl/mram_pkg rtl/mram_write_guard rtl/mram_qspi_backend rtl/mram_boot_copy \
         rtl/axi4_slave_wrapper rtl/mram_top sim/qspi_mram_model sim/tb_mram_top; do
    ghdl -a $FLAGS "$ROOT/$f.vhd"
done
ghdl -e $FLAGS tb_mram_top
RUNOPTS=""
if [[ "${1:-}" == "--wave" ]]; then RUNOPTS="--wave=$WORK/tb_mram_top.ghw"; fi
ghdl -r $FLAGS tb_mram_top $RUNOPTS
