#!/usr/bin/env bash
# Compile and run the self-checking testbench with GHDL (tested with 4.1).
# Usage: sim/run_ghdl.sh [--wave] [-gG_MODEL_TCO_PS=20000 -gG_SAMPLE_DLY=2 ...]
#   --wave writes build/tb_mram_top.ghw; -g options override testbench generics
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ROOT/sim/build"
mkdir -p "$WORK"
cd "$WORK"
FLAGS="--std=08 --workdir=$WORK"
for f in rtl/mram_pkg rtl/mram_write_guard rtl/mram_cmd_ctrl rtl/mram_qspi_backend rtl/mram_boot_copy \
         rtl/axi4_slave_wrapper rtl/mram_top sim/qspi_mram_model sim/tb_mram_top; do
    ghdl -a $FLAGS "$ROOT/$f.vhd"
done
ghdl -e $FLAGS tb_mram_top
RUNOPTS=()
for a in "$@"; do
    if [[ "$a" == "--wave" ]]; then RUNOPTS+=("--wave=$WORK/tb_mram_top.ghw"); else RUNOPTS+=("$a"); fi
done
ghdl -r $FLAGS tb_mram_top "${RUNOPTS[@]}"
