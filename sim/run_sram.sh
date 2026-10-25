#!/usr/bin/env bash
# Compile and run the SRAM boot-copy testbench with GHDL (tested with 4.1).
# Usage: sim/run_sram.sh [--wave] [-gG_STALL_PCT=0 -gG_SEED=3 -gG_COPY=16384 ...]
#   --wave writes build/tb_mram_sram.ghw; -g options override testbench generics
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ROOT/sim/build"
mkdir -p "$WORK"
cd "$WORK"
FLAGS="--std=08 --workdir=$WORK"
for f in rtl/mram_pkg rtl/mram_write_guard rtl/mram_cmd_ctrl rtl/mram_qspi_backend rtl/sram_boot_copy \
         rtl/axi4_slave_wrapper rtl/mram_top sim/qspi_mram_model sim/axi_sram_model \
         sim/axi_master_checker sim/tb_mram_sram; do
    ghdl -a $FLAGS "$ROOT/$f.vhd"
done
ghdl -e $FLAGS tb_mram_sram
RUNOPTS=()
for a in "$@"; do
    if [[ "$a" == "--wave" ]]; then RUNOPTS+=("--wave=$WORK/tb_mram_sram.ghw"); else RUNOPTS+=("$a"); fi
done
ghdl -r $FLAGS tb_mram_sram "${RUNOPTS[@]}"
