#!/usr/bin/env bash
# Randomized soak test of the SRAM boot copy (sim/tb_sram_soak.vhd), GHDL 4.x.
# Usage: sim/run_soak.sh [-gG_SEED=5 -gG_ITER=60 -gG_STALL_PCT=50 ...]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$ROOT/sim/build"
mkdir -p "$WORK"
cd "$WORK"
FLAGS="--std=08 --workdir=$WORK"
for f in rtl/mram_pkg rtl/mram_write_guard rtl/mram_cmd_ctrl rtl/mram_qspi_backend rtl/sram_boot_copy \
         rtl/axi4_slave_wrapper rtl/mram_top sim/qspi_mram_model sim/axi_sram_model \
         sim/axi_master_checker sim/tb_sram_soak; do
    ghdl -a $FLAGS "$ROOT/$f.vhd"
done
ghdl -e $FLAGS tb_sram_soak
ghdl -r $FLAGS tb_sram_soak "$@"
