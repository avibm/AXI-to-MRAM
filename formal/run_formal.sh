#!/usr/bin/env bash
# Bounded model check of sram_boot_copy (formal/fv_sram_boot_copy.vhd).
# Needs Yosys with the GHDL plugin (Ubuntu: yosys, yosys-plugin-ghdl).
#
#   BLOCKS=n formal/run_formal.sh ...      bursts per pass in the harness (default 2)
#   formal/run_formal.sh [depth]           prove ok = 1 in every cycle for
#                                          `depth` cycles after reset
#   formal/run_formal.sh depth probe_X     reachability: try to prove
#                                          probe_X = 0; a counterexample
#                                          (FAILED) means probe_X is reachable
#                                          within `depth` cycles, as it must be
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEPTH="${1:-60}"
PROVE="ok 1"
[[ $# -ge 2 ]] && PROVE="$2 0"
BLOCKS="${BLOCKS:-2}"
WORK="$ROOT/formal/build/b${BLOCKS}_d${DEPTH}_${2:-ok}"
mkdir -p "$WORK"
cd "$WORK"
if yosys -q -m ghdl -l "$WORK/formal.log" -p "
  ghdl --std=08 -gG_BLOCKS=$BLOCKS $ROOT/rtl/mram_pkg.vhd $ROOT/rtl/sram_boot_copy.vhd \
       $ROOT/formal/fv_sram_boot_copy.vhd -e fv_sram_boot_copy;
  prep -top fv_sram_boot_copy; flatten; memory_map; opt -fast;
  async2sync; dffunmap;
  sat -verify -seq $DEPTH -prove $PROVE -set env_ok 1 -set-init-zero -timeout 6000 -dump_vcd $WORK/cex.vcd
"; then
  echo "PROVED: $PROVE in every cycle for $DEPTH cycles from reset"
else
  if grep -q "SAT temporal induction proof finished - model found\|SAT proof finished - model found" "$WORK/formal.log"; then
    echo "COUNTEREXAMPLE: $PROVE fails within $DEPTH cycles (trace: $WORK/cex.vcd)"
  else
    echo "ERROR or TIMEOUT (see $WORK/formal.log)"
  fi
  exit 1
fi
