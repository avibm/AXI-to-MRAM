#!/usr/bin/env bash
# Unbounded proof of the sram_boot_copy properties (formal/fv_sram_boot_copy.vhd)
# with ABC's PDR engine: output "bad" must never be 1 in any reachable state.
# Needs Yosys with the GHDL plugin and yosys-abc (Ubuntu: yosys, yosys-plugin-ghdl).
# Usage: BLOCKS=n formal/run_pdr.sh      (bursts per pass in the harness, default 2)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BLOCKS="${BLOCKS:-2}"
WORK="$ROOT/formal/build/pdr_b${BLOCKS}"
mkdir -p "$WORK"
cd "$WORK"
yosys -q -m ghdl -l "$WORK/yosys.log" -p "
  ghdl --std=08 -gG_BLOCKS=$BLOCKS $ROOT/rtl/mram_pkg.vhd $ROOT/rtl/sram_boot_copy.vhd \
       $ROOT/formal/fv_sram_boot_copy.vhd -e fv_sram_boot_copy;
  prep -top fv_sram_boot_copy; flatten; memory_map; opt -fast;
  async2sync; dffunmap; formalff -clk2ff;
  select -module fv_sram_boot_copy o:* o:bad %d; delete -output; select -clear;
  opt_clean; setundef -undriven -anyseq;
  techmap; opt -fast; aigmap; opt_clean;
  write_aiger -zinit -map $WORK/model.aim $WORK/model.aig
"
yosys-abc -c "read_aiger $WORK/model.aig; strash; pdr" | tee "$WORK/abc.log" | tail -5
if grep -q "Property proved" "$WORK/abc.log"; then
  echo "PROVED (unbounded): bad is never 1"
elif grep -qi "Output 0 of miter .* was asserted\|counter-example\|failed" "$WORK/abc.log"; then
  echo "COUNTEREXAMPLE (see $WORK/abc.log)"; exit 1
else
  echo "UNDECIDED (see $WORK/abc.log)"; exit 1
fi
