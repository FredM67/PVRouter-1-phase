#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# The same scenario must give the same result on every board: compares the summaries of two
# runs, scenario by scenario.
#
#   compare_boards.sh [out dir [run [% [Wh [reference]]]]]   (default: out new 1 0.1, OLD_SEG)
#
# run: the prefix of the summaries to check: new, new20 or old-oled (make check-new...).
# reference: the prefix of the summaries to compare with; none for OLD_SEG (make check).
# Compared: grid import and export, and the energy of each load and relay. Not the diverted
# energy the firmware reports: it counts whole Wh, which a few hundredths of a Wh can tip.
# Allowed difference: the given % or Wh, whichever is larger. The boards differ in resolution
# (with 50 A CTs, the new board has 2.6 times fewer ADC steps per watt than the old ones) and in
# analog inputs, which can move a switching instant by a half-cycle now and then: a few
# hundredths of a Wh over a scenario.
out=${1:-out}
run=${2:-new}
pct=${3:-1}
min_wh=${4:-0.1}
ref=${5:-}
ref_name=${ref:-old_seg}
status=0
found=0
for new in "$out"/$run-*-summary.txt; do
  [ -e "$new" ] || break
  found=1
  name=$(basename "$new" -summary.txt)
  name=${name#$run-}
  old="$out/${ref:+$ref-}$name-summary.txt"
  if [ ! -e "$old" ]; then
    echo "$name: no $ref_name summary ($old): run make check${ref:+-$ref}"
    status=1
    continue
  fi
  awk -v name="$name ($run against $ref_name)" -v ref="$ref_name" -v run="$run" -v pct="$pct" -v min_wh="$min_wh" '
    # figures of a summary: grid import/export, energy of each load and relay
    function grab(  i) {
      if ($1 == "Grid") {
        fig["grid import"] = $3; fig["grid export"] = $6
      } else if ($1 == "Load" || $1 == "Relay") {
        for (i = 3; i < NF; ++i)
          if ($(i + 1) == "Wh,") { fig[$1 " " $2] = $i; break }
      }
    }
    FNR == 1 { file++ }
    { delete fig; grab(); for (k in fig) val[file, k] = fig[k]; if (file == 1) for (k in fig) keys[k] = 1 }
    END {
      bad = 0
      for (k in keys) {
        a = val[1, k]; b = val[2, k]
        if (b == "") { printf "%s: %s missing in %s\n", name, k, run; bad = 1; continue }
        d = a - b; if (d < 0) d = -d
        m = a > b ? a : b; tol = m * pct / 100; if (tol < min_wh) tol = min_wh
        if (d > tol) { printf "%s: %s %.2f Wh (%s) / %.2f Wh (%s), off by %.2f Wh\n", name, k, a, ref, b, run, d; bad = 1 }
      }
      printf "%s: %s\n", name, bad ? "DIFFERENT results" : "same result"
      exit bad
    }' "$old" "$new" || status=1
done
if [ $found = 0 ]; then
  echo "no $run-* summary in $out: run make check-$run"
  exit 1
fi
exit $status
