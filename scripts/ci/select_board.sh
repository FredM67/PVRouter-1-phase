#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# The board and the display of a build, set in config.h: for the build matrix of
# .github/workflows/build.yml and for the target branch measured by measure.sh, so that both
# build the same firmwares.
#
#   scripts/ci/select_board.sh <source tree> <OLD_SEG|OLD_OLED|NEW> [NONE|SEG|SEG_HW|OLED]
#
# SEG_HW also moves the loads off the display's pins, with sim/configs/seg_hw.patch (of this
# script's tree: the target tree may predate it). Fails if config.h does not take the change.
set -eu

tree=$1
board=$2
display=${3:-}
here=$(dirname "$0")
cfg=$tree/Mk2_1phase_RFdatalog_temp/config.h

if [ "$display" = SEG_HW ]; then
  sed -n '/^diff --git/,$p' "$here/../../sim/configs/seg_hw.patch" | patch -s -p1 -d "$tree" --forward
fi
sed -i -E "s/(PCB_VERSION\{ PcbVersion::)[A-Z_]+/\1$board/" "$cfg"
grep -q "PCB_VERSION{ PcbVersion::$board }" "$cfg"
if [ -n "$display" ]; then
  sed -i -E "s/(TYPE_OF_DISPLAY\{ DisplayType::)[A-Z_]+/\1$display/" "$cfg"
  grep -q "TYPE_OF_DISPLAY{ DisplayType::$display }" "$cfg"
fi
