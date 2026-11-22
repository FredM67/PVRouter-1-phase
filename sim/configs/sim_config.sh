#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Settings the simulated grid needs, whatever the defaults of config.h:
# - the board: OLD_SEG by default (V A3, CT1 A5, CT2 A4, AVCC reference), OLD_OLED (V A0, CT1 A1,
#   CT2 A3, AVCC reference) or NEW (the same inputs, internal 1.1 V reference); grid_sim takes
#   the board's inputs and calibration from board_scn.sh, which reads calibration.h;
# - optionally, the CT rating of the new board, written into f_ctRating and f_ctRatingDiverted of
#   calibration.h: 20 for 'make check-new20' (about the resolution of the old boards);
# - optionally, a 60 Hz mains: SUPPLY_FREQUENCY of config_system.h, for 'make check-60hz';
# - the human-readable output: grid_sim reads its datalog;
# - three local loads on D5, D6 and D7 (three_loads.sh): config.h ships with one, the scenarios and
#   the patches of this directory assume three.
#
#   sim/configs/sim_config.sh [OLD_SEG|OLD_OLED|NEW] [CT rating] [60Hz] [config.h]
#   (default: OLD_SEG, ratings of calibration.h, 50 Hz, Mk2_1phase_RFdatalog_temp/config.h)
set -e
board=OLD_SEG
case $1 in OLD_SEG | OLD_OLED | NEW) board=$1; shift ;; esac
ct=
case $1 in *Hz) ;; [0-9]*) ct=$1; shift ;; esac
freq=
case $1 in 60Hz) freq=60; shift ;; esac
cfg=${1:-Mk2_1phase_RFdatalog_temp/config.h}
dir=$(dirname "$cfg")
sed -i -E "s/(PCB_VERSION\{ PcbVersion::)[A-Z_]+/\1$board/; s/(SERIAL_OUTPUT_TYPE = SerialOutputType::)[A-Za-z]+/\1HumanReadable/" "$cfg"
grep -q "PCB_VERSION{ PcbVersion::$board }" "$cfg"
grep -q 'SERIAL_OUTPUT_TYPE = SerialOutputType::HumanReadable;' "$cfg"
"$(dirname "$0")/three_loads.sh" "$cfg"
if [ -n "$ct" ]; then
  case $ct in *.*) ;; *) ct=$ct.0 ;; esac # 20 -> 20.0F, a float literal
  cal=$dir/calibration.h
  sed -i -E "s/^(inline constexpr float f_ctRating(Diverted)?)\{ *[0-9.]+F *\}/\1{ ${ct}F }/" "$cal"
  grep -q "^inline constexpr float f_ctRating{ ${ct}F }" "$cal"
  grep -q "^inline constexpr float f_ctRatingDiverted{ ${ct}F }" "$cal"
fi
if [ -n "$freq" ]; then
  sys=$dir/config_system.h
  sed -i -E "s/^(inline constexpr uint8_t SUPPLY_FREQUENCY)\{ *[0-9]+ *\}/\1{ $freq }/" "$sys"
  grep -q "^inline constexpr uint8_t SUPPLY_FREQUENCY{ $freq }" "$sys"
fi
