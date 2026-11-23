#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-or-later
# Three local loads on D5, D6 and D7, whatever the load count config.h ships with: the grid
# scenarios (sim/scenarios) and the patches of this directory (remote_loads, seg_hw, relays)
# assume them. sim_config.sh calls it; the CI builds that apply a patch without sim_config.sh
# (remote loads, SEG_HW) call it first. Does nothing to a config.h that has them already.
#
#   sim/configs/three_loads.sh [config.h]   (default: Mk2_1phase_RFdatalog_temp/config.h)
set -e
cfg=${1:-Mk2_1phase_RFdatalog_temp/config.h}
sed -i -E 's/^(inline constexpr uint8_t NO_OF_DUMPLOADS\{ )[0-9]+( \})/\13\2/;
  s/^(inline constexpr uint8_t loadPrioritiesAtStartup\[NO_OF_DUMPLOADS\]\{ )[0-9, ]+( \})/\10, 1, 2\2/' "$cfg"
# the declaration only (the comment above it quotes the same text)
sed -i -z -E 's/\ninline constexpr uint8_t physicalLoadPin\[NO_OF_DUMPLOADS\]\{[^}]*\}/\ninline constexpr uint8_t physicalLoadPin[NO_OF_DUMPLOADS]{\n  Load::local(5),\n  Load::local(6),\n  Load::local(7)\n}/' "$cfg"
grep -q '^inline constexpr uint8_t NO_OF_DUMPLOADS{ 3 };' "$cfg"
grep -q '^inline constexpr uint8_t loadPrioritiesAtStartup\[NO_OF_DUMPLOADS\]{ 0, 1, 2 };' "$cfg"
grep -q '^inline constexpr uint8_t physicalLoadPin\[NO_OF_DUMPLOADS\]{$' "$cfg"
