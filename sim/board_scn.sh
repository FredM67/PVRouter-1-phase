#!/bin/sh
# The settings of a board, as grid_sim settings: its analog inputs, its ADC reference, its
# calibration read from calibration.h (the simulator must generate the voltages and powers the
# firmware reads back with the same values), and the mains.
#
#   board_scn.sh OLD_SEG|OLD_OLED|NEW [calibration.h [CT rating [frequency]]] > board.scn
#   then: grid_sim -b board.scn ...
#
# OLD_SEG:  V A3, CT1 A5, CT2 A4, AVCC 3.3 V
# OLD_OLED: V A0, CT1 A1, CT2 A3, AVCC 3.3 V
#           old boards: vcal = f_voltageCalOldBoard, pcal = f_powerCalOldBoard (CT1) and
#           f_powerCalDivertedOldBoard (CT2)
# NEW:      V A0, CT1 A1, CT2 A3, internal 1.1 V reference (AVCC unused: 5 V)
#           vcal = f_voltageCalNewBoard, pcal = f_powerCalPerAmp x f_ctRating (CT1) and
#           x f_ctRatingDiverted (CT2), or x the given CT rating for both: the firmware must be
#           built with the same rating (configs/sim_config.sh NEW <rating>)
# CT rating: '' for the ratings of calibration.h. frequency: 50 Hz by default; the firmware must
# be built for it (configs/sim_config.sh ... 60Hz).
# The lowest sample-set count per mains cycle is expected too: a set is 3 conversions of 1664
# cycles at 16 MHz, 64.1 sets per cycle at 50 Hz, 53.4 at 60 Hz; one less than the whole sets.
set -e
board=$1
cal=${2:-../Mk2_1phase_RFdatalog_temp/calibration.h}
ct=$3
freq=${4:-50}

# a value, from its definition line (commented-out lines are skipped)
value() {
  v=$(sed -n "s/^inline constexpr float $1{ *\([-0-9.eE]*\)F\{0,1\} *}.*/\1/p" "$cal")
  [ -n "$v" ] || { echo "board_scn.sh: no value for $1 in $cal" >&2; exit 1; }
  echo "$v"
}

echo "# $board board, from $cal${ct:+, CTs of $ct A}, $freq Hz"
case $board in
OLD_SEG | OLD_OLED)
  if [ "$board" = OLD_SEG ]; then echo "channels V A3 I1 A5 I2 A4"; else echo "channels V A0 I1 A1 I2 A3"; fi
  echo "avcc 3300"
  echo "vcal $(value f_voltageCalOldBoard)"
  echo "pcal $(value f_powerCalOldBoard) $(value f_powerCalDivertedOldBoard)"
  ;;
NEW)
  echo "channels V A0 I1 A1 I2 A3"
  echo "avcc 5000"
  echo "vcal $(value f_voltageCalNewBoard)"
  rating=$(value f_ctRating)
  rating2=$(value f_ctRatingDiverted)
  if [ -n "$ct" ]; then rating=$ct; rating2=$ct; fi
  echo "$(value f_powerCalPerAmp) $rating $rating2" | awk '{ printf "pcal %.9g %.9g\n", $1 * $2, $1 * $3 }'
  ;;
*)
  echo "usage: board_scn.sh OLD_SEG|OLD_OLED|NEW [calibration.h [CT rating [frequency]]]" >&2
  exit 2
  ;;
esac
echo "mains $freq 230"
awk -v f="$freq" 'BEGIN { printf "expect min_sample_sets %d\n", int(16000000 / (3 * 1664) / f) - 1 }'
