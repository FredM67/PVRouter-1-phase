# 20 s at up to 68 sample sets per cycle is 68000 sets: the 16-bit counter of the datalog period,
# the divisor of the datalog power, would wrap (3 conversions per set, 64 sets per cycle at 50 Hz).
replace 'DATALOG_PERIOD_IN_SECONDS{ 5 }' 'DATALOG_PERIOD_IN_SECONDS{ 20 }'
expect="the sample sets of one datalog period must fit in 16 bits"
