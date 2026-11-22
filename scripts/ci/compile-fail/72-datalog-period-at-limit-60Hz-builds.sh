# 60 Hz: 53 sample sets per cycle, up to 57; 19 s still fit in 16 bits and build.
replace 'SUPPLY_FREQUENCY{ 50 }' 'SUPPLY_FREQUENCY{ 60 }'
replace 'DATALOG_PERIOD_IN_SECONDS{ 5 }' 'DATALOG_PERIOD_IN_SECONDS{ 19 }'
