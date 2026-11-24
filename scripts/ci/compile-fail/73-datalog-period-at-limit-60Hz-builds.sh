# 30 s at 60 Hz (53 sample sets per cycle, up to 57: 102600 sets) builds too.
replace 'SUPPLY_FREQUENCY{ 50 }' 'SUPPLY_FREQUENCY{ 60 }'
replace 'DATALOG_PERIOD_IN_SECONDS{ 5 }' 'DATALOG_PERIOD_IN_SECONDS{ 30 }'
