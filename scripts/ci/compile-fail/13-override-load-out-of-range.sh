# LOAD(1) with 1 load (LOAD() counts from 0).
replace 'overridePins{ { { 4, ALL_LOADS() } } }' 'overridePins{ { { 4, { LOAD(1) } } } }'
expect='An override pin forces a pin that is neither a local load nor a relay'
