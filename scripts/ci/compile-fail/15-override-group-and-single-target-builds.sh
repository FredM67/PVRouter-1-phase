# A group and single targets in one list, which the former bitmask helpers could not mix, builds.
loads 3 'Load::local(5), Load::local(6), Load::local(7)'
replace 'overridePins{ { { 4, ALL_LOADS() } } }' 'overridePins{ { { 4, { ALL_LOCAL_LOADS(), LOAD(1) } } } }'
