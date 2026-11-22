# Bug 7: REMOTE_LOAD(3) with 3 remote loads (REMOTE_LOAD() counts from 0).
loads 3 'Load::remote(1), Load::remote(1), Load::remote(2)'
replace 'overridePins{ { { 4, ALL_LOADS() } } }' 'overridePins{ { { 4, { REMOTE_LOAD(3) } } } }'
expect='An override pin forces a remote load that does not exist'
