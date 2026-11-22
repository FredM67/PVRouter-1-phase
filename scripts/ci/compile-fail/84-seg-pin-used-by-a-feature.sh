# The 7-segment display (no chips) takes A2 (pin 16): a temperature sensor on A2 clashes with it.
# One load on D4, override on D3, as case 76, so that only A2 clashes.
replace 'PCB_VERSION{ PcbVersion::NEW }' 'PCB_VERSION{ PcbVersion::OLD_SEG }'
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::SEG }'
replace 'NO_OF_DUMPLOADS{ 3 }' 'NO_OF_DUMPLOADS{ 1 }'
replace $'{\n  Load::local(5),\n  Load::local(6),\n  Load::local(7)\n}' '{ Load::local(4) }'
replace 'loadPrioritiesAtStartup[NO_OF_DUMPLOADS]{ 0, 1, 2 }' 'loadPrioritiesAtStartup[NO_OF_DUMPLOADS]{ 0 }'
replace 'overridePins{ { { 4, ALL_LOADS() } } }' 'overridePins{ { { 3, ALL_LOADS() } } }'
temperature_on
replace 'temperatureSensing{ 3,' 'temperatureSensing{ 16,'
expect='Duplicate pin definition'
