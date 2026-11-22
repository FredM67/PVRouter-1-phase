# The 7-segment display without the pin-saving chips takes D2, D10 and D13, which the RF chip needs.
# One load on D4, the override pin on D3: the only free pins, so that only the RF chip clashes.
replace 'PCB_VERSION{ PcbVersion::NEW }' 'PCB_VERSION{ PcbVersion::OLD_SEG }'
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::SEG }'
replace 'physicalLoadPin[NO_OF_DUMPLOADS]{ Load::local(5) }' 'physicalLoadPin[NO_OF_DUMPLOADS]{ Load::local(4) }'
replace 'overridePins{ { { 4, ALL_LOADS() } } }' 'overridePins{ { { 3, ALL_LOADS() } } }'
replace 'RF_LOGGING_PRESENT{ false }' 'RF_LOGGING_PRESENT{ true }'
expect='The display uses pins that the RF chip needs'
