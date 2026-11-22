# The 7-segment display behind the pin-saving chips takes D5 to D9: a load on D5 clashes.
replace 'PCB_VERSION{ PcbVersion::NEW }' 'PCB_VERSION{ PcbVersion::OLD_SEG }'
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::SEG_HW }'
expect='Duplicate pin definition'
