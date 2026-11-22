# The 7-segment display is wired on the first old board only.
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::SEG_HW }'
expect='The 7-segment display only exists on PcbVersion::OLD_SEG'
