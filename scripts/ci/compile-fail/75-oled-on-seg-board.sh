# The OLED sits on A4/A5, the analog inputs of the first old board.
replace 'PCB_VERSION{ PcbVersion::NEW }' 'PCB_VERSION{ PcbVersion::OLD_SEG }'
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::OLED }'
expect='The OLED needs A4/A5, which PcbVersion::OLD_SEG uses as analog inputs'
