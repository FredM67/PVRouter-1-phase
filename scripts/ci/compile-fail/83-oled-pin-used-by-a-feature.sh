# The OLED sits on A4 (pin 18) and A5: a temperature sensor on A4 clashes with it.
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::OLED }'
temperature_on
replace 'temperatureSensing{ 3,' 'temperatureSensing{ 18,'
expect='Duplicate pin definition'
