# Every feature at once, with the OLED: relays on D8 and D9, two remote loads, a temperature
# sensor on D7, dual tariff on D3, automatic rotation. It must build.
replace 'TYPE_OF_DISPLAY{ DisplayType::NONE }' 'TYPE_OF_DISPLAY{ DisplayType::OLED }'
loads 4 'Load::local(5), Load::local(6), Load::remote(1), Load::remote(2)'
replace 'PRIORITY_ROTATION{ RotationModes::OFF }' 'PRIORITY_ROTATION{ RotationModes::AUTO }'
replace 'rotationGroups{};' 'rotationGroups{ { ALL_LOCAL_LOADS() } };'
replace 'RELAY_DIVERSION{ false }' 'RELAY_DIVERSION{ true }'
replace $'{ { unused_pin, 100, 200, 1, 1 },\n                                       { unused_pin, 300, 400, 1, 1 },\n                                       { unused_pin, 500, 600, 1, 1 } } }' $'{ { 8, 1000, 200, 2, 2 },\n                                       { 9, 2000, 200, 2, 2 } } }'
dual_tariff_on
replace 'dualTariffPin{ 8 }' 'dualTariffPin{ 3 }'
temperature_on
replace 'temperatureSensing{ 3,' 'temperatureSensing{ 7,'
expect=''
