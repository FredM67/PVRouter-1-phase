# A3 (pin 17) is an analog input of every board: no digital feature can use it, display or not.
temperature_on
replace 'temperatureSensing{ 3,' 'temperatureSensing{ 17,'
expect='Pins 14 and above'
