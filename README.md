# CDROM_motors
CDROM motors controller with ESP32 DevKitC_V4 and firmware for Arduino IDE and HTLM websocket.\
It uses the motors from an old CDROM with its mechanic assembly.\
It can be controlled by a smartphone, tablet or PC connecting it to its WiFi web server port 80 IP 192.168.4.1 or use websockets to port 81.\
If you need the hardware maybe there are still some on my https://www.ebay.it/usr/turcotronics or just use any ESP32 board with a 2 motors driver.\
Typical CDROM motors are 2 pins 5V DC.\
Absolute position is based on time, so it isn't precise, return backward to zeros.\
WiFi ssid: CDROM_motors\
WiFi password: 1234567890\
DO NOT use the ESP32 USB connector to power the motors, it's for programming only.

Turcotronics, License GPL Version 3\
https://www.turcotronics.it \
https://www.turcotronics.com \
https://www.ebay.it/usr/turcotronics 
