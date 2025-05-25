CDROM_motors
Ver. 1.4
Turcotronics, License GPL Version 3
https://www.turcotronics.it
https://www.turcotronics.com
https://www.ebay.it/usr/turcotronics

You can connect directly to port 80 IP 192.168.4.1 or use websockets to port 81
See example CDROM_motors.html

The used board is the ESP32 DevkitC V4 but it can be easily adapted to any ESP32 board
Check out it on https://www.ebay.it/usr/turcotronics or use your own one :-)

In the beginnig of CDROM_motors.ino you can choice the Kit number or your ports and constants.

Absolute position in based on time, so it isn't precise, return backward to zeros.

WiFi ssid: CDROM_motors
WiFi password: 1234567890

Use:
   WebSockets by Markus Sattler
   Freenove WS2812 Lib for ESP32 by Freenove

DO NOT use the ESP32 USB connector to power the motors, it's for programming only.
