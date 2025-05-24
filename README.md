# CDROM_motors
CDROM motors controller with ESP32 DevKitC_V4 and firmware.\
It use the motors from an old CDROM with its mechanic assembly.\
Open source Arduino IDE firmware and HTLM websocket software;\
it can be controlled by a smartphone, tablet or PC connecting it to its WiFi web server or via websocket.\
If you need the hardware maybe there are still some on my https://www.ebay.it/usr/turcotronics or just use any ESP32 board with a 2 motors driver and the connectors.\
Typical CDROM motors are normal 2 pins 5V DC, so you can use the software to controls 2 simple 5V DC motors.\
Absolute position in based on time, so it isn't precise.\
DO NOT use the ESP32 USB connector to power the motors, it's for programming only.
