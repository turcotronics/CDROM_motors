#!/bin/bash
# init
esptool --chip auto --port /dev/ttyUSB0 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size 4MB 0x0 ./CDROM_motors.ino.merged.bin
read -p "Press any key to continue"
