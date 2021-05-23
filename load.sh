#!/bin/sh
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program /home/ajh2/pico/PicoPTS/build/picopts.elf verify reset exit"
