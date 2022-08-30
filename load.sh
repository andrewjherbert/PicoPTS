#!/bin/bash
pushd ~/home/pico/PicoPTS
openocd -f interface/picoprobe.cfg -f picopts.cfg -f target/rp2040.cfg -c "program /home/ajh2/pico/PicoPTS/build/picopts.elf verify reset exit"
popd

