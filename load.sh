#!/bin/bash
pushd ~/home/pico/PicoPTS
sudo openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f picopts.cfg -f target/rp2040.cfg -c "program /home/ajh2/home/pico/PicoPTS/build/picopts.elf verify reset exit" -s tcl
popd

