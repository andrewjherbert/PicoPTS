#!/bin/bash
pushd $HOME/Dropbox/pico/PicoPTS
cd ../openocd/
src/openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f $HOME/Dropbox/pico/PicoPTS/picoptsm.cfg -f target/rp2040.cfg -c "program $HOME/Dropbox/pico/PicoPTS/build/picopts.elf verify reset exit" -s tcl
popd
