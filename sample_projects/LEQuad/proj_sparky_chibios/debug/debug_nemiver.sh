#!/bin/bash

# location of OpenOCD Board .cfg files
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# start xterm with openocd in the background
# xterm -e openocd -f $OPENOCD_BOARD_DIR/stm32f0discovery.cfg -f debug/stm32f0-openocd.cfg -f debug/debug.cfg &
xterm -e openocd -f debug/openocd.common.cfg -f debug/openocd.stm32f4disco.cfg  -f debug/debug.cfg &

# save the PID of the background process
XTERM_PID=$!

# wait a bit to be sure the hardware is ready
sleep 2

# execute some initialisation commands via gdb
arm-none-eabi-gdb --batch --command=debug/run.gdb $1

# start the gdb gui
nemiver --remote=localhost:3333 --gdb-binary="$(which arm-none-eabi-gdb)" $1

# close xterm when the user has exited nemiver
kill $XTERM_PID
