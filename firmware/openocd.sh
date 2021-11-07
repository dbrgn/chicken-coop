openocd -f jlink.cfg -f stm32f4x.cfg -c 'init; jlink hwstatus; reset; halt'
