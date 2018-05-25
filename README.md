## FRDM-KL46Z

### what
 - Bare-metal ARM/Thumb development support files for NXP/Freescale's Kinetis
   Freedom/Tower development boards.
 - Linker script for KL46Z
 - OpenOCD flash file for KL46Z included from OpenOCD mainline distribution
 - Makefile for compilation and flashing
 - Example mixed C and Assembly source code, translated from RIT CMPE-250
   laboratory exercise eleven.
 - Completely open-source toolchain

### why
 - Free, OS-independent, vendor-independent, open-source toolchains were desired
   for advanced development and compatibility with existing code on the more
   commonly used GNU compiler platform.
 - Fine-grained, clear and simple control over the linking process was desired

### licensing
 - uhh maybe don't look at this if you're in CMPE-250?

### running
 - Start openocd by typing `sudo make debug` at the console
 - Run the `telnet.sh` and `gdb.sh` scripts in separate terminals.
 - If GDB complains about 'Target not halted', try typing `reset init` at the
   telnet console.  This will instruct OpenOCD to reset the board and wait for
   the debugger.
