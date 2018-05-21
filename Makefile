TOOLCHAIN = arm-none-eabi
CC = $(TOOLCHAIN)-gcc
AS = $(TOOLCHAIN)-as
LD = $(TOOLCHAIN)-ld
SRC = .
OUT = build
OBJ = $(OUT)/obj
TARGET = -mcpu=cortex-m0plus -mthumb
OPTS = -ggdb3

.PHONY: all help build

all: init build

%.o: %.s
	$(AS) $(TARGET) $(OPTS) -c $(SRC)/$< -o $(OBJ)/$@ 

#%.o: %.c
#	$(CC) $(TARGET) -c $(SRC)/$< -o $(OBJ)/$@

init:
	mkdir -p $(OUT)
	mkdir -p $(OBJ)

build: uart0_driver.o queue.o string_io.o number_io.o Constants.o MKL46Z4.o Start.o Exercise11.o HardFault_LCD.o
	$(CC) $(TARGET) $(OPTS) -c Exercise11.c -o $(OBJ)/Exercise11_c.o
	$(LD) -T link.ld $(OBJ)/Exercise11_c.o $(OBJ)/Exercise11.o $(OBJ)/Start.o $(OBJ)/uart0_driver.o $(OBJ)/queue.o $(OBJ)/string_io.o $(OBJ)/number_io.o $(OBJ)/HardFault_LCD.o $(OBJ)/Constants.o $(OBJ)/MKL46Z4.o  -o $(OUT)/ex11.elf
	$(TOOLCHAIN)-objcopy -O binary $(OUT)/ex11.elf $(OUT)/ex11.bin

clean:
	-rm -Rf $(OUT)

disasm: build
	$(TOOLCHAIN)-objdump -D build/ex11.elf > $(OUT)/ex11.s

flash: build
	openocd -f frdm-kl46z.cfg -c "init;kinetis mdm mass_erase 0;reset halt;flash write_image build/ex11.bin 0 bin;reset halt; exit"
