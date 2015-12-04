##############################  Handle IRQ  #############################
HAVE_UART	= y

##############################     TOOLS    #############################
CROSS_COMPILE ?=mips-linux-gnu-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy

drop-sections := .reginfo .mdebug .oomment .note .pdr .options .MIPS.options
strip-flags   := $(addprefix --remove-section=,$(drop-sections))

HEX_CFLAGS += -Iinclude
HEX_CFLAGS += -nostdinc -Wall -Wundef -Werror-implicit-function-declaration \
	 -fno-common -EL -Os -march=mips32 -mabi=32 -G 0 -mno-abicalls -fno-pic
HEX_CFLAGS += -DINTERFACE_NEMC

HEX_LDFLAGS	:= -nostdlib -EL -T target.ld
HEX_OBJCOPY_ARGS  := -O elf32-tradlittlemips

HEX_NAME := firmware_uart.hex

OBJS := src/start.o		\
	src/delay.o		\
	src/main.o		\
	src/mcu_ops.o		\
	src/gpio.o		\

ifeq ($(HAVE_UART), y)
OBJS += src/uart.o
HEX_CFLAGS += -DHANDLE_UART
endif

all: firmware.bin
	@hexdump -v -e '"0x" 1/4 "%08x" "," "\n"' $< > $(HEX_NAME)

firmware.bin:firmware.o
	@$(LD) -nostdlib -EL -T target.ld $(OBJS) -Map tmp.map -o tmp.elf
	@$(OBJCOPY) $(strip-flags) $(HEX_OBJCOPY_ARGS) -O binary tmp.elf $@

firmware.o : $(OBJS)

%.o:%.c
	$(CC) $(HEX_CFLAGS) -o $@ -c $^

%.o:%.S
	$(CC) $(HEX_CFLAGS) -o $@ -c $^

clean:
	@find . -name "*.o" | xargs rm -vf
	@find . -name "*.o.cmd" | xargs rm -vf
	@find . -name "*.hex" | xargs rm -vf
	@find . -name "*.bin" | xargs rm -vf
	@rm -vf tmp.map tmp.elf
