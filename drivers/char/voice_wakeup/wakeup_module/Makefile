TOPDIR = .
CROSS_COMPILE ?= mips-linux-gnu-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP	= $(CROSS_COMPILE)objdump
drop-sections := .reginfo .mdebug .oomment .note .pdr .options .MIPS.options
strip-flags   := $(addprefix --remove-section=,$(drop-sections))

CFLAGS += -I$(TOPDIR)/include
CFLAGS += -nostdinc -Wall -Wundef -Werror-implicit-function-declaration \
	 -fno-common -EL -Os -march=mips32 -mabi=32 -G 0 -mno-abicalls -fno-pic\
	 -msoft-float

LDFLAGS	:= -nostdlib -EL -T target.ld
OBJCOPY_ARGS  := -O elf32-tradlittlemips

OBJS := $(TOPDIR)/src/start.o		\
	$(TOPDIR)/src/interface.o		\
	$(TOPDIR)/src/dmic_ops.o		\
	$(TOPDIR)/src/dma_ops.o		\
	$(TOPDIR)/src/rtc_ops.o		\
	$(TOPDIR)/src/jz_dma.o		\
	$(TOPDIR)/src/trigger_value_adjust.o		\
	$(TOPDIR)/src/voice_wakeup.o		\
	$(TOPDIR)/src/tcu_timer.o		\
	$(TOPDIR)/lib/libivw3_6.a


all: $(TOPDIR)/firmware.bin
	@hexdump -v -e '"0x" 1/4 "%08x" "," "\n"' $< > $(TOPDIR)/firmware.hex

$(TOPDIR)/firmware.bin:$(TOPDIR)/firmware.o
	@$(LD) -nostdlib -EL -T $(TOPDIR)/target.ld $(OBJS) -Map $(TOPDIR)/tmp.map -o $(TOPDIR)/tmp.elf
	@$(OBJCOPY) $(strip-flags) $(OBJCOPY_ARGS) -O binary $(TOPDIR)/tmp.elf $@
	@$(OBJDUMP) $(TOPDIR)/tmp.elf -D > tmp.dump
$(TOPDIR)/firmware.o:$(OBJS)

$(TOPDIR)/lib/libivw3_6.a:$(TOPDIR)/lib/libivw3_6.hex
	cp $(TOPDIR)/lib/libivw3_6.hex $(TOPDIR)/lib/libivw3_6.a

%.o:%.c
	$(CC) $(CFLAGS) -o $@ -c $^

%.o:%.S
	$(CC) $(CFLAGS) -o $@ -c $^

clean:
	find . -name "*.o" | xargs rm -vf
	find . -name "*.o.cmd" | xargs rm -vf

