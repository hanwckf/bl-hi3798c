#
# (C) Copyright 2017 Linaro Limited
#
# Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
#
# Configuration for Poplar 96boards EE. Parts were derived from other ARM
# configurations.
#
# SPDX-License-Identifier:	GPL-2.0+
#

RECOVERY ?= 0
ARM_TRUSTED_FIRMWARE ?= ../arm-trusted-firmware/
ARM_TF_INCLUDE ?= $(ARM_TRUSTED_FIRMWARE)/plat/hisilicon/poplar/include

# Must use a 32-bit ARM cross-compiler
CROSS_COMPILE ?= arm-linux-gnueabihf-

CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
OBJCOPY=$(CROSS_COMPILE)objcopy

CFLAGS := -march=armv7-a
ifeq ($(RECOVERY),1)
CFLAGS += -DRECOVERY
endif

# Use build date/time and Git commit id to form a version message
VDATE=$(shell date '+%Y/%m/%d %H:%M:%S%z')
VCOMMIT=$(shell git rev-parse --verify --short HEAD 2>/dev/null)
VERSION_MSG='"LOADER:  Built $(VDATE) Commit-id $(VCOMMIT)"'

# LLOADER_LEN is the maximum size the loader can ever be (without
# changing the size of the ~4MB partition we use to hold it).  The
# size *includes* the single sector reserved to hold the MBR.  We
# truncate (up or down) the size of "l-loader.bin" to this size.
#
# Immediately after this area (at offset LLOADER_LEN on boot media)
# is 64KB reserved for U-Boot to store its persistent data.  And
# following that (at offset 2048KB) is 2048KB reserved for UEFI to
# hold its persistent data.
#
# Despite this, the "l-loader.bin" won't work if it's more than
# about 1029KB--arising from a FIP size of 790016 bytes--so the
# constant here is higher than the practical maximum.
LLOADER_LEN=1984K

ifeq ($(RECOVERY),1)
all: fastboot.bin loader.bin

fastboot.bin: l-loader.bin
	@cp $< $@
else
all: loader.bin
endif

loader.bin: l-loader.bin
	@dd status=none if=$< of=$@ bs=512 skip=1 conv=notrunc
	@cp $@ installer/recovery_files/
	@gzip installer/recovery_files/$@

l-loader.bin: l-loader
	$(OBJCOPY) -O binary $< $@
	truncate -s ${LLOADER_LEN} $@

l-loader: start.o debug.o l-loader.lds
	$(LD) -Bstatic -Tl-loader.lds start.o debug.o -o $@

start.o: start.S
	$(CC) -c -o $@ $< -I$(ARM_TF_INCLUDE) -DVERSION_MSG=$(VERSION_MSG) \
		$(CFLAGS)

start.S: atf/bl1.bin atf/fip.bin

atf/bl1.bin atf/fip.bin:
	@echo ""
	@echo "Error: \"$@\" is missing; it must be built from ARM-TF"
	@echo ""
	@false

debug.o: debug.S
	$(CC) -c -o $@ $<

l-loader.lds: l-loader.ld.in
	$(CPP) -P -o $@ - < $< -I$(ARM_TF_INCLUDE) $(CFLAGS)

clean:
	@rm -f *.o l-loader.lds l-loader l-loader.bin fastboot.bin \
		loader.bin installer/recovery_files/loader.bin.gz

distclean: clean
	@rm -f *.orig cscope.* atf/bl1.bin atf/fip.bin
