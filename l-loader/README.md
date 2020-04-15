Copyright (c) 2017 Linaro Limited

The l-loader is a 32 bit executable that transitions the default
processor state to 64 bit mode and boots the ARM Trusted
Firmware.<br/>

It complies with platform's BootROM requirements to support booting
the Poplar board <br/>

This is achieved via encoding values as well as binaries at
pre-defined locations in the program file.<br/>

Requirements:
 1. BootROM shall receive the PLL/DDR/regulator settings from l-loader
 <br/>
 
 2. BootROM shall receive the size of the binary to boot as well as
 the expected RAM location to boot it from.<br/>

BootROM platform initialization
-------------------------------
The basic platform initialization is encoded in a number of
pre-compiled binaries included at certain locations in l-loader<br/>


- AUXCODE.img<br/>
- BOOT_[0..2].reg<br/>

The location (i.e., offset relative to the base of the TEXT segment)
of the AUXCODE is defined by the address stored at offset
CONFIG_AUXCODE_AREA_POS.<br/>
The location of BOOT_[0..2].reg blocks are defined by the address
stored at offset CONFIG_PARAM_START_ADDR_POS.<br/>


The boot process.
-----------------
Once the BootROM has executed the AUXCODE initialization binary, it
loads the rest of the file in DDR for execution.<br/>

The location in DDR where the binary shall be placed is written at
CONFIG_BOOT_STORE_ADDR_POS and its value defined by
CONFIG_BOOT_STORE_ADDR_VAL.

The size of the binary that shall be copied to DDR needs to be placed
at CONFIG_TOTAL_BOOT_AREA_LEN_POS.<br/>

ROM then jumps to the offset specified in CONFIG_BOOT_ENTRY_POS, which contains
the instruction to branch to; l-loader then executes a warm reboot into 64bit
mode to start.S in the atf.<br/>

Compilation
------------

- pull and build the ATF:<br/>

  $ git clone https://github.com/96boards-poplar/arm-trusted-firmware.git <br/>
  $ cd arm-trusted-firmware<br/>
  $ make CROSS_COMPILE=aarch64-linux-gnu- all fip \
		SPD=none BL33=~/poplar/bin/u-boot.bin DEBUG=1 PLAT=poplar <br\>

- copy the ATF (bl1.bin, fip.bin) to l-loader/atf/ <br/>

- build l-loader:<br/>

  $ cd l-loader<br/>
  $ make clean <br/>
  $ make<br/>


