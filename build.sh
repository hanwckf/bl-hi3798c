#!/bin/sh

ATF_DEBUG=0
UBOOT_CFG=u-boot-sn1.config
CROSS_64=aarch64-linux-gnu-
CROSS_32=arm-linux-gnueabihf-

rm -f fastboot.bin l-loader.bin

echo "######  BUILD U-BOOT ######"
if [ -f ${UBOOT_CFG} ]; then
	cp -f ${UBOOT_CFG} u-boot/.config
else
	echo "${UBOOT_CFG} not found!"
	exit 1
fi
make -C u-boot CROSS_COMPILE=${CROSS_64} clean
make -j4 -C u-boot CROSS_COMPILE=${CROSS_64}

if [ ! -f u-boot/u-boot.bin ]; then
	echo "u-boot build failed!"
	exit 1
fi

echo "######  BUILD ATF ######"
if [ $ATF_DEBUG = 0 ]; then
	ATF_BIN_DIR=release
else
	ATF_BIN_DIR=debug
fi

make -C atf distclean
if [ "$1" = "RECOVERY" ]; then
	echo "BUILD ATF WITH RECOVERY"
	make -C atf CROSS_COMPILE=${CROSS_64} all fip \
		DEBUG=${ATF_DEBUG} PLAT=poplar SPD=none BL33=../u-boot/u-boot.bin \
		POPLAR_RECOVERY=1
else
	make -C atf CROSS_COMPILE=${CROSS_64} all fip \
		DEBUG=${ATF_DEBUG} PLAT=poplar SPD=none BL33=../u-boot/u-boot.bin
fi

if [ ! -f ./atf/build/poplar/${ATF_BIN_DIR}/bl1.bin ] || [ ! -f ./atf/build/poplar/${ATF_BIN_DIR}/fip.bin ]; then
	echo "ATF build failed!"
	exit 1
fi

echo "######  BUILD l-loader ######"
cp -f ./atf/build/poplar/${ATF_BIN_DIR}/bl1.bin ./l-loader/atf/
cp -f ./atf/build/poplar/${ATF_BIN_DIR}/fip.bin ./l-loader/atf/
make -C l-loader clean
if [ "$1" = "RECOVERY" ]; then
	echo "BUILD l-loader WITH RECOVERY"
	make -C l-loader CROSS_COMPILE=${CROSS_32} \
		ARM_TRUSTED_FIRMWARE=../atf RECOVERY=1
else
	make -C l-loader CROSS_COMPILE=${CROSS_32} \
		ARM_TRUSTED_FIRMWARE=../atf
fi

if [ "$1" = "RECOVERY" ]; then
	if [ -f l-loader/fastboot.bin ]; then
		cp -f l-loader/fastboot.bin .
	else
		echo "l-loader build failed!"
		exit 1
	fi
else
	if [ -f l-loader/l-loader.bin ]; then
		cp -f l-loader/l-loader.bin .
	else
		echo "l-loader build failed!"
		exit 1
	fi
fi
echo "######  BUILD DONE ######"

