#!/bin/sh

UBOOT_CFG=u-boot-sn1.config
CROSS_64=aarch64-linux-gnu-
CROSS_32=arm-linux-gnueabihf-

rm -f fastboot.bin l-loader.bin

echo "######  BUILD U-BOOT ######"
cp -f ${UBOOT_CFG} u-boot/.config
make -C u-boot clean
make -j4 -C u-boot CROSS_COMPILE=${CROSS_64}

if [ ! -f u-boot/u-boot.bin ]; then
	echo "u-boot build failed!"
	exit 1
fi

echo "######  BUILD ATF ######"
make -C atf distclean
if [ "$1" = RECOVERY ]; then
	echo "BUILD ATF WITH DEBUG"
	make -C atf CROSS_COMPILE=${CROSS_64} all fip \
		DEBUG=1 PLAT=poplar SPD=none BL33=../u-boot/u-boot.bin \
		POPLAR_RECOVERY=1
else
	make -C atf CROSS_COMPILE=${CROSS_64} all fip \
		DEBUG=1 PLAT=poplar SPD=none BL33=../u-boot/u-boot.bin
fi

if [ ! -f ./atf/build/poplar/debug/bl1.bin ] || [ ! -f ./atf/build/poplar/debug/fip.bin ]; then
	echo "ATF build failed!"
	exit 1
fi

echo "######  BUILD l-loader ######"
cp -f ./atf/build/poplar/debug/bl1.bin ./l-loader/atf/
cp -f ./atf/build/poplar/debug/fip.bin ./l-loader/atf/
make -C l-loader clean
if [ "$1" = RECOVERY ]; then
	echo "BUILD l-loader WITH DEBUG"
	make -C l-loader CROSS_COMPILE=${CROSS_32} RECOVERY=1
else
	make -C l-loader CROSS_COMPILE=${CROSS_32}
fi

if [ "$1" = RECOVERY ]; then
	if [ -f l-loader/fastboot.bin ]; then
		cp -f l-loader/fastboot.bin .
		echo "######  BUILD DONE ######"
	else
		echo "l-loader build failed!"
		exit 1
	fi
else
	if [ -f l-loader/l-loader.bin ]; then
		cp -f l-loader/l-loader.bin .
		echo "######  BUILD DONE ######"
	else
		echo "l-loader build failed!"
		exit 1
	fi
fi

