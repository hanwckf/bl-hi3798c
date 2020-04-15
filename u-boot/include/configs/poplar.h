/*
 * (C) Copyright 2017 Linaro
 *
 * Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 *
 * Configuration for Poplar 96boards CE. Parts were derived from other ARM
 * configurations.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _POPLAR_H_
#define _POPLAR_H_

#include <linux/sizes.h>

/* DRAM banks */
#define CONFIG_NR_DRAM_BANKS			2

/* SYS */
#define CONFIG_SYS_BOOTM_LEN			0x1400000
#define CONFIG_SYS_INIT_SP_ADDR			0x04200000
#define CONFIG_SYS_LOAD_ADDR			0x04800000
#define CONFIG_SYS_MALLOC_LEN			SZ_32M

/* ATF bl33.bin load address (must match) */
#define CONFIG_SYS_TEXT_BASE			0x37000000

/* PL010/PL011 */
#define CONFIG_PL01X_SERIAL

/* USB configuration */
#define CONFIG_USB_MAX_CONTROLLER_COUNT		2

/* SD/MMC */
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_SYS_MMC_MAX_BLK_COUNT		32767
#define CONFIG_SUPPORT_EMMC_BOOT

/* Network config */
#define CONFIG_MII				1
#define CONFIG_NET_MULTI			1
#define CONFIG_PHY_GIGE				1
#define CONFIG_ARP_TIMEOUT			500000L
#define CONFIG_NET_RETRY_COUNT 			50
#define CONFIG_SYS_FAULT_ECHO_LINK_DOWN 	1
#define CONFIG_SYS_RX_ETH_BUFFER		16

/* Android Image format*/
#define CONFIG_ANDROID_BOOT_IMAGE
#define CONFIG_SYS_BOOT_RAMDISK_HIGH
#define CONFIG_CMDLINE_TAG
#define CONFIG_INITRD_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_SUPPORT_RAW_INITRD
/*****************************************************************************
 *  Initial environment variables
 *****************************************************************************/

#define BOOT_TARGET_DEVICES(func)					\
					func(USB, usb, 0)		\
					func(MMC, mmc, 0)		\
					func(DHCP, dhcp, na)
#ifndef CONFIG_SPL_BUILD
#include <config_distro_defaults.h>
#include <config_distro_bootcmd.h>
#endif

#define CONFIG_EXTRA_ENV_SETTINGS					\
			"loader_mmc_blknum=0x0\0"			\
			"loader_mmc_nblks=0x780\0"			\
			"env_mmc_blknum=0xf80\0"			\
			"env_mmc_nblks=0x80\0"				\
			"kernel_addr_r=0x30000000\0"			\
			"pxefile_addr_r=0x32000000\0"			\
			"scriptaddr=0x32000000\0"			\
			"fdt_addr_r=0x32200000\0"			\
			"fdtfile=hisilicon/hi3798cv200-poplar.dtb\0"	\
			"ramdisk_addr_r=0x32400000\0"			\
			"initrd_high=0xffffffffffffffff\0"		\
			"android_addr_r=0x30000000\0"			\
			"android_bootargs=androidboot.hardware=poplar " \
			    "androidboot.selinux=permissive "		\
			    "mmz=ddr,0,0,60M\0"				\
			"setupa=setenv bootargs $android_bootargs; "	\
			    "usb start; "				\
			    "fatload usb 0:1 ${kernel_addr_r} Image; "	\
			    "fatload usb 0:1 ${fdt_addr_r} hi3798cv200-poplar.dtb; " \
			    "fatload usb 0:1 ${ramdisk_addr_r} ramdisk.android.uboot\0" \
			"boota=booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}\0" \
			"bootai=part start mmc 0 2 aistart; "		\
			    "part size mmc 0 2 aisize; "		\
			    "mmc read ${android_addr_r} ${aistart} ${aisize}; " \
			    "booti ${android_addr_r}\0" \
			BOOTENV


/* Command line configuration */
#define CONFIG_SYS_MMC_ENV_DEV		0
#define CONFIG_ENV_OFFSET		(0xf80 * 512) /* env_mmc_blknum bytes */
#define CONFIG_ENV_SIZE			(0x80 * 512)  /* env_mmc_nblks bytes */
#define CONFIG_FAT_WRITE
#define CONFIG_ENV_VARS_UBOOT_CONFIG

/* Monitor Command Prompt */
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_CBSIZE		512
#define CONFIG_SYS_MAXARGS		64

#define CONFIG_ENV_OVERWRITE

#endif /* _POPLAR_H_ */
