/*
 * (C) Copyright 2017 Linaro Limited
 *
 * Jorge Ramirez-Ortiz <jorge.ramirez-ortiz@linaro.org>
 *
 * Configuration for Poplar 96boards EE. Parts were derived from other ARM
 * configurations.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
 * +--------------------------------------------+
 * | HW info					|
 * +--------------------------------------------+
 */
#define REG_BASE_SCTL				(0xF8000000)
#define REG_SC_GEN1				(0x0084)

#define REG_PERI_CPU_RVBARADDR			(0xF8A80000)
#define REG_PERI_CPU_RVBARADDR_OFFSET		(0x34)
#define REG_PERI_CPU_AARCH_MODE_OFFSET		(0x30)

#define REG_SYS_CNT_BASE			(0xF8BB0000)
#define REG_SYS_CNTCR				(REG_SYS_CNT_BASE)
#define REG_SYS_CNTFID0				(REG_SYS_CNT_BASE + 0x20)

#define CPU_CTRL_AARCH64_MODE			(0xF)

/*
 * +--------------------------------------------+
 * | Binary configuration			|
 * +--------------------------------------------+
 */

/* 1. ARM TRUSTED FIRMWARE */
#include "poplar_layout.h"

/* 2. BOOTROM REQUIREMENTS */
#define CONFIG_AUXAREA_ADR_POS			(0x214)
#define CONFIG_AUXAREA_LEN_POS			(0x218)
#define CONFIG_BOOT_ENTRY_POS			(0x21C)
#define CONFIG_SCS_HASHED_AREA_LEN_POS		(0x404)
#define CONFIG_TOTAL_BOOT_AREA_LEN_POS		(0x408)
#define CONFIG_DEFAULT_BOOT_REG_POS		(0x480)
#define CONFIG_PARAM_AREA_POS			(0x400)
#define CONFIG_SCS_HASHED_AREA_OFF_POS		(0x400)
#define CONFIG_PARAM_AREA_SIG_POS		(0x2EC0)
#define CONFIG_SUPPORT_MULTI_PARAM_POS		(0x2FE0)
#define CONFIG_AUXCODE_AREA_POS			(0x3000)
#define CONFIG_CHECKED_AREA_START		(0x6500)

#define CONFIG_UNCHK_SIZE			(0x100)
#define CONFIG_SCS_HASHED_AREA_OFF_VAL		(0x100)
#define CONFIG_AUXAREA_LEN			(0x3400)

#define CONFIG_BOOT_FLAG_POS			(0x2FC4)
#define CONFIG_BOOT_FLAG_VAL			(0x435A590D)

#define CONFIG_AUX_ENC_FLAG_POS			(0x2FC8)
#define CONFIG_AUX_ENC_FLAG_VAL			(0x2A13C812)

#define CONFIG_PARAM_START_ADDR_POS		(0x2FE4)
#define CONFIG_PARAM_ITEM_LEN_POS		(0x2FE8)

#define CONFIG_BOOT_STORE_ADDR_POS		(0x2FEC)
#define CONFIG_BOOT_STORE_ADDR_VAL		(LLOADER_TEXT_BASE)

#define CONFIG_REG_SIZE				(0x2000)

/*
 * +--------------------------------------------+
 * | Boot Message				|
 * +--------------------------------------------+
 */
#define BOOT_MSG				\
"\nLOADER:  Switched to aarch64 mode\nLOADER:  Entering ARM TRUSTED FIRMWARE\nLOADER:  CPU0 executes at 0x"

.global start
start:
	b _checked_area_start
/*
 * +--------------------------------------------+
 * | 1. Head Area (Key and Param)		|
 * +--------------------------------------------+
 */
.=CONFIG_AUXAREA_ADR_POS
	.word _auxcode_start-LLOADER_TEXT_BASE

.=CONFIG_AUXAREA_LEN_POS
	.word CONFIG_AUXAREA_LEN

.=CONFIG_BOOT_ENTRY_POS
	.word _checked_area_start
/*
 * +--------------------------------------------+
 * | 2. Param Area				|
 * +--------------------------------------------+
 */
.=CONFIG_PARAM_AREA_POS

.=CONFIG_SCS_HASHED_AREA_OFF_POS
	.word CONFIG_SCS_HASHED_AREA_OFF_VAL

.=CONFIG_SCS_HASHED_AREA_LEN_POS
	.word _checked_area_end - _checked_area_start

.=CONFIG_TOTAL_BOOT_AREA_LEN_POS
	.word _total_boot_area_end - _total_boot_area_start

.=CONFIG_DEFAULT_BOOT_REG_POS
	.incbin	"bin/BOOT_0.reg"

/* 2.5 Param signature */
.=CONFIG_PARAM_AREA_SIG_POS
	.fill 0x100,1,0

/*
 * +--------------------------------------------+
 * | 3. Unchecked area for hisi			|
 * +--------------------------------------------+
 */

/* 3.2 Boot flag */
.=CONFIG_BOOT_FLAG_POS
	.word CONFIG_BOOT_FLAG_VAL

.=CONFIG_AUX_ENC_FLAG_POS
	.word CONFIG_AUX_ENC_FLAG_VAL

.=CONFIG_SUPPORT_MULTI_PARAM_POS
	.word 0x1

.=CONFIG_PARAM_START_ADDR_POS
	.word reg_list_start-LLOADER_TEXT_BASE

/* 3.6 param item length */
.=CONFIG_PARAM_ITEM_LEN_POS
	.word CONFIG_REG_SIZE

.=CONFIG_BOOT_STORE_ADDR_POS
	.word CONFIG_BOOT_STORE_ADDR_VAL
/*
 * +--------------------------------------------+
 * | 4. Auxiliary Code Area			|
 * +--------------------------------------------+
 */
.=CONFIG_AUXCODE_AREA_POS
_auxcode_start:
	.incbin   "bin/AUXCODE.img"
	.align	8
_auxcode_end:
 .=CONFIG_AUXCODE_AREA_POS + CONFIG_AUXAREA_LEN

/*
 * +--------------------------------------------+
 * | 5. Unchecked Area				|
 * +--------------------------------------------+
 */
_total_boot_area_start:
	.fill CONFIG_UNCHK_SIZE,1,0

/*
 * +--------------------------------------------+
 * | 6. Checked Area (fastboot)			|
 * +--------------------------------------------+
 */
.=CONFIG_CHECKED_AREA_START
_checked_area_start:
	b	reset
/*
 * +--------------------------------------------+
 * | 7. AUX code register files			|
 * +--------------------------------------------+
 */
.align	9
reg_list_start:
boot_reg0:
	.incbin "bin/BOOT_0.reg"
.= boot_reg0 + CONFIG_REG_SIZE

boot_reg1:
	.incbin	"bin/BOOT_1.reg"
.= boot_reg1 + CONFIG_REG_SIZE

boot_reg2:
	.incbin "bin/BOOT_2.reg"
.= boot_reg2 + CONFIG_REG_SIZE
reg_list_end:
.align	8

/*
 * +--------------------------------------------+
 * | 8. ARM Trusted Firmware (first stage)	|
 * +--------------------------------------------+
 */
.=BL1_OFFSET
.incbin "atf/bl1.bin"

.=BL2_OFFSET
.skip BL2_SIZE,0

.=BL31_OFFSET
.skip BL31_SIZE,0

/*
 * +--------------------------------------------+
 * | 9. Reset and debug code			|
 * +--------------------------------------------+
 */
.=BL31_OFFSET + BL31_SIZE
reset:
	isb

	/* loop until the system is ready; REG_SC_GEN1 is 0 at power-on  */
	ldr	r0, =REG_BASE_SCTL
	mov	r1, #1
wait_mux_finish:
	str	r1, [r0, #REG_SC_GEN1]
	ldr	r2, [r0, #REG_SC_GEN1]
	cmp	r1, r2
	bne	wait_mux_finish

	/* init serial */
	bl	uart_init

	/* enable jtag support  */
	mov	r1, #0x130
	ldr	r4, =0xf8a21000
	mov	r5, #0x0ec
	mov	r6, #0x100
0:
	str	r1, [r4, r5]
	add	r5, r5, #0x04
	cmp	r5, r6
	ble	0b

	/* prepare the aarch64 jump address */
	ldr	r4, =REG_PERI_CPU_RVBARADDR
	ldr	r5, =REG_PERI_CPU_RVBARADDR_OFFSET
	ldr	r6, =BL1_BASE
	str	r6, [r4, r5]

	/* prepare the 64bit mode */
	ldr	r4, =REG_PERI_CPU_RVBARADDR
	ldr	r5, =REG_PERI_CPU_AARCH_MODE_OFFSET
	ldr	r0, [r4, r5]
	orr	r0, r0, #CPU_CTRL_AARCH64_MODE
	str	r0, [r4, r5]

	/* enable the system counter */
	ldr	r4, =REG_SYS_CNTFID0
	mov	r5, #0x1
	str	r5, [r4]

	ldr	r4, =REG_SYS_CNTCR
	ldr	r1, =0x103
	str	r1, [r4]

	mov	r0, #'\n'
	bl	printch
	adr	r0, str_version
	bl	printascii
	adr	r0, str_aarch64
	bl	printascii
	ldr	r0, =BL1_BASE
	bl	printhex
	mov	r0, #'\n'
	bl	printch
	mov	r0, #'\n'
	bl	printch

	/* jump */
	dsb
	mrc p15, 0, r2, c12, c0, 2
	orr r2, r2, #0x3
	mcr p15, 0, r2, c12, c0, 2
	isb
	wfi

panic:
	b	panic

str_version:
	.asciz	VERSION_MSG
str_aarch64:
	.asciz	BOOT_MSG

/*
 * +--------------------------------------------+
 * | 8. ARM Trusted Firmware (FIP)	|
 * +--------------------------------------------+
 */
#ifdef RECOVERY
.section .fip,#alloc
.incbin "atf/fip.bin"
#endif

.section .tail,#alloc

_checked_area_reserve:
	.fill 0x10,1,0

.align	8
_checked_area_end:

/*
 * +--------------------------------------------+
 * | 10. Boot Signature				|
 * +--------------------------------------------+
 */
_boot_sign_area:
	.fill 0x100,1,0
_total_boot_area_end:

#ifndef RECOVERY
.section .fip,#alloc
.incbin "atf/fip.bin"
#endif

.global _boot_end
_boot_end:

