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

#define DEBUG_UART_PHYS			0xF8B00000
#define DEBUG_UART_VIRT			0xF0000000

#define UART01x_FR_TXFF			0x020
#define UART01x_FR_BUSY			0x008
#define UART01x_DR			0x00
#define UART01x_FR			0x18

#define UART01x_IBRD			0x24
#define UART01x_FBRD			0x28
#define UART01x_LCR_H			0x2c
#define UART01x_CR			0x30

#define UART01x_CR_DISABLE		0x00
#define UART01x_CR_ENABLE		0x301

@-------------------------------------------
@
@-------------------------------------------
.text
.align	2
.global	uart_init
.type		uart_init, %function
uart_init:
	ldr	a4, uart_base_addr_L0
	mov	a3, #UART01x_CR_DISABLE
	/* disable */
	str	a3, [a4, #UART01x_CR]

	add	a3, a3, #UART01x_FBRD
	str	a3, [a4, #UART01x_IBRD]
	mov	a3, #UART01x_LCR_H
	str	a3, [a4, #UART01x_FBRD]

	/* 8 bits, 1 stop bit, no parity, fifo enabled. */
	mov	a3, #112
	str	a3, [a4, #UART01x_LCR_H]

	/* enable */
	ldr	a3, =UART01x_CR_ENABLE
	str	a3, [a4, #UART01x_CR]
	bx	lr
uart_base_addr_L0:
	.word DEBUG_UART_PHYS

@-------------------------------------------
@  Macros
@-------------------------------------------
	.macro	addruart, rp
	ldr	\rp, =DEBUG_UART_PHYS
	.endm

	.macro	senduart, rd, rx
	strb	\rd, [\rx, #UART01x_DR]
	.endm

	.macro	waituart, rd, rx
1001:	ldr	\rd, [\rx, #UART01x_FR]
	tst	\rd, #UART01x_FR_TXFF
	bne	1001b
	.endm

	.macro	busyuart, rd, rx
1001:	ldr	\rd, [\rx, #UART01x_FR]
	tst	\rd, #UART01x_FR_BUSY
	bne	1001b
	.endm

	.global printascii
	.global printch
	.global	printhex

/* print a 32-bit hexadecimal value passed via r0 */
printhex:
	adr	r2, hexbuf
	add	r3, r2, #8
	mov	r1, #0
	strb	r1, [r3]
1:	and	r1, r0, #15
	mov	r0, r0, lsr #4
	cmp	r1, #10
	addlt	r1, r1, #'0'
	addge	r1, r1, #'a' - 10
	strb	r1, [r3, #-1]!
	teq	r3, r2
	bne	1b
	mov	r0, r2
	b	printascii

hexbuf:	.space 16
	.ltorg

printascii:
	addruart r3
	b	2f
1:	waituart r2, r3
	senduart r1, r3
	busyuart r2, r3
	teq	r1, #'\n'
	moveq	r1, #'\r'
	beq	1b
2:	teq	r0, #0
	ldrneb	r1, [r0], #1
	teqne	r1, #0
	bne	1b
	mov	pc, lr

printch:
	addruart r3
	mov	r1, r0
	mov	r0, #0
	b	1b
