#define SH_QSPI_SPSR		0x03
#define SH_QSPI_SPDR		0x04
#define SH_QSPI_SPBFCR		0x18

#define SPSR_SPRFF	0x80
#define SPSR_SPTEF	0x20
#define SPBFCR_TXTRG	0x30
#define SPBFCR_RXTRG	0x07

/*
 * r0, controller base address
 * r1, tx buffer
 * r2, rx buffer
 * r3, xfer len, non-zero
 */
.syntax unified
.arm
.text

.macro wait_for_spsr, spsrbit
	1:	ldrb	r4, [r0, #SH_QSPI_SPSR]
		tst	r4, \spsrbit
		beq	1b
.endm

.global _start
_start:

prepcopy:
	ldr	r4, [r0, #SH_QSPI_SPBFCR]
	orr	r4, #(SPBFCR_TXTRG | SPBFCR_RXTRG)
	mov	r5, #32
	cmp	r3, #32

	biclt	r4, #(SPBFCR_TXTRG | SPBFCR_RXTRG)
	movlt	r5, #1

copy:
	str	r4, [r0, #SH_QSPI_SPBFCR]

	wait_for_spsr SPSR_SPTEF

	mov	r6, r5
	mov	r4, #0
	cmp	r1, #0
	beq	3f

2:	ldrb	r4, [r1], #1
	strb	r4, [r0, #SH_QSPI_SPDR]
	subs	r6, #1
	bne	2b
	b	4f

3:	strb	r4, [r0, #SH_QSPI_SPDR]
	subs	r6, #1
	bne	3b

4:	wait_for_spsr SPSR_SPRFF

	mov	r6, r5
	cmp	r2, #0
	beq	6f

5:	ldrb	r4, [r0, #SH_QSPI_SPDR]
	strb	r4, [r2], #1
	subs	r6, #1
	bne	5b
	b	7f

6:	ldrb	r4, [r0, #SH_QSPI_SPDR]
	subs	r6, #1
	bne	6b

7:	subs	r3, r5
	bne	prepcopy

exit:	bkpt	#0
