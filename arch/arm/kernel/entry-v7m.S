/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * linux/arch/arm/kernel/entry-v7m.S
 *
 * Copyright (C) 2008 ARM Ltd.
 *
 * Low-level vector interface routines for the ARMv7-M architecture
 */
#include <asm/page.h>
#include <asm/glue.h>
#include <asm/thread_notify.h>
#include <asm/v7m.h>

#include "entry-header.S"

#ifdef CONFIG_TRACE_IRQFLAGS
#error "CONFIG_TRACE_IRQFLAGS not supported on the current ARMv7M implementation"
#endif

__invalid_entry:
	cpsid	i
	@ Reject unsupported/incomplete frames before dereferencing PSP/MSP.
	mvn	r0, lr
	cmp	r0, #14		@ 0xfffffff1: Handler/MSP
	beq	3f
	cmp	r0, #6		@ 0xfffffff9: Thread/MSP
	beq	3f
	cmp	r0, #2		@ 0xfffffffd: Thread/PSP
	bne	4f
3:	ldr	r0, =BASEADDR_V7M_SCB
	ldr	r0, [r0, #0x28]	@ CFSR
	movw	r1, #0x3838		@ stacking/unstacking/lazy-save faults
	tst	r0, r1
	bne	4f
	v7m_exception_entry
	@ Hand the fault to the platform before doing anything else.  r0 is the
	@ saved register frame, r1 the pre-exception EXC_RETURN (captured before
	@ the call clobbers lr) and r2 the exception number being taken.
	mov	r0, sp
	mov	r1, lr
	mrs	r2, ipsr
	push	{r3, lr}		@ preserve EXC_RETURN across the C handler
	bl	v7m_handle_fault
	pop	{r3, lr}

	@ A zero return means the fault is not recoverable.  The handler has
	@ already reported it, so park the CPU with interrupts disabled.
	cmp	r0, #0
	bne	1f
2:	b	2b

	@ Retire the fault before processing its signal.  Scheduling directly
	@ from a fault would leave that higher-priority exception active while
	@ another task runs in SVC/PendSV context.  PendSV handles the pending
	@ signal after exception return, just as for the IRQ exit path.
1:
	ldr	r1, =BASEADDR_V7M_SCB
	mov	r0, #V7M_SCB_ICSR_PENDSVSET
	str	r0, [r1, V7M_SCB_ICSR]
	ldmia	sp!, {r0-r11}
	add	sp, #PT_REGS_SIZE-S_IP
	cpsie	i
	bx	lr
4:	mov	r0, lr
	mrs	r1, ipsr
	bl	v7m_handle_bad_frame
5:	b	5b
ENDPROC(__invalid_entry)

	.align	2
__irq_entry:
	v7m_exception_entry

	@
	@ Invoke the IRQ handler
	@
	mov	r0, sp
	ldr_this_cpu sp, irq_stack_ptr, r1, r2

	@
	@ If we took the interrupt while running in the kernel, we may already
	@ be using the IRQ stack, so revert to the original value in that case.
	@
	subs	r2, sp, r0		@ SP above bottom of IRQ stack?
	rsbscs	r2, r2, #THREAD_SIZE	@ ... and below the top?
	movcs	sp, r0

	push	{r0, lr}		@ preserve LR and original SP

	@ routine called with r0 = struct pt_regs *
	bl	generic_handle_arch_irq

	pop	{r0, lr}
	mov	sp, r0

	@
	@ Check for any pending work if returning to user
	@
	ldr	r1, =BASEADDR_V7M_SCB
	ldr	r0, [r1, V7M_SCB_ICSR]
	tst	r0, V7M_SCB_ICSR_RETTOBASE
	beq	2f

	get_thread_info tsk
	ldr	r2, [tsk, #TI_FLAGS]
	movs	r2, r2, lsl #16
	beq	2f			@ no work pending
	mov	r0, #V7M_SCB_ICSR_PENDSVSET
	str	r0, [r1, V7M_SCB_ICSR]	@ raise PendSV

2:
	@ registers r0-r3 and r12 are automatically restored on exception
	@ return. r4-r7 were not clobbered in v7m_exception_entry so for
	@ correctness they don't need to be restored. So only r8-r11 must be
	@ restored here. The easiest way to do so is to restore r0-r7, too.
	ldmia	sp!, {r0-r11}
	add	sp, #PT_REGS_SIZE-S_IP
	cpsie	i
	bx	lr
ENDPROC(__irq_entry)

__pendsv_entry:
	v7m_exception_entry

	ldr	r1, =BASEADDR_V7M_SCB
	mov	r0, #V7M_SCB_ICSR_PENDSVCLR
	str	r0, [r1, V7M_SCB_ICSR]	@ clear PendSV

	@ execute the pending work, including reschedule
	get_thread_info tsk
	mov	why, #0
	b	ret_to_user_from_irq
ENDPROC(__pendsv_entry)

/*
 * Register switch for ARMv7-M processors.
 * r0 = previous task_struct, r1 = previous thread_info, r2 = next thread_info
 * previous and next are guaranteed not to be the same.
 */
ENTRY(__switch_to)
	.fnstart
	.cantunwind
#ifdef CONFIG_CPU_V7M_FPU
	/* Manual state management keeps V7-M exception frames basic. */
	add	ip, r1, #TI_V7M_FPU
	vstmia	ip, {d0-d15}
	vmrs	r3, fpscr
	str	r3, [ip, #V7M_FPU_FPSCR]
#endif
	add	ip, r1, #TI_CPU_SAVE
	stmia	ip!, {r4 - r11}		@ Store most regs on stack
	str	sp, [ip], #4
	str	lr, [ip], #4
	mov	r5, r0
	mov	r6, r2			@ Preserve 'next'
	add	r4, r2, #TI_CPU_SAVE
	ldr	r0, =thread_notify_head
	mov	r1, #THREAD_NOTIFY_SWITCH
	bl	atomic_notifier_call_chain
	mov	r0, r5
	mov	r1, r6

#ifdef CONFIG_CPU_V7M_FPU
	add	ip, r6, #TI_V7M_FPU
	vldmia	ip, {d0-d15}
	ldr	r3, [ip, #V7M_FPU_FPSCR]
	vmsr	fpscr, r3
#endif
	ldmia	r4, {r4 - r12, lr}	@ Load all regs saved previously
	set_current r1, r2
	mov	sp, ip
	bx	lr
	.fnend
ENDPROC(__switch_to)

#ifdef CONFIG_CPU_V7M_FPU
ENTRY(v7m_fpu_save_state)
	vstmia	r0, {d0-d15}
	vmrs	r1, fpscr
	str	r1, [r0, #V7M_FPU_FPSCR]
	bx	lr
ENDPROC(v7m_fpu_save_state)

ENTRY(v7m_fpu_restore_state)
	vldmia	r0, {d0-d15}
	ldr	r1, [r0, #V7M_FPU_FPSCR]
	vmsr	fpscr, r1
	bx	lr
ENDPROC(v7m_fpu_restore_state)
#endif

	.data
#if CONFIG_CPU_V7M_NUM_IRQ <= 112
	.align	9
#else
	.align	10
#endif

/*
 * Vector table (Natural alignment need to be ensured)
 */
ENTRY(vector_table)
	.long	0			@ 0 - Reset stack pointer
	.long	__invalid_entry		@ 1 - Reset
	.long	__invalid_entry		@ 2 - NMI
	.long	__invalid_entry		@ 3 - HardFault
	.long	__invalid_entry		@ 4 - MemManage
	.long	__invalid_entry		@ 5 - BusFault
	.long	__invalid_entry		@ 6 - UsageFault
	.long	__invalid_entry		@ 7 - Reserved
	.long	__invalid_entry		@ 8 - Reserved
	.long	__invalid_entry		@ 9 - Reserved
	.long	__invalid_entry		@ 10 - Reserved
	.long	vector_swi		@ 11 - SVCall
	.long	__invalid_entry		@ 12 - Debug Monitor
	.long	__invalid_entry		@ 13 - Reserved
	.long	__pendsv_entry		@ 14 - PendSV
	.long	__invalid_entry		@ 15 - SysTick
	.rept	CONFIG_CPU_V7M_NUM_IRQ
	.long	__irq_entry		@ External Interrupts
	.endr
	.align	2
	.globl	exc_ret
exc_ret:
	.space	4
