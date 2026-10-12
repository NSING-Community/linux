// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Uwe Kleine-Koenig for Pengutronix
 */
#include <linux/io.h>
#include <linux/ptrace.h>
#include <linux/reboot.h>
#include <linux/sched/debug.h>
#include <asm/barrier.h>
#include <asm/v7m.h>


void armv7m_restart(enum reboot_mode mode, const char *cmd)
{
	dsb();
	__raw_writel(V7M_SCB_AIRCR_VECTKEY | V7M_SCB_AIRCR_SYSRESETREQ,
			BASEADDR_V7M_SCB + V7M_SCB_AIRCR);
	dsb();
}

void v7m_handle_bad_frame(unsigned long exc_return, unsigned long ipsr)
{
	pr_emerg("ARMv7-M exception %lu: unusable frame EXC_RETURN=%#lx CFSR=%#x HFSR=%#x\n",
		 ipsr, exc_return, readl(BASEADDR_V7M_SCB + 0x28),
		 readl(BASEADDR_V7M_SCB + 0x2c));
}

/*
 * Hook for the ARMv7-M fatal exception vectors in entry-v7m.S.
 *
 * The vector table routes Reset, NMI, HardFault, MemManage, BusFault and
 * UsageFault at __invalid_entry.  With no platform handler that entry just
 * reports the exception and spins forever with interrupts masked, so a
 * recoverable user-space fault takes the whole machine down.
 *
 * A platform that wants to report the fault properly and/or turn user-space
 * faults into signals overrides this strong.  Returning non-zero asks
 * __invalid_entry to retire the fault and defer its queued signal to PendSV;
 * returning zero keeps the historical "report and park" behaviour.
 *
 * The default only runs on platforms that do not provide their own handler,
 * so it stays deliberately minimal.
 */
int __weak v7m_handle_fault(struct pt_regs *regs, unsigned long exc_return,
			    unsigned long ipsr)
{
	pr_emerg("Unhandled ARMv7-M exception %lu (IPSR=%#lx LR=%#lx)\n",
		 ipsr, regs->ARM_cpsr & 0x1ff, exc_return);
	show_regs(regs);
	return 0;
}
