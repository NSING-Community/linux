// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Uwe Kleine-Koenig for Pengutronix
 */
#include <linux/io.h>
#include <linux/init.h>
#include <linux/ptrace.h>
#include <linux/reboot.h>
#include <linux/sched/debug.h>
#include <asm/barrier.h>
#include <asm/elf.h>
#include <asm/v7m.h>

#ifdef CONFIG_CPU_V7M_FPU
void __init v7m_fpu_init(void)
{
	u32 mvfr0 = readl(BASEADDR_V7M_SCB + V7M_SCB_MVFR0);
	u32 mvfr1 = readl(BASEADDR_V7M_SCB + V7M_SCB_MVFR1);
	u32 cpacr = readl(BASEADDR_V7M_SCB + V7M_SCB_CPACR);

	if ((mvfr0 & V7M_MVFR0_FP_MASK) != V7M_MVFR0_D16_SP_DP ||
	    (cpacr & V7M_SCB_CPACR_CP10_CP11_FULL) !=
	    V7M_SCB_CPACR_CP10_CP11_FULL)
		panic("CPU_V7M_FPU requires accessible D16 single/double precision FPU (MVFR0=%#x CPACR=%#x)",
		      mvfr0, cpacr);

	elf_hwcap |= HWCAP_VFP | HWCAP_VFPv3 | HWCAP_VFPv3D16;
	if (((mvfr1 >> 28) & 0xf) == 1)
		elf_hwcap |= HWCAP_VFPv4;
}
#endif

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
