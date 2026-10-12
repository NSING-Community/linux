// SPDX-License-Identifier: GPL-2.0-only
/*
 * ARMv7-M fatal exception handling for the NSING N32H7xx (Cortex-M7).
 *
 * The ARMv7-M vector table routes Reset, NMI, HardFault, MemManage, BusFault
 * and UsageFault at __invalid_entry in arch/arm/kernel/entry-v7m.S.  That
 * entry used to print a single line and then spin forever with interrupts
 * masked, which meant any user-space fault killed the whole board and hid
 * everything needed to find the real bug.
 *
 * This handler decodes the SCB fault status registers, reports the failing
 * access together with the task that made it, and:
 *
 *   - for a fault raised while the CPU was executing user code, queues the
 *     matching signal (SIGSEGV/SIGILL/SIGFPE) and returns non-zero so that
 *     __invalid_entry retires the fault and defers signal handling to PendSV.
 *     The offending task dies and the system keeps running.
 *   - for a fault raised while the kernel was running, reports registers
 *     and panics without scheduling from an active fault exception.
 *
 * The register frame, the pre-exception EXC_RETURN and the exception number
 * are supplied by __invalid_entry.
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/ptrace.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/sched/signal.h>
#include <linux/signal.h>

#include <asm/ptrace.h>
#include <asm/v7m.h>

/* Offsets from BASEADDR_V7M_SCB (0xe000ed00). */
#define N32H7XX_SCB_CFSR		0x28
#define N32H7XX_SCB_HFSR		0x2c
#define N32H7XX_SCB_DFSR		0x30
#define N32H7XX_SCB_MMFAR		0x34
#define N32H7XX_SCB_BFAR		0x38
#define N32H7XX_SCB_AFSR		0x3c
#define N32H7XX_SCB_MPU_CTRL		0x94

#define N32H7XX_SCB_CFSR_MMARVALID	BIT(7)
#define N32H7XX_SCB_CFSR_BFARVALID	BIT(15)
#define N32H7XX_SCB_CFSR_IACCVIOL	BIT(0)
#define N32H7XX_SCB_CFSR_DACCVIOL	BIT(1)

struct n32h7xx_fault_bit {
	u32 mask;
	const char *text;
};

static const struct n32h7xx_fault_bit n32h7xx_mm_faults[] = {
	{ BIT(0),  "IACCVIOL: instruction access violation" },
	{ BIT(1),  "DACCVIOL: data access violation" },
	{ BIT(3),  "MUNSTKERR: fault while unstacking on exception return" },
	{ BIT(4),  "MSTKERR: fault while stacking on exception entry" },
	{ BIT(5),  "MLSPERR: fault during lazy floating point state save" },
	{ BIT(7),  "MMARVALID: MMFAR holds the faulting address" },
};

static const struct n32h7xx_fault_bit n32h7xx_bus_faults[] = {
	{ BIT(8),  "IBUSERR: instruction bus error" },
	{ BIT(9),  "PRECISERR: precise data bus error" },
	{ BIT(10), "IMPRECISERR: imprecise data bus error (write buffer)" },
	{ BIT(12), "STKERR: bus fault while stacking on exception entry" },
	{ BIT(11), "UNSTKERR: bus fault while unstacking on exception return" },
	{ BIT(13), "LSPERR: bus fault during lazy floating point state save" },
	{ BIT(15), "BFARVALID: BFAR holds the faulting address" },
};

static const struct n32h7xx_fault_bit n32h7xx_usage_faults[] = {
	{ BIT(16), "UNDEFINSTR: undefined instruction" },
	{ BIT(17), "INVSTATE: invalid EPSR/T bit state" },
	{ BIT(18), "INVPC: invalid PC on exception return" },
	{ BIT(19), "NOCP: coprocessor not present or disabled" },
	{ BIT(24), "UNALIGNED: unaligned access" },
	{ BIT(25), "DIVBYZERO: divide by zero" },
};

static const struct n32h7xx_fault_bit n32h7xx_hard_faults[] = {
	{ BIT(1),  "VECTTBL: fault while reading the vector table" },
	{ BIT(30), "FORCED: fault escalated from a configurable exception" },
	{ BIT(31), "DEBUGEVT: debug event" },
};

static void n32h7xx_fault_decode(const struct n32h7xx_fault_bit *bits,
				 unsigned int count, u32 value)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		if (value & bits[i].mask)
			pr_emerg("n32h7xx:   %s\n", bits[i].text);
}

static const char *n32h7xx_exception_name(unsigned long ipsr)
{
	switch (ipsr) {
	case 1:  return "Reset";
	case 2:  return "NMI";
	case 3:  return "HardFault";
	case 4:  return "MemManage";
	case 5:  return "BusFault";
	case 6:  return "UsageFault";
	case 11: return "SVCall";
	case 12: return "DebugMonitor";
	case 14: return "PendSV";
	case 15: return "SysTick";
	default: return "external interrupt";
	}
}

int v7m_handle_fault(struct pt_regs *regs, unsigned long exc_return,
		     unsigned long ipsr)
{
	void __iomem *scb = BASEADDR_V7M_SCB;
	unsigned long fault_addr = 0;
	bool have_fault_addr = false, valid_exc_return;
	bool from_user, from_exc_return, from_xpsr;
	u32 cfsr, hfsr, dfsr, mmfar, bfar, afsr, shcsr, mpu_ctrl;
	int sig = SIGSEGV, code = SEGV_MAPERR;

	cfsr = readl(scb + N32H7XX_SCB_CFSR);
	hfsr = readl(scb + N32H7XX_SCB_HFSR);
	dfsr = readl(scb + N32H7XX_SCB_DFSR);
	mmfar = readl(scb + N32H7XX_SCB_MMFAR);
	bfar = readl(scb + N32H7XX_SCB_BFAR);
	afsr = readl(scb + N32H7XX_SCB_AFSR);
	shcsr = readl(scb + V7M_SCB_SHCSR);
	mpu_ctrl = readl(scb + N32H7XX_SCB_MPU_CTRL);

	if (cfsr & N32H7XX_SCB_CFSR_BFARVALID) {
		fault_addr = bfar;
		have_fault_addr = true;
	} else if (cfsr & N32H7XX_SCB_CFSR_MMARVALID) {
		fault_addr = mmfar;
		have_fault_addr = true;
	}

	/*
	 * EXC_RETURN bit 2 selects the stack the interrupted context used:
	 * set means the process stack (thread mode, i.e. user code), clear
	 * means the main stack.  Only basic-frame EXC_RETURN values are
	 * supported by entry-v7m.S; an invalid value cannot be recovered by
	 * trusting the stacked xPSR instead.
	 *
	 * Both sources are required to agree before a fault is treated as
	 * recoverable; a bogus EXC_RETURN must not turn a kernel fault into a
	 * spurious signal.
	 */
	valid_exc_return = exc_return == 0xfffffff1UL ||
			   exc_return == 0xfffffff9UL ||
			   exc_return == 0xfffffffdUL;
	from_xpsr = (regs->ARM_cpsr & 0x1ff) == 0;
	from_exc_return = valid_exc_return && (exc_return & 0x4);
	from_user = from_exc_return && from_xpsr;

	pr_emerg("\n");
	pr_emerg("n32h7xx: ARMv7-M %s (exception %lu) from %s mode\n",
		 n32h7xx_exception_name(ipsr), ipsr,
		 from_user ? "user" : "kernel");
	pr_emerg("n32h7xx: CFSR=%#010x HFSR=%#010x DFSR=%#010x SHCSR=%#010x\n",
		 cfsr, hfsr, dfsr, shcsr);
	pr_emerg("n32h7xx: MMFAR=%#010x BFAR=%#010x AFSR=%#010x MPU_CTRL=%#010x\n",
		 mmfar, bfar, afsr, mpu_ctrl);
	pr_emerg("n32h7xx: EXC_RETURN=%#010lx stacked_xPSR=%#010lx\n",
		 exc_return, regs->ARM_cpsr);
	pr_emerg("n32h7xx: task=%s pid=%d pc=%#lx lr=%#lx sp=%#lx\n",
		 current->comm, task_pid_nr(current), instruction_pointer(regs),
		 regs->ARM_lr, regs->ARM_sp);

	if (have_fault_addr)
		pr_emerg("n32h7xx: faulting address %#010lx\n", fault_addr);
	if (from_exc_return != from_xpsr)
		pr_emerg("n32h7xx: mode mismatch (EXC_RETURN says %s, xPSR says %s)\n",
			 from_exc_return ? "user" : "kernel",
			 from_xpsr ? "user" : "kernel");

	if (cfsr & 0x000000ff)
		n32h7xx_fault_decode(n32h7xx_mm_faults,
				     ARRAY_SIZE(n32h7xx_mm_faults), cfsr);
	if (cfsr & 0x0000ff00)
		n32h7xx_fault_decode(n32h7xx_bus_faults,
				     ARRAY_SIZE(n32h7xx_bus_faults), cfsr);
	if (cfsr & 0xffff0000)
		n32h7xx_fault_decode(n32h7xx_usage_faults,
				     ARRAY_SIZE(n32h7xx_usage_faults), cfsr);
	if (hfsr)
		n32h7xx_fault_decode(n32h7xx_hard_faults,
				     ARRAY_SIZE(n32h7xx_hard_faults), hfsr);

	/* Sticky status bits are write-one-to-clear; latch is already saved. */
	writel(cfsr, scb + N32H7XX_SCB_CFSR);
	writel(hfsr, scb + N32H7XX_SCB_HFSR);

	if (!valid_exc_return) {
		pr_emerg("n32h7xx: unsupported EXC_RETURN, cannot retire fault\n");
		show_regs(regs);
		return 0;
	}

	if (!from_user) {
		/* A fatal exception is still active: do_exit() cannot schedule. */
		show_regs(regs);
		panic("n32h7xx ARMv7-M kernel exception");
	}

	/* These faults do not provide a trustworthy resumable user frame. */
	if (ipsr < 3 || ipsr > 6 ||
	    (cfsr & (BIT(3) | BIT(4) | BIT(5) | BIT(10) | BIT(11) |
		     BIT(12) | BIT(13) | BIT(18))) ||
	    (hfsr & (BIT(1) | BIT(31)))) {
		show_regs(regs);
		return 0;
	}

	if (cfsr & 0xffff0000) {
		/* UsageFault: arithmetic/instruction encoding problems. */
		if (cfsr & BIT(24)) {
			sig = SIGBUS;
			code = BUS_ADRALN;
		} else if (cfsr & BIT(25)) {
			sig = SIGFPE;
			code = FPE_INTDIV;
		} else {
			sig = SIGILL;
			code = cfsr & BIT(19) ? ILL_COPROC : ILL_ILLOPC;
		}
	} else if (cfsr & (N32H7XX_SCB_CFSR_IACCVIOL |
			   N32H7XX_SCB_CFSR_DACCVIOL)) {
		/* Access violation, typically the MPU or a privileged access. */
		sig = SIGSEGV;
		code = SEGV_ACCERR;
	} else {
		sig = SIGSEGV;
		code = SEGV_MAPERR;
	}

	if (!have_fault_addr && (sig == SIGILL || sig == SIGFPE)) {
		fault_addr = instruction_pointer(regs);
		have_fault_addr = true;
	}

	pr_emerg("n32h7xx: killing %s (pid %d) with signal %d, code %d\n",
		 current->comm, task_pid_nr(current), sig, code);
	show_regs(regs);
	force_sig_fault(sig, code,
			have_fault_addr ? (void __user *)fault_addr : NULL);

	return 1;
}
