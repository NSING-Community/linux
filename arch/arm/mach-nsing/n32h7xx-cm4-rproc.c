// SPDX-License-Identifier: GPL-2.0-only
/*
 * N32H7xx Cortex-M4 remoteproc.
 *
 * The CM4 hardware starts from its AHB SRAM vector table, unlike Cortex-A
 * Linux images whose ELF entry point is the reset address.  The ELF loader
 * therefore uses p_paddr for all PT_LOAD segments.  Exception fetches use the
 * SRAM base as well, so the vector table must start at N32H7XX_CM4_SRAM_START.
 */

#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <asm/cacheflush.h>

#define N32H7XX_CM4_SRAM_START		0x30000000U
#define N32H7XX_CM4_SRAM_END		0x30058000U
#define N32H7XX_CM4_VECTOR_BASE		N32H7XX_CM4_SRAM_START
#define N32H7XX_CM4_VECTOR_ALIGN	128U
#define N32H7XX_CM4_CORE_VECTOR_SIZE	(16 * sizeof(u32))
#define N32H7XX_CM4_MAX_SEGMENTS	16U

#define N32H7XX_RCC_M4RSTREL		0x174U
#define N32H7XX_RCC_M4RSTREL_EN		BIT(0)
#define N32H7XX_RCC_CFG4		0x168U
#define N32H7XX_RCC_CFG4_DCMU_M7_EN	BIT(5)
#define N32H7XX_RCC_CFG4_DCMU_M4_EN	BIT(4)

#define N32H7XX_DCMU_TXMSG0		0x00U
#define N32H7XX_DCMU_RCVMSG0		0x10U
#define N32H7XX_DCMU_STS		0x20U
#define N32H7XX_DCMU_CTRL		0x24U
#define N32H7XX_DCMU_STS_TEF0		BIT(23)
#define N32H7XX_DCMU_STS_RFF0		BIT(27)
#define N32H7XX_DCMU_STS_GPIF0		BIT(31)
#define N32H7XX_DCMU_CTRL_RFIE0		BIT(27)
#define N32H7XX_DCMU_MESSAGE_SIZE	sizeof(u32)

struct n32h7xx_cm4_region {
	u32 start;
	u32 end;
};

struct n32h7xx_cm4_loaded_segment {
	u32 start;
	u32 size;
};

struct n32h7xx_cm4 {
	struct rproc *rproc;
	void __iomem *boot_vector;
	void __iomem *dcmu;
	void __iomem *rcc;
	struct n32h7xx_cm4_region *regions;
	struct n32h7xx_cm4_loaded_segment loaded_segments[N32H7XX_CM4_MAX_SEGMENTS];
	u32 *heartbeat;
	struct miscdevice msg_misc;
	struct mutex msg_lock;
	wait_queue_head_t msg_wait;
	spinlock_t msg_rx_lock;
	u32 msg_rx_value;
	u32 msg_rx_seq;
	bool msg_rx_pending;
	bool running;
	unsigned int num_regions;
	unsigned int num_loaded_segments;
};

static void n32h7xx_cm4_iounmap(void *data)
{
	iounmap(data);
}

static bool n32h7xx_cm4_region_contains(const struct n32h7xx_cm4 *priv,
					u64 da, size_t len)
{
	unsigned int i;

	for (i = 0; i < priv->num_regions; i++) {
		const struct n32h7xx_cm4_region *region = &priv->regions[i];

		if (da < region->start || da > region->end)
			continue;
		if (len <= region->end - da)
			return true;
	}

	return false;
}

static int n32h7xx_cm4_parse_regions(struct platform_device *pdev,
				     struct n32h7xx_cm4 *priv)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	u32 *cells;
	int count;
	int i;
	int j;
	int ret;

	count = of_property_count_elems_of_size(np, "nsing,load-regions",
						sizeof(u32) * 2);
	if (count <= 0) {
		dev_err(dev, "missing or empty nsing,load-regions\n");
		return count ? count : -EINVAL;
	}

	cells = devm_kcalloc(dev, count * 2, sizeof(*cells), GFP_KERNEL);
	priv->regions = devm_kcalloc(dev, count, sizeof(*priv->regions),
				     GFP_KERNEL);
	if (!cells || !priv->regions)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "nsing,load-regions", cells,
					 count * 2);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		u64 start = cells[i * 2];
		u64 size = cells[i * 2 + 1];
		u64 end = start + size;

		if (!size || start < N32H7XX_CM4_SRAM_START ||
		    end > N32H7XX_CM4_SRAM_END) {
			dev_err(dev, "load region %d is outside CM4 SRAM\n", i);
			return -EINVAL;
		}

		priv->regions[i].start = start;
		priv->regions[i].end = end;
	}

	for (i = 0; i < count; i++) {
		for (j = i + 1; j < count; j++) {
			if (priv->regions[i].start < priv->regions[j].end &&
			    priv->regions[j].start < priv->regions[i].end) {
				dev_err(dev, "load regions %d and %d overlap\n",
					i, j);
				return -EINVAL;
			}
		}
	}

	priv->num_regions = count;
	return 0;
}

static void *n32h7xx_cm4_da_to_va(struct rproc *rproc, u64 da, size_t len,
				  bool *is_iomem)
{
	struct n32h7xx_cm4 *priv = rproc->priv;

	if (!n32h7xx_cm4_region_contains(priv, da, len))
		return NULL;

	if (is_iomem)
		*is_iomem = false;

	/* The ARMv7-M NOMMU kernel runs at physical addresses. */
	return (void *)(uintptr_t)da;
}

static int n32h7xx_cm4_find_vector_table(const struct firmware *fw,
					 u32 *vector_base,
					 const struct elf32_phdr **vector_phdr)
{
	const struct elf32_hdr *ehdr = (const struct elf32_hdr *)fw->data;
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *found = NULL;
	u32 base = U32_MAX;
	unsigned int i;

	phdrs = (const struct elf32_phdr *)(fw->data + ehdr->e_phoff);
	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];

		if (phdr->p_type != PT_LOAD ||
		    phdr->p_filesz < N32H7XX_CM4_CORE_VECTOR_SIZE)
			continue;
		if (phdr->p_paddr < base) {
			base = phdr->p_paddr;
			found = phdr;
		}
	}

	if (!found)
		return -EINVAL;

	*vector_base = base;
	*vector_phdr = found;
	return 0;
}

static bool n32h7xx_cm4_is_executable(const struct elf32_hdr *ehdr, u32 addr)
{
	const struct elf32_phdr *phdrs;
	unsigned int i;

	phdrs = (const struct elf32_phdr *)((const void *)ehdr + ehdr->e_phoff);
	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];
		u64 end = (u64)phdr->p_paddr + phdr->p_memsz;

		if (phdr->p_type == PT_LOAD && (phdr->p_flags & PF_X) &&
		    addr >= phdr->p_paddr && addr < end)
			return true;
	}

	return false;
}

static bool n32h7xx_cm4_is_writable(const struct elf32_hdr *ehdr, u32 addr)
{
	const struct elf32_phdr *phdrs;
	unsigned int i;

	phdrs = (const struct elf32_phdr *)((const void *)ehdr + ehdr->e_phoff);
	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];
		u64 end = (u64)phdr->p_paddr + phdr->p_memsz;

		if (phdr->p_type == PT_LOAD && (phdr->p_flags & PF_W) &&
		    addr >= phdr->p_paddr && addr + sizeof(u32) <= end)
			return true;
	}

	return false;
}

static u64 n32h7xx_cm4_get_boot_addr(struct rproc *rproc,
				     const struct firmware *fw)
{
	const struct elf32_phdr *vector_phdr;
	u32 vector_base;
	int ret;

	ret = n32h7xx_cm4_find_vector_table(fw, &vector_base, &vector_phdr);
	if (ret ||
	    !n32h7xx_cm4_region_contains(rproc->priv, vector_base,
					 2 * sizeof(u32))) {
		dev_err(&rproc->dev, "no valid CM4 vector table in ELF\n");
		return 0;
	}
	if (vector_base & (N32H7XX_CM4_VECTOR_ALIGN - 1)) {
		dev_err(&rproc->dev, "vector table 0x%x is not %u-byte aligned\n",
			vector_base, N32H7XX_CM4_VECTOR_ALIGN);
		return 0;
	}
	if (vector_base != N32H7XX_CM4_VECTOR_BASE) {
		dev_err(&rproc->dev,
			"vector table 0x%x must start at CM4 SRAM base 0x%x\n",
			vector_base, N32H7XX_CM4_VECTOR_BASE);
		return 0;
	}

	return vector_base;
}

static int n32h7xx_cm4_sanity_check(struct rproc *rproc,
				    const struct firmware *fw)
{
	struct n32h7xx_cm4 *priv = rproc->priv;
	const struct elf32_hdr *ehdr;
	const struct elf32_phdr *phdrs;
	const struct elf32_phdr *vector_phdr;
	u32 vector_base;
	u32 initial_sp;
	u32 reset_vector;
	u32 entry;
	unsigned int i;
	unsigned int j;
	unsigned int segment_count = 0;
	int ret;

	if (fw->size < sizeof(*ehdr))
		return -EINVAL;

	ehdr = (const struct elf32_hdr *)fw->data;
	if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) ||
	    ehdr->e_ident[EI_CLASS] != ELFCLASS32 ||
	    ehdr->e_ident[EI_DATA] != ELFDATA2LSB ||
	    ehdr->e_machine != EM_ARM || ehdr->e_type != ET_EXEC ||
	    ehdr->e_ehsize != sizeof(*ehdr) ||
	    ehdr->e_phentsize != sizeof(*phdrs) || !ehdr->e_phnum ||
	    ehdr->e_phnum > N32H7XX_CM4_MAX_SEGMENTS) {
		dev_err(&rproc->dev, "firmware is not an ARM ELF32 executable\n");
		return -EINVAL;
	}

	if (ehdr->e_phoff > fw->size ||
	    ehdr->e_phnum > (fw->size - ehdr->e_phoff) / sizeof(*phdrs)) {
		dev_err(&rproc->dev, "truncated ELF program headers\n");
		return -EINVAL;
	}

	phdrs = (const struct elf32_phdr *)(fw->data + ehdr->e_phoff);
	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];
		u64 da_end;

		if (phdr->p_type != PT_LOAD)
			continue;
		if (!phdr->p_memsz || phdr->p_filesz > phdr->p_memsz ||
		    phdr->p_vaddr != phdr->p_paddr ||
		    phdr->p_offset > fw->size ||
		    phdr->p_filesz > fw->size - phdr->p_offset) {
			dev_err(&rproc->dev, "invalid PT_LOAD segment %u\n", i);
			return -EINVAL;
		}
		if (!n32h7xx_cm4_region_contains(priv, phdr->p_paddr,
						 phdr->p_memsz)) {
			dev_err(&rproc->dev,
				"PT_LOAD %u outside allowed CM4 SRAM: 0x%08x+0x%x\n",
				i, phdr->p_paddr, phdr->p_memsz);
			return -EINVAL;
		}

		da_end = (u64)phdr->p_paddr + phdr->p_memsz;
		for (j = i + 1; j < ehdr->e_phnum; j++) {
			const struct elf32_phdr *other = &phdrs[j];
			u64 other_end;

			if (other->p_type != PT_LOAD || !other->p_memsz)
				continue;
			other_end = (u64)other->p_paddr + other->p_memsz;
			if (phdr->p_paddr < other_end &&
			    other->p_paddr < da_end) {
				dev_err(&rproc->dev,
					"PT_LOAD segments %u and %u overlap\n",
					i, j);
				return -EINVAL;
			}
		}
	}

	entry = ehdr->e_entry & ~BIT(0);
	if (!(ehdr->e_entry & BIT(0)) || !n32h7xx_cm4_is_executable(ehdr, entry)) {
		dev_err(&rproc->dev, "invalid Thumb entry point 0x%08x\n",
			ehdr->e_entry);
		return -EINVAL;
	}

	ret = n32h7xx_cm4_find_vector_table(fw, &vector_base, &vector_phdr);
	if (ret || !n32h7xx_cm4_region_contains(priv, vector_base,
						2 * sizeof(u32))) {
		dev_err(&rproc->dev, "missing aligned CM4 vector table\n");
		return -EINVAL;
	}
	if (vector_base & (N32H7XX_CM4_VECTOR_ALIGN - 1)) {
		dev_err(&rproc->dev, "vector table 0x%x is not %u-byte aligned\n",
			vector_base, N32H7XX_CM4_VECTOR_ALIGN);
		return -EINVAL;
	}
	if (vector_base != N32H7XX_CM4_VECTOR_BASE) {
		dev_err(&rproc->dev,
			"vector table 0x%x must start at CM4 SRAM base 0x%x\n",
			vector_base, N32H7XX_CM4_VECTOR_BASE);
		return -EINVAL;
	}

	memcpy(&initial_sp, fw->data + vector_phdr->p_offset, sizeof(initial_sp));
	memcpy(&reset_vector, fw->data + vector_phdr->p_offset + sizeof(u32),
	       sizeof(reset_vector));
	if (initial_sp < N32H7XX_CM4_SRAM_START ||
	    initial_sp > N32H7XX_CM4_SRAM_END || (initial_sp & 7)) {
		dev_err(&rproc->dev, "invalid initial MSP 0x%08x\n", initial_sp);
		return -EINVAL;
	}
	if (!(reset_vector & BIT(0)) ||
	    !n32h7xx_cm4_region_contains(priv, reset_vector & ~BIT(0), 1) ||
	    !n32h7xx_cm4_is_executable(ehdr, reset_vector & ~BIT(0))) {
		dev_err(&rproc->dev, "invalid reset vector 0x%08x\n",
			reset_vector);
		return -EINVAL;
	}
	if (ehdr->e_entry != reset_vector) {
		dev_err(&rproc->dev,
			"ELF entry 0x%08x does not match reset vector 0x%08x\n",
			ehdr->e_entry, reset_vector);
		return -EINVAL;
	}
	if (priv->heartbeat &&
	    !n32h7xx_cm4_is_writable(ehdr,
				     (u32)(uintptr_t)priv->heartbeat)) {
		dev_err(&rproc->dev,
			"heartbeat address is not in a writable PT_LOAD segment\n");
		return -EINVAL;
	}

	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];

		if (phdr->p_type != PT_LOAD)
			continue;
		priv->loaded_segments[segment_count].start = phdr->p_paddr;
		priv->loaded_segments[segment_count].size = phdr->p_memsz;
		segment_count++;
	}

	priv->num_loaded_segments = segment_count;

	return 0;
}

static int n32h7xx_cm4_load_segments(struct rproc *rproc,
				     const struct firmware *fw)
{
	const struct elf32_hdr *ehdr = (const struct elf32_hdr *)fw->data;
	const struct elf32_phdr *phdrs;
	unsigned int i;

	phdrs = (const struct elf32_phdr *)(fw->data + ehdr->e_phoff);
	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];
		bool is_iomem = false;
		void *ptr;

		if (phdr->p_type != PT_LOAD || !phdr->p_memsz)
			continue;

		ptr = rproc_da_to_va(rproc, phdr->p_paddr, phdr->p_memsz,
				     &is_iomem);
		if (!ptr) {
			dev_err(&rproc->dev,
				"cannot map PT_LOAD %u at 0x%08x+0x%x\n",
				i, phdr->p_paddr, phdr->p_memsz);
			return -EINVAL;
		}

		if (phdr->p_filesz) {
			if (is_iomem)
				memcpy_toio((void __iomem *)ptr,
					    fw->data + phdr->p_offset,
					    phdr->p_filesz);
			else
				memcpy(ptr, fw->data + phdr->p_offset,
				       phdr->p_filesz);
		}
		if (phdr->p_memsz > phdr->p_filesz) {
			if (is_iomem)
				memset_io((void __iomem *)ptr + phdr->p_filesz, 0,
					  phdr->p_memsz - phdr->p_filesz);
			else
				memset((u8 *)ptr + phdr->p_filesz, 0,
				       phdr->p_memsz - phdr->p_filesz);
		}
	}

	return 0;
}

static int n32h7xx_cm4_dcmu_enable_clocks(struct n32h7xx_cm4 *priv)
{
	u32 expected = N32H7XX_RCC_CFG4_DCMU_M7_EN |
		       N32H7XX_RCC_CFG4_DCMU_M4_EN;
	u32 value;

	value = readl(priv->rcc + N32H7XX_RCC_CFG4);
	writel(value | expected, priv->rcc + N32H7XX_RCC_CFG4);
	value = readl(priv->rcc + N32H7XX_RCC_CFG4);
	if ((value & expected) != expected) {
		dev_err(&priv->rproc->dev, "failed to enable DCMU clocks\n");
		return -EIO;
	}

	return 0;
}

static void n32h7xx_cm4_dcmu_clear_pending(struct n32h7xx_cm4 *priv)
{
	/* Match the vendor flow: clear the notification, then consume MSG0. */
	writel(N32H7XX_DCMU_STS_GPIF0, priv->dcmu + N32H7XX_DCMU_STS);
	readl(priv->dcmu + N32H7XX_DCMU_RCVMSG0);
}

static int n32h7xx_cm4_dcmu_start(struct n32h7xx_cm4 *priv)
{
	u32 value;
	int ret;

	mutex_lock(&priv->msg_lock);
	if (priv->running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = n32h7xx_cm4_dcmu_enable_clocks(priv);
	if (ret)
		goto out_unlock;

	n32h7xx_cm4_dcmu_clear_pending(priv);
	value = readl(priv->dcmu + N32H7XX_DCMU_CTRL);
	writel(value | N32H7XX_DCMU_CTRL_RFIE0,
	       priv->dcmu + N32H7XX_DCMU_CTRL);
	WRITE_ONCE(priv->running, true);

out_unlock:
	mutex_unlock(&priv->msg_lock);
	return ret;
}

static void n32h7xx_cm4_dcmu_stop(struct n32h7xx_cm4 *priv)
{
	unsigned long flags;
	u32 value;

	mutex_lock(&priv->msg_lock);
	WRITE_ONCE(priv->running, false);
	value = readl(priv->dcmu + N32H7XX_DCMU_CTRL);
	writel(value & ~N32H7XX_DCMU_CTRL_RFIE0,
	       priv->dcmu + N32H7XX_DCMU_CTRL);
	mutex_unlock(&priv->msg_lock);

	spin_lock_irqsave(&priv->msg_rx_lock, flags);
	priv->msg_rx_pending = false;
	priv->msg_rx_seq++;
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
	wake_up_interruptible_all(&priv->msg_wait);
}

static irqreturn_t n32h7xx_cm4_dcmu_irq(int irq, void *data)
{
	struct n32h7xx_cm4 *priv = data;
	unsigned long flags;
	u32 status;
	u32 message;

	status = readl(priv->dcmu + N32H7XX_DCMU_STS);
	if (!(status & (N32H7XX_DCMU_STS_RFF0 | N32H7XX_DCMU_STS_GPIF0)))
		return IRQ_NONE;

	if (status & N32H7XX_DCMU_STS_GPIF0)
		writel(N32H7XX_DCMU_STS_GPIF0,
		       priv->dcmu + N32H7XX_DCMU_STS);
	if (!(status & N32H7XX_DCMU_STS_RFF0))
		return IRQ_HANDLED;

	message = readl(priv->dcmu + N32H7XX_DCMU_RCVMSG0);
	if (!READ_ONCE(priv->running))
		return IRQ_HANDLED;

	spin_lock_irqsave(&priv->msg_rx_lock, flags);
	if (priv->msg_rx_pending)
		dev_warn_ratelimited(&priv->rproc->dev,
				     "CM4 message overwrote an unread message\n");
	priv->msg_rx_value = message;
	priv->msg_rx_seq++;
	priv->msg_rx_pending = true;
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
	wake_up_interruptible(&priv->msg_wait);

	return IRQ_HANDLED;
}

static int n32h7xx_cm4_msg_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;

	file->private_data = container_of(misc, struct n32h7xx_cm4, msg_misc);
	return 0;
}

static ssize_t n32h7xx_cm4_msg_read(struct file *file, char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct n32h7xx_cm4 *priv = file->private_data;
	unsigned long flags;
	u32 message;
	int ret;

	if (count < N32H7XX_DCMU_MESSAGE_SIZE)
		return -EINVAL;

	if (file->f_flags & O_NONBLOCK) {
		if (!READ_ONCE(priv->running))
			return -ENODEV;
		if (!READ_ONCE(priv->msg_rx_pending))
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(priv->msg_wait,
				!READ_ONCE(priv->running) ||
				READ_ONCE(priv->msg_rx_pending));
		if (ret)
			return ret;
		if (!READ_ONCE(priv->running))
			return -ENODEV;
	}

	spin_lock_irqsave(&priv->msg_rx_lock, flags);
	if (!priv->msg_rx_pending) {
		spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
		return -EAGAIN;
	}
	message = priv->msg_rx_value;
	priv->msg_rx_pending = false;
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);

	if (copy_to_user(buf, &message, sizeof(message)))
		return -EFAULT;

	return sizeof(message);
}

static ssize_t n32h7xx_cm4_msg_write(struct file *file,
				     const char __user *buf, size_t count,
				     loff_t *ppos)
{
	struct n32h7xx_cm4 *priv = file->private_data;
	u32 message;
	u32 status;
	int ret = sizeof(message);

	if (count != N32H7XX_DCMU_MESSAGE_SIZE)
		return -EINVAL;
	if (copy_from_user(&message, buf, sizeof(message)))
		return -EFAULT;

	mutex_lock(&priv->msg_lock);
	if (!priv->running) {
		ret = -ENODEV;
		goto out_unlock;
	}

	status = readl(priv->dcmu + N32H7XX_DCMU_STS);
	if (!(status & N32H7XX_DCMU_STS_TEF0)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	writel(message, priv->dcmu + N32H7XX_DCMU_TXMSG0);

out_unlock:
	mutex_unlock(&priv->msg_lock);
	return ret;
}

static __poll_t n32h7xx_cm4_msg_poll(struct file *file, poll_table *wait)
{
	struct n32h7xx_cm4 *priv = file->private_data;
	unsigned long flags;
	__poll_t events = 0;

	poll_wait(file, &priv->msg_wait, wait);

	spin_lock_irqsave(&priv->msg_rx_lock, flags);
	if (READ_ONCE(priv->running)) {
		if (priv->msg_rx_pending)
			events |= EPOLLIN | EPOLLRDNORM;
		events |= EPOLLOUT | EPOLLWRNORM;
	} else {
		events |= EPOLLERR | EPOLLHUP;
	}
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);

	return events;
}

static const struct file_operations n32h7xx_cm4_msg_fops = {
	.owner = THIS_MODULE,
	.open = n32h7xx_cm4_msg_open,
	.read = n32h7xx_cm4_msg_read,
	.write = n32h7xx_cm4_msg_write,
	.poll = n32h7xx_cm4_msg_poll,
	.llseek = noop_llseek,
};

static int n32h7xx_cm4_prepare(struct rproc *rproc)
{
	struct n32h7xx_cm4 *priv = rproc->priv;
	u32 value;

	value = readl(priv->rcc + N32H7XX_RCC_M4RSTREL);
	if (value & N32H7XX_RCC_M4RSTREL_EN) {
		dev_err(&rproc->dev,
			"CM4 was released by another boot stage; refusing to load\n");
		return -EBUSY;
	}

	return 0;
}

static int n32h7xx_cm4_unprepare(struct rproc *rproc)
{
	struct n32h7xx_cm4 *priv = rproc->priv;

	priv->num_loaded_segments = 0;
	return 0;
}

static int n32h7xx_cm4_start(struct rproc *rproc)
{
	struct n32h7xx_cm4 *priv = rproc->priv;
	void __iomem *vectors;
	u32 initial_sp;
	u32 reset_vector;
	u32 value;
	unsigned int i;
	int ret;

	if (!priv->num_loaded_segments)
		return -EINVAL;

	/* Publish only data the ELF loader actually wrote for this image. */
	for (i = 0; i < priv->num_loaded_segments; i++)
		clean_dcache_area((void *)(uintptr_t)
				  priv->loaded_segments[i].start,
				  priv->loaded_segments[i].size);

	if (!n32h7xx_cm4_region_contains(priv, rproc->bootaddr,
					 2 * sizeof(u32)))
		return -EINVAL;

	vectors = (void __iomem *)(uintptr_t)rproc->bootaddr;
	initial_sp = readl(vectors);
	reset_vector = readl(vectors + sizeof(u32));

	if (initial_sp < N32H7XX_CM4_SRAM_START ||
	    initial_sp > N32H7XX_CM4_SRAM_END || (initial_sp & 7)) {
		dev_err(&rproc->dev, "invalid initial MSP 0x%08x\n", initial_sp);
		return -EINVAL;
	}
	if (!(reset_vector & BIT(0)) ||
	    !n32h7xx_cm4_region_contains(priv, reset_vector & ~BIT(0), 1)) {
		dev_err(&rproc->dev, "invalid reset vector 0x%08x\n",
			reset_vector);
		return -EINVAL;
	}

	writel(rproc->bootaddr, priv->boot_vector);
	readl(priv->boot_vector);

	ret = n32h7xx_cm4_dcmu_start(priv);
	if (ret)
		return ret;

	value = readl(priv->rcc + N32H7XX_RCC_M4RSTREL);
	writel(value | N32H7XX_RCC_M4RSTREL_EN,
	       priv->rcc + N32H7XX_RCC_M4RSTREL);
	value = readl(priv->rcc + N32H7XX_RCC_M4RSTREL);
	if (!(value & N32H7XX_RCC_M4RSTREL_EN)) {
		n32h7xx_cm4_dcmu_stop(priv);
		dev_err(&rproc->dev, "failed to release CM4 from reset\n");
		return -EIO;
	}

	dev_info(&rproc->dev, "started at vectors 0x%llx, entry 0x%08x\n",
		 rproc->bootaddr, reset_vector);
	return 0;
}

static int n32h7xx_cm4_stop(struct rproc *rproc)
{
	struct n32h7xx_cm4 *priv = rproc->priv;
	u32 value;

	n32h7xx_cm4_dcmu_stop(priv);
	value = readl(priv->rcc + N32H7XX_RCC_M4RSTREL);
	writel(value & ~N32H7XX_RCC_M4RSTREL_EN,
	       priv->rcc + N32H7XX_RCC_M4RSTREL);
	value = readl(priv->rcc + N32H7XX_RCC_M4RSTREL);
	priv->num_loaded_segments = 0;

	if (value & N32H7XX_RCC_M4RSTREL_EN) {
		dev_err(&rproc->dev, "failed to hold CM4 in reset\n");
		return -EIO;
	}

	return 0;
}

static ssize_t heartbeat_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct rproc *rproc = container_of(dev, struct rproc, dev);
	struct n32h7xx_cm4 *priv = rproc->priv;
	u32 heartbeat;

	if (!priv->heartbeat)
		return -ENOENT;

	sync_cache_r(priv->heartbeat);
	heartbeat = READ_ONCE(*priv->heartbeat);
	return sysfs_emit(buf, "%u\n", heartbeat);
}

static DEVICE_ATTR_RO(heartbeat);

static void n32h7xx_cm4_remove_heartbeat(void *data)
{
	device_remove_file(data, &dev_attr_heartbeat);
}

static void n32h7xx_cm4_unregister_misc(void *data)
{
	misc_deregister(data);
}

static const struct rproc_ops n32h7xx_cm4_rproc_ops = {
	.prepare = n32h7xx_cm4_prepare,
	.unprepare = n32h7xx_cm4_unprepare,
	.start = n32h7xx_cm4_start,
	.stop = n32h7xx_cm4_stop,
	.load = n32h7xx_cm4_load_segments,
	.da_to_va = n32h7xx_cm4_da_to_va,
	.sanity_check = n32h7xx_cm4_sanity_check,
	.get_boot_addr = n32h7xx_cm4_get_boot_addr,
};

static int n32h7xx_cm4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *rcc_np;
	struct n32h7xx_cm4 *priv;
	struct rproc *rproc;
	const char *firmware_name;
	u32 heartbeat_addr;
	int irq;
	int ret;

	rproc = devm_rproc_alloc(dev, "n32h787-cm4", &n32h7xx_cm4_rproc_ops,
				 "", sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	priv = rproc->priv;
	priv->rproc = rproc;
	mutex_init(&priv->msg_lock);
	spin_lock_init(&priv->msg_rx_lock);
	init_waitqueue_head(&priv->msg_wait);
	rproc->auto_boot = false;

	ret = of_property_read_string(np, "firmware-name", &firmware_name);
	if (!ret) {
		if (!*firmware_name)
			return dev_err_probe(dev, -EINVAL,
					     "empty firmware-name\n");
		ret = rproc_set_firmware(rproc, firmware_name);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to set firmware-name\n");
	} else if (ret != -EINVAL) {
		return dev_err_probe(dev, ret, "invalid firmware-name\n");
	}

	priv->boot_vector =
		devm_platform_ioremap_resource_byname(pdev, "boot-vector");
	if (IS_ERR(priv->boot_vector))
		return PTR_ERR(priv->boot_vector);

	priv->dcmu = devm_platform_ioremap_resource_byname(pdev, "dcmu");
	if (IS_ERR(priv->dcmu))
		return PTR_ERR(priv->dcmu);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, n32h7xx_cm4_dcmu_irq, 0,
			       "n32h787-cm4-dcmu", priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request DCMU IRQ\n");

	rcc_np = of_parse_phandle(np, "nsing,rcc", 0);
	if (!rcc_np)
		return dev_err_probe(dev, -EINVAL, "missing nsing,rcc\n");
	priv->rcc = of_iomap(rcc_np, 0);
	of_node_put(rcc_np);
	if (!priv->rcc)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, n32h7xx_cm4_iounmap, priv->rcc);
	if (ret)
		return ret;

	ret = n32h7xx_cm4_parse_regions(pdev, priv);
	if (ret)
		return ret;

	ret = of_property_read_u32(np, "nsing,heartbeat-address",
				   &heartbeat_addr);
	if (!ret) {
		if ((heartbeat_addr & (sizeof(u32) - 1)) ||
		    !n32h7xx_cm4_region_contains(priv, heartbeat_addr,
						 sizeof(u32)))
			return dev_err_probe(dev, -EINVAL,
					     "invalid heartbeat address\n");
		priv->heartbeat = (u32 *)(uintptr_t)heartbeat_addr;
	} else if (ret != -EINVAL) {
		return dev_err_probe(dev, ret, "invalid heartbeat property\n");
	}

	ret = devm_rproc_add(dev, rproc);
	if (ret)
		return ret;

	priv->msg_misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "cm4-msg0",
		.fops = &n32h7xx_cm4_msg_fops,
		.parent = dev,
		.mode = 0600,
	};
	ret = misc_register(&priv->msg_misc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register CM4 message device\n");
	ret = devm_add_action_or_reset(dev, n32h7xx_cm4_unregister_misc,
				       &priv->msg_misc);
	if (ret)
		return ret;

	if (priv->heartbeat) {
		ret = device_create_file(&rproc->dev, &dev_attr_heartbeat);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(dev,
					       n32h7xx_cm4_remove_heartbeat,
					       &rproc->dev);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, rproc);
	return 0;
}

static const struct of_device_id n32h7xx_cm4_of_match[] = {
	{ .compatible = "nsing,n32h787-cm4" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_cm4_of_match);

static struct platform_driver n32h7xx_cm4_driver = {
	.probe = n32h7xx_cm4_probe,
	.driver = {
		.name = "n32h7xx-cm4-rproc",
		.of_match_table = n32h7xx_cm4_of_match,
	},
};
module_platform_driver(n32h7xx_cm4_driver);

MODULE_DESCRIPTION("NSING N32H7xx Cortex-M4 remoteproc driver");
MODULE_LICENSE("GPL");
