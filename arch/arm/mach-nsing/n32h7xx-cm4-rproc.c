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
#include <linux/crc32.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
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

#include <linux/soc/nsing/n32h787-cm4.h>

#define N32H7XX_CM4_SRAM_START		0x30000000U
#define N32H7XX_CM4_SRAM_END		0x30058000U
#define N32H7XX_CM4_VECTOR_BASE		N32H7XX_CM4_SRAM_START
#define N32H7XX_CM4_VECTOR_ALIGN	128U
#define N32H7XX_CM4_CORE_VECTOR_SIZE	(16 * sizeof(u32))
#define N32H7XX_CM4_MAX_SEGMENTS	16U
#define N32H7XX_EF_ARM_ABI_FLOAT_HARD	0x400U

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
#define N32H7XX_CM4_RX_QUEUE_DEPTH	32U

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
	struct n32h787_cm4_control *control;
	struct n32h787_cm4_message rx_queue[N32H7XX_CM4_RX_QUEUE_DEPTH];
	unsigned int rx_queue_head;
	unsigned int rx_queue_tail;
	u32 c2h_tail;
	u32 msg_rx_dropped;
	u32 msg_rx_seq;
	bool running;
	unsigned int num_regions;
	unsigned int num_loaded_segments;
};

static_assert(sizeof(struct n32h787_cm4_control) == 32);
static_assert(sizeof(struct n32h787_cm4_message) ==
	      N32H787_CM4_MESSAGE_SIZE);
static_assert(sizeof(struct n32h787_cm4_image_meta) == 64);

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

static bool n32h7xx_cm4_meta_name_valid(const char *name)
{
	unsigned int i;

	for (i = 0; i < N32H787_CM4_IMAGE_NAME_SIZE; i++) {
		char c = name[i];

		if (!c)
			return i > 0;
		if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
		      (c >= '0' && c <= '9') || c == '.' || c == '_' ||
		      c == '-'))
			return false;
	}

	return false;
}

static int n32h7xx_cm4_validate_image_meta(struct rproc *rproc,
					   const struct elf32_hdr *ehdr,
					   const struct firmware *fw,
					   u32 entry, u32 initial_sp)
{
	const struct n32h787_cm4_image_meta *found = NULL;
	struct n32h787_cm4_image_meta meta;
	const struct elf32_phdr *phdrs;
	unsigned int load_count = 0;
	unsigned int note_count = 0;
	u64 image_bytes = 0;
	u64 mem_bytes = 0;
	u32 crc = ~0U;
	u32 expected_crc;
	unsigned int i;

	phdrs = (const struct elf32_phdr *)(fw->data + ehdr->e_phoff);
	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];

		if (phdr->p_type == PT_NOTE) {
			note_count++;
			if (phdr->p_filesz != sizeof(meta) ||
			    (phdr->p_memsz && phdr->p_memsz != sizeof(meta)) ||
			    phdr->p_offset > fw->size ||
			    phdr->p_filesz > fw->size - phdr->p_offset)
				continue;
			found = (const void *)(fw->data + phdr->p_offset);
			continue;
		}
		if (phdr->p_type != PT_LOAD)
			continue;
		load_count++;
		image_bytes += phdr->p_filesz;
		mem_bytes += phdr->p_memsz;
	}

	if (note_count != 1 || !found) {
		dev_err(&rproc->dev,
			"expected one valid .n32cm4 PT_NOTE metadata segment\n");
		return -EINVAL;
	}
	if (image_bytes > U32_MAX || mem_bytes > U32_MAX) {
		dev_err(&rproc->dev, "CM4 image or memory budget exceeds 32 bits\n");
		return -EINVAL;
	}

	memcpy(&meta, found, sizeof(meta));
	if (meta.magic != N32H787_CM4_IMAGE_MAGIC ||
	    meta.version != N32H787_CM4_IMAGE_META_VERSION ||
	    meta.size != sizeof(meta)) {
		dev_err(&rproc->dev, "invalid .n32cm4 magic/version/size\n");
		return -EINVAL;
	}
	if (!n32h7xx_cm4_meta_name_valid(meta.name)) {
		dev_err(&rproc->dev, "invalid .n32cm4 task name\n");
		return -EINVAL;
	}
	if (meta.abi_version != N32H787_CM4_ABI_VERSION ||
	    !meta.task_version) {
		dev_err(&rproc->dev, "unsupported .n32cm4 ABI or task version\n");
		return -EINVAL;
	}
	if (meta.flags & ~N32H787_CM4_IMAGE_FLAG_SUPPORTED) {
		dev_err(&rproc->dev, "unknown .n32cm4 flags 0x%08x\n",
			meta.flags & ~N32H787_CM4_IMAGE_FLAG_SUPPORTED);
		return -EINVAL;
	}
	if (!!(meta.flags & N32H787_CM4_IMAGE_FLAG_HARD_FLOAT) !=
	    !!(ehdr->e_flags & N32H7XX_EF_ARM_ABI_FLOAT_HARD)) {
		dev_err(&rproc->dev, ".n32cm4 float ABI does not match ELF\n");
		return -EINVAL;
	}
	if (meta.entry != entry || meta.initial_sp != initial_sp) {
		dev_err(&rproc->dev,
			".n32cm4 entry or initial stack does not match vectors\n");
		return -EINVAL;
	}
	if (meta.image_bytes != image_bytes ||
	    meta.mem_bytes != mem_bytes ||
	    meta.segment_count != load_count) {
		dev_err(&rproc->dev,
			".n32cm4 image/memory/segment budget mismatch\n");
		return -EINVAL;
	}
	if (meta.capabilities & ~N32H787_CM4_CAP_SUPPORTED) {
		dev_err(&rproc->dev, "unknown .n32cm4 capabilities 0x%08x\n",
			meta.capabilities & ~N32H787_CM4_CAP_SUPPORTED);
		return -EINVAL;
	}

	for (i = 0; i < ehdr->e_phnum; i++) {
		const struct elf32_phdr *phdr = &phdrs[i];
		size_t remaining;
		static const u8 zeros[256];

		if (phdr->p_type != PT_LOAD)
			continue;
		crc = crc32_le(crc, fw->data + phdr->p_offset, phdr->p_filesz);
		remaining = phdr->p_memsz - phdr->p_filesz;
		while (remaining) {
			size_t chunk = min_t(size_t, remaining, sizeof(zeros));

			crc = crc32_le(crc, zeros, chunk);
			remaining -= chunk;
		}
	}
	expected_crc = crc ^ ~0U;
	if (meta.load_crc32 != expected_crc) {
		dev_err(&rproc->dev,
			".n32cm4 load CRC 0x%08x does not match 0x%08x\n",
			meta.load_crc32, expected_crc);
		return -EINVAL;
	}

	dev_info(&rproc->dev,
		 "CM4 image %.16s v%u: image=%llu mem=%llu segments=%u caps=0x%x\n",
		 meta.name, meta.task_version, image_bytes, mem_bytes,
		 meta.segment_count, meta.capabilities);
	return 0;
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
		if (phdr->p_paddr < N32H787_CM4_ABI_END &&
		    N32H787_CM4_ABI_BASE < phdr->p_paddr + phdr->p_memsz) {
			dev_err(&rproc->dev,
				"PT_LOAD %u overlaps the CM4 platform ABI\n", i);
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

	ret = n32h7xx_cm4_validate_image_meta(rproc, ehdr, fw, ehdr->e_entry,
					      initial_sp);
	if (ret)
		return ret;

	if (priv->heartbeat &&
	    !n32h7xx_cm4_is_writable(ehdr,
				     (u32)(uintptr_t)priv->heartbeat) &&
	    !((u32)(uintptr_t)priv->heartbeat >= N32H787_CM4_ABI_BASE &&
	      (u32)(uintptr_t)priv->heartbeat + sizeof(u32) <=
			N32H787_CM4_ABI_END)) {
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

static void n32h7xx_cm4_ipc_reset(struct n32h7xx_cm4 *priv)
{
	struct n32h787_cm4_control *control =
		(struct n32h787_cm4_control *)(uintptr_t)N32H787_CM4_ABI_BASE;
	unsigned long flags;

	/* Discard stale cached lines before reinitializing the shared region. */
	clean_dcache_area(control, N32H787_CM4_ABI_SIZE);
	memset(control, 0, N32H787_CM4_ABI_SIZE);
	priv->control = control;
	WRITE_ONCE(priv->control->magic, N32H787_CM4_ABI_MAGIC);
	WRITE_ONCE(priv->control->version, N32H787_CM4_ABI_VERSION);
	WRITE_ONCE(priv->control->size, sizeof(*priv->control));
	WRITE_ONCE(priv->control->state, N32H787_CM4_STATE_OFFLINE);
	WRITE_ONCE(priv->c2h_tail, 0);
	clean_dcache_area((void *)(uintptr_t)N32H787_CM4_ABI_BASE,
			  N32H787_CM4_ABI_SIZE);

	spin_lock_irqsave(&priv->msg_rx_lock, flags);
	priv->rx_queue_head = 0;
	priv->rx_queue_tail = 0;
	priv->msg_rx_dropped = 0;
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
}

static void n32h7xx_cm4_drain_c2h(struct n32h7xx_cm4 *priv)
{
	struct n32h787_cm4_message *slots =
		(struct n32h787_cm4_message *)(uintptr_t)N32H787_CM4_C2H_DATA;
	volatile u32 *headp = (u32 *)(uintptr_t)N32H787_CM4_C2H_HEAD;
	volatile u32 *tailp = (u32 *)(uintptr_t)N32H787_CM4_C2H_TAIL;
	unsigned long flags;
	u32 head;
	u32 tail = priv->c2h_tail;
	bool queued = false;

	sync_cache_r((void *)headp);
	head = READ_ONCE(*headp);
	if (head - tail > N32H787_CM4_RING_SLOTS) {
		priv->msg_rx_dropped += head - tail - N32H787_CM4_RING_SLOTS;
		tail = head - N32H787_CM4_RING_SLOTS;
	}

	while (tail != head) {
		struct n32h787_cm4_message *slot =
			&slots[tail & N32H787_CM4_RING_MASK];
		struct n32h787_cm4_message message;
		unsigned int next;

		sync_cache_r((void *)slot);
		memcpy(&message, slot, sizeof(message));
		if (message.payload_len > N32H787_CM4_MESSAGE_PAYLOAD) {
			priv->msg_rx_dropped++;
			tail++;
			continue;
		}

		spin_lock_irqsave(&priv->msg_rx_lock, flags);
		next = (priv->rx_queue_head + 1) % N32H7XX_CM4_RX_QUEUE_DEPTH;
		if (next == priv->rx_queue_tail) {
			priv->msg_rx_dropped++;
			spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
		} else {
			priv->rx_queue[priv->rx_queue_head] = message;
			WRITE_ONCE(priv->rx_queue_head, next);
			priv->msg_rx_seq++;
			queued = true;
			spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
		}
		tail++;
	}

	WRITE_ONCE(priv->c2h_tail, tail);
	WRITE_ONCE(*tailp, tail);
	clean_dcache_area((void *)tailp, sizeof(*tailp));
	if (queued)
		wake_up_interruptible(&priv->msg_wait);
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
	priv->rx_queue_head = 0;
	priv->rx_queue_tail = 0;
	priv->msg_rx_seq++;
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
	wake_up_interruptible_all(&priv->msg_wait);
}

static irqreturn_t n32h7xx_cm4_dcmu_irq(int irq, void *data)
{
	struct n32h7xx_cm4 *priv = data;
	u32 status;

	status = readl(priv->dcmu + N32H7XX_DCMU_STS);
	if (!(status & (N32H7XX_DCMU_STS_RFF0 | N32H7XX_DCMU_STS_GPIF0)))
		return IRQ_NONE;

	if (status & N32H7XX_DCMU_STS_GPIF0)
		writel(N32H7XX_DCMU_STS_GPIF0,
		       priv->dcmu + N32H7XX_DCMU_STS);
	if (!(status & N32H7XX_DCMU_STS_RFF0))
		return IRQ_HANDLED;

	(void)readl(priv->dcmu + N32H7XX_DCMU_RCVMSG0);
	if (!READ_ONCE(priv->running))
		return IRQ_HANDLED;

	n32h7xx_cm4_drain_c2h(priv);

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
	struct n32h787_cm4_message message;
	unsigned long flags;
	int ret;

	if (file->f_flags & O_NONBLOCK) {
		if (!READ_ONCE(priv->running))
			return -ENODEV;
		if (READ_ONCE(priv->rx_queue_head) ==
		    READ_ONCE(priv->rx_queue_tail))
			return -EAGAIN;
	} else {
		ret = wait_event_interruptible(priv->msg_wait,
				!READ_ONCE(priv->running) ||
				READ_ONCE(priv->rx_queue_head) !=
				READ_ONCE(priv->rx_queue_tail));
		if (ret)
			return ret;
		if (!READ_ONCE(priv->running))
			return -ENODEV;
	}

	spin_lock_irqsave(&priv->msg_rx_lock, flags);
	if (priv->rx_queue_head == priv->rx_queue_tail) {
		spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
		return -EAGAIN;
	}
	message = priv->rx_queue[priv->rx_queue_tail];
	if (count < message.payload_len) {
		spin_unlock_irqrestore(&priv->msg_rx_lock, flags);
		return -EMSGSIZE;
	}
	priv->rx_queue_tail = (priv->rx_queue_tail + 1) %
			      N32H7XX_CM4_RX_QUEUE_DEPTH;
	spin_unlock_irqrestore(&priv->msg_rx_lock, flags);

	if (copy_to_user(buf, message.payload, message.payload_len))
		return -EFAULT;

	return message.payload_len;
}

static ssize_t n32h7xx_cm4_msg_write(struct file *file,
				     const char __user *buf, size_t count,
				     loff_t *ppos)
{
	struct n32h7xx_cm4 *priv = file->private_data;
	struct n32h787_cm4_message *slot;
	volatile u32 *headp = (u32 *)(uintptr_t)N32H787_CM4_H2C_HEAD;
	volatile u32 *tailp = (u32 *)(uintptr_t)N32H787_CM4_H2C_TAIL;
	u8 payload[N32H787_CM4_MESSAGE_PAYLOAD];
	u32 head;
	u32 tail;
	u32 status;
	int ret = count;

	if (!count || count > N32H787_CM4_MESSAGE_PAYLOAD)
		return -EINVAL;
	if (copy_from_user(payload, buf, count))
		return -EFAULT;

	mutex_lock(&priv->msg_lock);
	if (!priv->running) {
		ret = -ENODEV;
		goto out_unlock;
	}

	sync_cache_r((void *)tailp);
	head = READ_ONCE(*headp);
	tail = READ_ONCE(*tailp);
	if (head - tail >= N32H787_CM4_RING_SLOTS) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	status = readl(priv->dcmu + N32H7XX_DCMU_STS);
	if (!(status & N32H7XX_DCMU_STS_TEF0)) {
		ret = -EBUSY;
		goto out_unlock;
	}

	slot = (struct n32h787_cm4_message *)(uintptr_t)
		(N32H787_CM4_H2C_DATA +
		 (head & N32H787_CM4_RING_MASK) * sizeof(*slot));
	memset(slot, 0, sizeof(*slot));
	slot->sequence = head + 1;
	slot->type = N32H787_CM4_MSG_USER;
	slot->payload_len = count;
	memcpy(slot->payload, payload, count);
	clean_dcache_area(slot, sizeof(*slot));

	WRITE_ONCE(*headp, head + 1);
	clean_dcache_area((void *)headp, sizeof(*headp));
	wmb();
	writel(N32H787_CM4_DOORBELL, priv->dcmu + N32H7XX_DCMU_TXMSG0);

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
		if (priv->rx_queue_head != priv->rx_queue_tail)
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

	n32h7xx_cm4_ipc_reset(priv);
	WRITE_ONCE(priv->control->state, N32H787_CM4_STATE_EARLY_BOOT);
	clean_dcache_area(priv->control, sizeof(*priv->control));

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

	if (priv->control) {
		WRITE_ONCE(priv->control->state, N32H787_CM4_STATE_OFFLINE);
		clean_dcache_area(priv->control, sizeof(*priv->control));
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
		if (heartbeat_addr != N32H787_CM4_ABI_BASE +
		    offsetof(struct n32h787_cm4_control, heartbeat))
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
