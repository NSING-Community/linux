// SPDX-License-Identifier: GPL-2.0-only
/*
 * NSING N32H7xx hardware JPEG encoder.
 *
 * The first interface is deliberately small: one synchronous GREY8 encode
 * through /dev/jpeg.  The SGDMA uses its CSR descriptor and the RBC only needs
 * block-line staging buffers, so the reserved DMA pool is never used as a copy
 * of a complete image.
 */

#include <linux/atomic.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/soc/nsing/n32h7xx-jpeg.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <uapi/linux/n32jpeg.h>

#define N32H7XX_JPEG_MAX_WIDTH		4096U
#define N32H7XX_JPEG_MAX_HEIGHT		4096U
#define N32H7XX_JPEG_DIMENSION_ALIGNMENT	8U
#define N32H7XX_JPEG_IO_ALIGNMENT	8U
#define N32H7XX_JPEG_MIN_TIMEOUT_US	500000U
#define N32H7XX_JPEG_MAX_TIMEOUT_US	5000000U
#define N32H7XX_JPEG_P2H_DESC_COUNT	4096U
#define N32H7XX_JPEG_P2H_SPILL_BYTES	SZ_32K

/* Encoder registers. */
#define JPEG_ENC_HT0_DCOL		0x0000
#define JPEG_ENC_HT0_ACOL		0x0040
#define JPEG_ENC_HT0_DCCODE		0x0400
#define JPEG_ENC_HT0_ACCODE		0x0440
#define JPEG_ENC_HT0_EOB		0x0480
#define JPEG_ENC_HT0_ZRL		0x0484
#define JPEG_ENC_QT0			0x2000
#define JPEG_ENC_HFBUFFER		0x4000
#define JPEG_ENC_HEADD			0x4800
#define JPEG_ENC_HUFTAB0E		0x4814
#define JPEG_ENC_HUFTAB1E		0x4818
#define JPEG_ENC_HUFTAB2E		0x481c
#define JPEG_ENC_HUFTAB3E		0x4820
#define JPEG_ENC_FEADD			0x4824
#define JPEG_ENC_CTRL			0x5000
#define JPEG_ENC_HSEL			0x503c
#define JPEG_ENC_PIPESTS		0x5100
#define JPEG_ENC_RICTRL			0x6000

#define JPEG_ENC_CTRL_ERST		BIT(1)
#define JPEG_ENC_CTRL_EN		BIT(0)
#define JPEG_ENC_CTRL_C1_HT0		BIT(6)
#define JPEG_ENC_CTRL_C2_HT0		BIT(8)
#define JPEG_ENC_CTRL_C1_QT0		BIT(14)
#define JPEG_ENC_CTRL_C2_QT0		BIT(16)
#define JPEG_ENC_CTRL_MONO		(JPEG_ENC_CTRL_C1_HT0 | \
					 JPEG_ENC_CTRL_C2_HT0 | \
					 JPEG_ENC_CTRL_C1_QT0 | \
					 JPEG_ENC_CTRL_C2_QT0)
#define JPEG_ENC_HSEL_HT0		BIT(4)
#define JPEG_ENC_HSEL_QT0		BIT(0)

/* Rate-control block. */
#define JPEG_RBC_INIT			0x00
#define JPEG_RBC_EN			0x04
#define JPEG_RBC_SWITCH			0x08
#define JPEG_RBC_FRMW			0x0c
#define JPEG_RBC_FRMH			0x10
#define JPEG_RBC_PFORM			0x18
#define JPEG_RBC_CNAME			0x1c
#define JPEG_RBC_C0SADD			0x20
#define JPEG_RBC_C0EADD			0x24
#define JPEG_RBC_BPS0			0x60
#define JPEG_RBC_BPS12			0x64
#define JPEG_RBC_ROWS0			0x68
#define JPEG_RBC_ROWS12			0x6c
#define JPEG_RBC_HHALF			0x70
#define JPEG_RBC_BLSS0			0x74
#define JPEG_RBC_BLSS12			0x78
#define JPEG_RBC_BPRS0			0x7c
#define JPEG_RBC_BPRS12			0x80

#define JPEG_RBC_INIT_INITF		BIT(4)
#define JPEG_RBC_INIT_INIT		BIT(0)
#define JPEG_RBC_EN_EN			BIT(0)
#define JPEG_RBC_PFORM_MONO		3
#define JPEG_RBC_CNAME_MONO		0x00020101
#define JPEG_RBC_LINES			3

/* SGDMA registers. */
#define JPEG_DMA_CTRL			0x00
#define JPEG_DMA_STS			0x04
#define JPEG_DMA_IE			0x08
#define JPEG_DMA_INTSTS			0x0c
#define JPEG_DMA_AOODT			0x10
#define JPEG_DMA_MBSIZE			0x14
#define JPEG_DMA_SGLPL			0x18
#define JPEG_DMA_SGLPH			0x1c
#define JPEG_DMA_SGLSIZE		0x20
#define JPEG_DMA_SGLHEAD		0x24
#define JPEG_DMA_SGLTAIL		0x28
#define JPEG_DMA_MRADD			0x2c
#define JPEG_DMA_MWADD			0x34
#define JPEG_DMA_DESCF			0x3c
#define JPEG_DMA_DESC_MBSIZE		0x40
#define JPEG_DMA_DESC_MUS		0x44
#define JPEG_DMA_DESC_MBADDL		0x48
#define JPEG_DMA_DESC_MBADDH		0x4c
#define JPEG_DMA_PARACFG		0x5c

#define JPEG_DMA_CTRL_HARDRST		BIT(31)
#define JPEG_DMA_CTRL_MMODE_CSR		BIT(6)
#define JPEG_DMA_CTRL_OPMODE_INCR	BIT(4)
#define JPEG_DMA_CTRL_START		BIT(2)
#define JPEG_DMA_CTRL_EN		BIT(1)
#define JPEG_DMA_CTRL_CLR		BIT(0)

#define JPEG_DMA_DESCF_START		BIT(1)
#define JPEG_DMA_DESCF_END		BIT(2)

#define JPEG_DMA_DESC_START		BIT(1)
#define JPEG_DMA_DESC_END		BIT(2)

#define JPEG_DMA_STS_BUSY		BIT(0)
#define JPEG_DMA_STS_GINT		BIT(1)
#define JPEG_DMA_STS_EOFOUT		BIT(2)
#define JPEG_DMA_STS_EOFIN		BIT(3)
#define JPEG_DMA_STS_OOD		BIT(4)
#define JPEG_DMA_STS_AOOD		BIT(5)
#define JPEG_DMA_STS_EOD		BIT(6)
#define JPEG_DMA_STS_FIFOEMPTY		BIT(7)
#define JPEG_DMA_STS_FIFOFULL		BIT(8)
#define JPEG_DMA_STS_RDESCERR		BIT(10)
#define JPEG_DMA_STS_WDESCERR		BIT(11)
#define JPEG_DMA_STS_RDATAERR		BIT(12)
#define JPEG_DMA_STS_WDATAERR		BIT(13)
#define JPEG_DMA_STS_WRERR		BIT(14)
#define JPEG_DMA_STS_RRERR		BIT(15)

#define JPEG_DMA_STS_FATAL					\
	(JPEG_DMA_STS_RDESCERR | JPEG_DMA_STS_WDESCERR |		\
	 JPEG_DMA_STS_RDATAERR | JPEG_DMA_STS_WDATAERR |		\
	 JPEG_DMA_STS_WRERR | JPEG_DMA_STS_RRERR)

#define JPEG_HEADER_LEN			15U
#define JPEG_DHT_LEN			216U
#define JPEG_FOOTER_LEN			2U
#define JPEG_HF_BUFFER_BYTES		(ALIGN(JPEG_HEADER_LEN, 4) + \
					 JPEG_DHT_LEN + JPEG_FOOTER_LEN)
#define JPEG_HFBUFFER_WORDS		512U

static bool n32h7xx_jpeg_debug;
module_param_named(debug, n32h7xx_jpeg_debug, bool, 0644);
MODULE_PARM_DESC(debug, "Log JPEG DMA status for each completed frame");

struct n32jpeg_huff_entry {
	u16 code;
	u16 offset;
	u8 len;
};

struct n32h7xx_jpeg_dma_desc {
	u32 flags;
	u32 block_size;
	u32 block_used;
	u32 block_addr_lo;
	u32 block_addr_hi;
	u32 reserved;
	u32 link_addr_lo;
	u32 link_addr_hi;
} __aligned(32);

struct n32h7xx_jpeg {
	struct device *dev;
	void __iomem *encoder;
	void __iomem *encoder_rbc;
	void __iomem *encoder_sgdma;
	void __iomem *decoder;
	void __iomem *decoder_brc;
	void __iomem *decoder_sgdma;
	void __iomem *common;
	struct reset_control *encoder_reset;
	struct reset_control *decoder_reset;

	void *scratch;
	dma_addr_t scratch_dma;
	size_t block_line_bytes;

	struct n32h7xx_jpeg_dma_desc *p2h_desc;
	dma_addr_t p2h_desc_dma;
	void *p2h_spill;
	dma_addr_t p2h_spill_dma;

	u32 dc_col[12];
	u32 ac_col[160];
	u32 dc_code[16];
	u32 ac_code[16];
	u32 eob_sym;
	u32 zrl_sym;
	u8 dht[JPEG_DHT_LEN];
	u8 qt[64];
	u32 hf_words[JPEG_HFBUFFER_WORDS];

	struct miscdevice misc;
	struct mutex lock;		/* serializes codec ioctls */
	atomic_t open;
};

static const u8 n32jpeg_dc_bits[16] = {
	0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
};

static const u8 n32jpeg_dc_values[12] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11,
};

static const u8 n32jpeg_ac_bits[16] = {
	0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 125,
};

static const u8 n32jpeg_ac_values[162] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa,
};

static const u8 n32jpeg_luma_quant[64] = {
	16, 11, 10, 16, 24, 40, 51, 61,
	12, 12, 14, 19, 26, 58, 60, 55,
	14, 13, 16, 24, 40, 57, 69, 56,
	14, 17, 22, 29, 51, 87, 80, 62,
	18, 22, 37, 56, 68, 109, 103, 77,
	24, 35, 55, 64, 81, 104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103, 99,
};

static const u8 n32jpeg_zigzag[64] = {
	0, 1, 8, 16, 9, 2, 3, 10,
	17, 24, 32, 25, 18, 11, 4, 5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13, 6, 7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63,
};

static void n32jpeg_be16_put(u8 *p, u16 value)
{
	p[0] = value >> 8;
	p[1] = value;
}

static int n32jpeg_build_canonical(const u8 *counts, const u8 *values,
				   unsigned int value_count,
				   struct n32jpeg_huff_entry *entries,
				   u32 *code_start)
{
	unsigned int index = 0;
	unsigned int length;
	u32 code = 0;
	unsigned int i;

	for (length = 1; length <= 16; length++) {
		u32 first = code;

		/* The encoder expects the unshifted end of the prior code length. */
		code_start[length - 1] = code >> 1;
		for (i = 0; i < counts[length - 1]; i++) {
			if (index >= value_count)
				return -EINVAL;
			entries[index].len = length;
			entries[index].code = code;
			entries[index].offset = code - first;
			code++;
			index++;
		}
		code <<= 1;
	}

	return index == value_count ? 0 : -EINVAL;
}

static size_t n32jpeg_append_dht(u8 *out, u8 table_id, const u8 *counts,
				 const u8 *values, unsigned int value_count)
{
	unsigned int length = 2 + 1 + 16 + value_count;
	u8 *p = out;

	*p++ = 0xff;
	*p++ = 0xc4;
	n32jpeg_be16_put(p, length);
	p += 2;
	*p++ = table_id;
	memcpy(p, counts, 16);
	p += 16;
	memcpy(p, values, value_count);
	p += value_count;

	return p - out;
}

static int n32h7xx_jpeg_build_tables(struct n32h7xx_jpeg *jpeg)
{
	struct n32jpeg_huff_entry dc[ARRAY_SIZE(n32jpeg_dc_values)];
	struct n32jpeg_huff_entry ac[ARRAY_SIZE(n32jpeg_ac_values)];
	u32 eob_code = 0, eob_len = 0, zrl_code = 0, zrl_len = 0;
	size_t dht_len = 0;
	unsigned int i;
	int ret;

	memset(jpeg->dc_col, 0, sizeof(jpeg->dc_col));
	memset(jpeg->ac_col, 0, sizeof(jpeg->ac_col));

	ret = n32jpeg_build_canonical(n32jpeg_dc_bits,
				      n32jpeg_dc_values,
				      ARRAY_SIZE(n32jpeg_dc_values),
				      dc, jpeg->dc_code);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(dc); i++)
		jpeg->dc_col[i] = ((u32)(dc[i].len - 1) << 8) |
				  dc[i].offset;

	ret = n32jpeg_build_canonical(n32jpeg_ac_bits,
				      n32jpeg_ac_values,
				      ARRAY_SIZE(n32jpeg_ac_values),
				      ac, jpeg->ac_code);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ac); i++) {
		u8 value = n32jpeg_ac_values[i];
		unsigned int run = value >> 4;
		unsigned int size = value & 0x0f;
		unsigned int slot;

		if (!size) {
			if (value == 0x00) {
				eob_code = ac[i].code;
				eob_len = ac[i].len;
			} else if (value == 0xf0) {
				zrl_code = ac[i].code;
				zrl_len = ac[i].len;
			}
			continue;
		}
		if (size > 10)
			return -EINVAL;

		slot = (size - 1) * 16 + run;
		if (slot >= ARRAY_SIZE(jpeg->ac_col))
			return -EINVAL;
		jpeg->ac_col[slot] = ((u32)(ac[i].len - 1) << 8) |
				     ac[i].offset;
	}

	if (!eob_len || !zrl_len)
		return -EINVAL;

	jpeg->eob_sym = ((eob_len - 1) << 16) | eob_code;
	jpeg->zrl_sym = ((zrl_len - 1) << 16) | zrl_code;

	dht_len += n32jpeg_append_dht(jpeg->dht + dht_len, 0x00,
				      n32jpeg_dc_bits, n32jpeg_dc_values,
				      ARRAY_SIZE(n32jpeg_dc_values));
	dht_len += n32jpeg_append_dht(jpeg->dht + dht_len, 0x10,
				      n32jpeg_ac_bits, n32jpeg_ac_values,
				      ARRAY_SIZE(n32jpeg_ac_values));
	if (dht_len != JPEG_DHT_LEN)
		return -EINVAL;

	return 0;
}

static void n32h7xx_jpeg_build_quant(struct n32h7xx_jpeg *jpeg, u32 quality)
{
	unsigned int scale;
	unsigned int i;

	if (quality < 50)
		scale = 5000 / quality;
	else
		scale = 200 - quality * 2;

	for (i = 0; i < ARRAY_SIZE(jpeg->qt); i++) {
		unsigned int value = n32jpeg_luma_quant[n32jpeg_zigzag[i]];

		value = DIV_ROUND_CLOSEST(value * scale, 100);
		jpeg->qt[i] = clamp(value, 1U, 255U);
	}
}

static void n32h7xx_jpeg_fill_header(struct n32h7xx_jpeg *jpeg, u32 width,
				     u32 height)
{
	u8 header[JPEG_HEADER_LEN] = {
		0xff, 0xd8, 0xff, 0xc0, 0x00, 0x0b, 0x08,
		0, 0, 0, 0, 0x01, 0x01, 0x11, 0x00,
	};
	unsigned int dht_word_bytes = ALIGN(JPEG_HEADER_LEN, 4);
	unsigned int footer_word_bytes = dht_word_bytes + JPEG_DHT_LEN;
	u8 *buf = (u8 *)jpeg->hf_words;

	n32jpeg_be16_put(header + 7, height);
	n32jpeg_be16_put(header + 9, width);

	memset(jpeg->hf_words, 0, sizeof(jpeg->hf_words));
	memcpy(buf, header, sizeof(header));
	memcpy(buf + dht_word_bytes, jpeg->dht, JPEG_DHT_LEN);
	buf[footer_word_bytes] = 0xff;
	buf[footer_word_bytes + 1] = 0xd9;
}

static void n32h7xx_jpeg_write_tables(struct n32h7xx_jpeg *jpeg)
{
	void __iomem *enc = jpeg->encoder;
	unsigned int i, hf_words;

	for (i = 0; i < ARRAY_SIZE(jpeg->dc_col); i++)
		writel(jpeg->dc_col[i], enc + JPEG_ENC_HT0_DCOL + i * 4);
	for (i = 0; i < ARRAY_SIZE(jpeg->ac_col); i++)
		writel(jpeg->ac_col[i], enc + JPEG_ENC_HT0_ACOL + i * 4);
	for (i = 0; i < ARRAY_SIZE(jpeg->dc_code); i++)
		writel(jpeg->dc_code[i], enc + JPEG_ENC_HT0_DCCODE + i * 4);
	for (i = 0; i < ARRAY_SIZE(jpeg->ac_code); i++)
		writel(jpeg->ac_code[i], enc + JPEG_ENC_HT0_ACCODE + i * 4);

	writel(jpeg->eob_sym, enc + JPEG_ENC_HT0_EOB);
	writel(jpeg->zrl_sym, enc + JPEG_ENC_HT0_ZRL);

	for (i = 0; i < ARRAY_SIZE(jpeg->qt); i++)
		writel(jpeg->qt[i], enc + JPEG_ENC_QT0 + i * 4);
	/* HFBUFFER is a packed header SRAM, not a 2 KiB scratch buffer. */
	hf_words = DIV_ROUND_UP(JPEG_HF_BUFFER_BYTES, 4);
	for (i = 0; i < hf_words; i++)
		writel(jpeg->hf_words[i], enc + JPEG_ENC_HFBUFFER + i * 4);

	writel(JPEG_HEADER_LEN - 1, enc + JPEG_ENC_HEADD);
	writel(ALIGN(JPEG_HEADER_LEN, 4) + JPEG_DHT_LEN - 1,
	       enc + JPEG_ENC_HUFTAB0E);
	writel(0, enc + JPEG_ENC_HUFTAB1E);
	writel(0, enc + JPEG_ENC_HUFTAB2E);
	writel(0, enc + JPEG_ENC_HUFTAB3E);
	writel(ALIGN(JPEG_HEADER_LEN, 4) + JPEG_DHT_LEN + JPEG_FOOTER_LEN - 1,
	       enc + JPEG_ENC_FEADD);
	writel(JPEG_ENC_HSEL_QT0 | JPEG_ENC_HSEL_HT0,
	       enc + JPEG_ENC_HSEL);
	writel(0, enc + JPEG_ENC_RICTRL);
}

static int n32h7xx_jpeg_configure_rbc(struct n32h7xx_jpeg *jpeg, u32 width,
				      u32 height)
{
	void __iomem *rbc = jpeg->encoder_rbc;
	u32 width_mcus = DIV_ROUND_UP(width, 8);
	u32 height_mcus = DIV_ROUND_UP(height, 8);
	u32 init;
	int ret;

	writel(0, rbc + JPEG_RBC_EN);
	writel(JPEG_RBC_PFORM_MONO, rbc + JPEG_RBC_PFORM);
	writel(JPEG_RBC_CNAME_MONO, rbc + JPEG_RBC_CNAME);
	writel(jpeg->scratch_dma, rbc + JPEG_RBC_C0SADD);
	writel(JPEG_RBC_LINES - 1, rbc + JPEG_RBC_C0EADD);
	writel(0, rbc + JPEG_RBC_SWITCH);
	writel(width, rbc + JPEG_RBC_FRMW);
	writel(height - 1, rbc + JPEG_RBC_FRMH);
	writel(((height + 1) >> 1) - 1, rbc + JPEG_RBC_HHALF);
	writel(width_mcus * height_mcus - 1, rbc + JPEG_RBC_BPS0);
	writel(width_mcus * height_mcus - 1, rbc + JPEG_RBC_BPS12);
	writel(height_mcus - 1, rbc + JPEG_RBC_ROWS0);
	writel(height_mcus - 1, rbc + JPEG_RBC_ROWS12);
	writel(width_mcus * 8, rbc + JPEG_RBC_BLSS0);
	writel(width_mcus * 8, rbc + JPEG_RBC_BLSS12);
	writel(width_mcus - 1, rbc + JPEG_RBC_BPRS0);
	writel(width_mcus - 1, rbc + JPEG_RBC_BPRS12);

	writel(JPEG_RBC_INIT_INIT, rbc + JPEG_RBC_INIT);
	ret = readl_poll_timeout(rbc + JPEG_RBC_INIT, init,
				 init & JPEG_RBC_INIT_INITF, 10,
				 N32H7XX_JPEG_MAX_TIMEOUT_US);
	writel(0, rbc + JPEG_RBC_INIT);
	if (ret)
		return ret;

	writel(JPEG_RBC_EN_EN, rbc + JPEG_RBC_EN);
	return 0;
}

static int n32h7xx_jpeg_reset_sgdma(void __iomem *dma)
{
	u32 ctrl, sts;
	int ret;

	/* CLR must not be asserted while an AXI transaction is still active. */
	ret = readl_poll_timeout(dma + JPEG_DMA_STS, sts,
				 !(sts & JPEG_DMA_STS_BUSY), 10, 100000);
	if (ret)
		return ret;

	writel(JPEG_DMA_CTRL_CLR, dma + JPEG_DMA_CTRL);
	return readl_poll_timeout(dma + JPEG_DMA_CTRL, ctrl,
				  !(ctrl & JPEG_DMA_CTRL_CLR), 10, 10000);
}

static void n32h7xx_jpeg_program_sgdma(void __iomem *dma,
				       dma_addr_t address, u32 size)
{
	writel(0, dma + JPEG_DMA_IE);
	writel(0, dma + JPEG_DMA_INTSTS);
	writel(0, dma + JPEG_DMA_AOODT);
	writel(0x80, dma + JPEG_DMA_MBSIZE);
	writel(JPEG_DMA_CTRL_EN | JPEG_DMA_CTRL_MMODE_CSR |
	       JPEG_DMA_CTRL_OPMODE_INCR,
	       dma + JPEG_DMA_CTRL);
	writel(JPEG_DMA_DESCF_START | JPEG_DMA_DESCF_END,
	       dma + JPEG_DMA_DESCF);
	writel(size, dma + JPEG_DMA_DESC_MBSIZE);
	writel(0, dma + JPEG_DMA_DESC_MUS);
	writel(lower_32_bits(address), dma + JPEG_DMA_DESC_MBADDL);
	writel(upper_32_bits(address), dma + JPEG_DMA_DESC_MBADDH);
}

static void n32h7xx_jpeg_program_p2h_list(struct n32h7xx_jpeg *jpeg,
					  dma_addr_t output_dma,
					  u32 output_capacity)
{
	struct n32h7xx_jpeg_dma_desc *desc = jpeg->p2h_desc;
	void __iomem *dma = jpeg->decoder_sgdma;
	unsigned int i;

	memset(desc, 0, sizeof(*desc) * N32H7XX_JPEG_P2H_DESC_COUNT);
	for (i = 0; i < N32H7XX_JPEG_P2H_DESC_COUNT; i++) {
		if (!i) {
			desc[i].flags = JPEG_DMA_DESC_START;
			desc[i].block_size = output_capacity;
			desc[i].block_addr_lo = lower_32_bits(output_dma);
			desc[i].block_addr_hi = upper_32_bits(output_dma);
		} else {
			desc[i].block_size = N32H7XX_JPEG_P2H_SPILL_BYTES;
			desc[i].block_addr_lo =
				lower_32_bits(jpeg->p2h_spill_dma);
			desc[i].block_addr_hi =
				upper_32_bits(jpeg->p2h_spill_dma);
		}
	}
	/*
	 * A final END marks the last pad; the codec's own EOF still terminates
	 * the transfer when the JPEG ends in an earlier descriptor.
	 */
	desc[N32H7XX_JPEG_P2H_DESC_COUNT - 1].flags |= JPEG_DMA_DESC_END;
	dma_wmb();
	dma_sync_single_for_device(jpeg->dev, jpeg->p2h_desc_dma,
				   sizeof(*desc) * N32H7XX_JPEG_P2H_DESC_COUNT,
				   DMA_TO_DEVICE);

	writel(0, dma + JPEG_DMA_IE);
	writel(0, dma + JPEG_DMA_INTSTS);
	writel(0, dma + JPEG_DMA_AOODT);
	writel(0x80, dma + JPEG_DMA_MBSIZE);
	writel(JPEG_DMA_CTRL_EN | JPEG_DMA_CTRL_OPMODE_INCR,
	       dma + JPEG_DMA_CTRL);
	writel(lower_32_bits(jpeg->p2h_desc_dma), dma + JPEG_DMA_SGLPL);
	writel(upper_32_bits(jpeg->p2h_desc_dma), dma + JPEG_DMA_SGLPH);
	writel(N32H7XX_JPEG_P2H_DESC_COUNT, dma + JPEG_DMA_SGLSIZE);
	writel(0, dma + JPEG_DMA_SGLHEAD);
	writel(0, dma + JPEG_DMA_SGLTAIL);
}

static void n32h7xx_jpeg_dump_dma(struct n32h7xx_jpeg *jpeg, const char *tag)
{
	void __iomem *dma = jpeg->encoder_sgdma;

	dev_err(jpeg->dev,
		"%s h2p: ctrl=%#x sts=%#x descf=%#x size=%u used=%u addr=%#x/%#x mr=%#x/%#x para=%#x\n",
		tag, readl(dma + JPEG_DMA_CTRL), readl(dma + JPEG_DMA_STS),
		readl(dma + JPEG_DMA_DESCF),
		readl(dma + JPEG_DMA_DESC_MBSIZE),
		readl(dma + JPEG_DMA_DESC_MUS),
		readl(dma + JPEG_DMA_DESC_MBADDH),
		readl(dma + JPEG_DMA_DESC_MBADDL), readl(dma + JPEG_DMA_MRADD),
		readl(dma + JPEG_DMA_MRADD + 4), readl(dma + JPEG_DMA_PARACFG));

	dma = jpeg->decoder_sgdma;
	dev_err(jpeg->dev,
		"%s p2h: ctrl=%#x sts=%#x head=%u tail=%u desc0=%#x/%u/%u mw=%#x/%#x para=%#x\n",
		tag, readl(dma + JPEG_DMA_CTRL), readl(dma + JPEG_DMA_STS),
		readl(dma + JPEG_DMA_SGLHEAD), readl(dma + JPEG_DMA_SGLTAIL),
		jpeg->p2h_desc[0].flags, jpeg->p2h_desc[0].block_size,
		jpeg->p2h_desc[0].block_used, readl(dma + JPEG_DMA_MWADD),
		readl(dma + JPEG_DMA_MWADD + 4), readl(dma + JPEG_DMA_PARACFG));

	dev_err(jpeg->dev,
		"%s enc: common=%#x ctrl=%#x hsel=%#x pipes=%#x rbc init=%#x en=%#x frm=%ux%u pform=%u c0=%#x..%#x bps=%u rows=%u blss=%u bprs=%u\n",
		tag, readl(jpeg->common), readl(jpeg->encoder + JPEG_ENC_CTRL),
		readl(jpeg->encoder + JPEG_ENC_HSEL),
		readl(jpeg->encoder + JPEG_ENC_PIPESTS),
		readl(jpeg->encoder_rbc + JPEG_RBC_INIT),
		readl(jpeg->encoder_rbc + JPEG_RBC_EN),
		readl(jpeg->encoder_rbc + JPEG_RBC_FRMW),
		readl(jpeg->encoder_rbc + JPEG_RBC_FRMH) + 1,
		readl(jpeg->encoder_rbc + JPEG_RBC_PFORM),
		readl(jpeg->encoder_rbc + JPEG_RBC_C0SADD),
		readl(jpeg->encoder_rbc + JPEG_RBC_C0EADD),
		readl(jpeg->encoder_rbc + JPEG_RBC_BPS0),
		readl(jpeg->encoder_rbc + JPEG_RBC_ROWS0),
		readl(jpeg->encoder_rbc + JPEG_RBC_BLSS0),
		readl(jpeg->encoder_rbc + JPEG_RBC_BPRS0));
}

static int n32h7xx_jpeg_wait_transfer(struct n32h7xx_jpeg *jpeg,
				      u32 timeout_us, u32 *h2p_status,
				      u32 *p2h_status)
{
	u64 deadline = ktime_get_ns() + (u64)timeout_us * NSEC_PER_USEC;
	u32 h2p, p2h;

	for (;;) {
		p2h = readl(jpeg->decoder_sgdma + JPEG_DMA_STS);
		h2p = readl(jpeg->encoder_sgdma + JPEG_DMA_STS);
		if ((h2p | p2h) & JPEG_DMA_STS_FATAL) {
			*h2p_status = h2p;
			*p2h_status = p2h;
			return -EIO;
		}

		/*
		 * A consumed list descriptor sets EOD even though the spill list
		 * still has space.  Only an empty list is an output-full result.
		 */
		if (p2h & JPEG_DMA_STS_OOD) {
			*h2p_status = h2p;
			*p2h_status = p2h;
			return -ENOSPC;
		}

		if ((h2p & JPEG_DMA_STS_EOFIN) &&
		    (p2h & JPEG_DMA_STS_EOFOUT)) {
			*h2p_status = h2p;
			*p2h_status = p2h;
			return 0;
		}

		if (ktime_get_ns() >= deadline) {
			*h2p_status = h2p;
			*p2h_status = p2h;
			return -ETIMEDOUT;
		}
		usleep_range(10, 20);
	}
}

static void n32h7xx_jpeg_abort(struct n32h7xx_jpeg *jpeg)
{
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev, "abort: stopping JPEG engine\n");
	writel(0, jpeg->encoder_rbc + JPEG_RBC_EN);
	writel(0, jpeg->encoder + JPEG_ENC_CTRL);
	/*
	 * Once OOD is entered, waiting for BUSY to fall can never complete.
	 * Reset both codec halves so all SGDMA masters and in-flight state return
	 * to a known idle state before the next request.
	 */
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev, "abort: resetting JPEG codec blocks\n");
	reset_control_assert(jpeg->encoder_reset);
	reset_control_assert(jpeg->decoder_reset);
	udelay(20);
	reset_control_deassert(jpeg->encoder_reset);
	reset_control_deassert(jpeg->decoder_reset);
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev, "abort: complete\n");
}

static u32 n32h7xx_jpeg_timeout_us(u32 width, u32 height)
{
	u64 timeout = N32H7XX_JPEG_MIN_TIMEOUT_US +
		      DIV_ROUND_UP_ULL((u64)width * height, 16);

	return clamp_t(u64, timeout, N32H7XX_JPEG_MIN_TIMEOUT_US,
		       N32H7XX_JPEG_MAX_TIMEOUT_US);
}

static int n32h7xx_jpeg_find_eoi(const struct n32jpeg_encode *enc, u32 used,
				 u32 *jpeg_size)
{
	const u8 __user *output = u64_to_user_ptr(enc->output);
	u32 scan_end = used;
	u32 scan_start;
	u32 pos;
	u8 b;

	if (scan_end < 4 || scan_end > enc->output_capacity)
		return -EIO;

	if (get_user(b, output) || b != 0xff)
		return -EIO;
	if (get_user(b, output + 1) || b != 0xd8)
		return -EIO;

	scan_start = used > 256 ? used - 256 : 0;
	for (pos = scan_end - 2;; pos--) {
		u8 next;

		if (get_user(b, output + pos) ||
		    get_user(next, output + pos + 1))
			return -EFAULT;
		if (b == 0xff && next == 0xd9) {
			*jpeg_size = pos + 2;
			return 0;
		}
		if (pos == scan_start)
			break;
	}

	return used == enc->output_capacity ? -ENOSPC : -EIO;
}

static bool n32h7xx_jpeg_user_range_valid(u64 address, size_t length)
{
	if (!address || address > ULONG_MAX ||
	    length > ULONG_MAX - (unsigned long)address)
		return false;

	return access_ok(u64_to_user_ptr(address), length);
}

static int n32h7xx_jpeg_validate_encode(const struct n32jpeg_encode *enc,
					size_t *input_size)
{
	u64 pixels;

	if (enc->struct_size != sizeof(*enc) || enc->flags)
		return -EINVAL;
	if (enc->format != N32JPEG_FORMAT_GREY8)
		return enc->format == N32JPEG_FORMAT_RGB24 ||
		       enc->format == N32JPEG_FORMAT_YUV422 ?
		       -EOPNOTSUPP : -EINVAL;
	if (!enc->width || enc->width > N32H7XX_JPEG_MAX_WIDTH ||
	    !enc->height || enc->height > N32H7XX_JPEG_MAX_HEIGHT)
		return -EINVAL;
	/* RBC fetches whole 8x8 MCUs; a short row would shift every later row. */
	if ((enc->width & (N32H7XX_JPEG_DIMENSION_ALIGNMENT - 1)) ||
	    (enc->height & (N32H7XX_JPEG_DIMENSION_ALIGNMENT - 1)))
		return -EINVAL;
	if (enc->input_stride != enc->width || enc->quality < 1 ||
	    enc->quality > 100)
		return -EINVAL;
	if (!enc->output_capacity)
		return -EINVAL;

	pixels = (u64)enc->width * enc->height;
	if (pixels > SIZE_MAX || enc->input_size < pixels)
		return -EINVAL;
	*input_size = pixels;

	if ((enc->input | enc->output) & (N32H7XX_JPEG_IO_ALIGNMENT - 1))
		return -EINVAL;
	if (!n32h7xx_jpeg_user_range_valid(enc->input, *input_size) ||
	    !n32h7xx_jpeg_user_range_valid(enc->output, enc->output_capacity))
		return -EFAULT;
	if (enc->input < enc->output + enc->output_capacity &&
	    enc->output < enc->input + *input_size)
		return -EINVAL;

	return 0;
}

static int n32h7xx_jpeg_encode(struct n32h7xx_jpeg *jpeg,
			       struct n32jpeg_encode *enc)
{
	dma_addr_t input_dma, output_dma;
	void __user *input = u64_to_user_ptr(enc->input);
	void __user *output = u64_to_user_ptr(enc->output);
	size_t input_size;
	u32 h2p_sts = 0, p2h_sts = 0, output_flags, output_used, output_tail;
	u32 timeout_us;
	int ret;

	ret = n32h7xx_jpeg_validate_encode(enc, &input_size);
	if (ret)
		return ret;

	input_dma = dma_map_single(jpeg->dev, input, input_size,
				   DMA_TO_DEVICE);
	if (dma_mapping_error(jpeg->dev, input_dma))
		return -EFAULT;

	output_dma = dma_map_single(jpeg->dev, output, enc->output_capacity,
				    DMA_FROM_DEVICE);
	if (dma_mapping_error(jpeg->dev, output_dma)) {
		ret = -EFAULT;
		goto unmap_input;
	}

	dma_sync_single_for_device(jpeg->dev, input_dma, input_size,
				   DMA_TO_DEVICE);
	dma_sync_single_for_device(jpeg->dev, output_dma,
				   enc->output_capacity, DMA_FROM_DEVICE);

	ret = n32h7xx_jpeg_reset_sgdma(jpeg->encoder_sgdma);
	if (ret)
		goto abort;
	ret = n32h7xx_jpeg_reset_sgdma(jpeg->decoder_sgdma);
	if (ret)
		goto abort;

	n32h7xx_jpeg_build_quant(jpeg, enc->quality);
	n32h7xx_jpeg_fill_header(jpeg, enc->width, enc->height);

	writel(1, jpeg->common);
	writel(0, jpeg->encoder + JPEG_ENC_CTRL);
	n32h7xx_jpeg_write_tables(jpeg);
	writel(JPEG_ENC_CTRL_MONO | JPEG_ENC_CTRL_ERST | JPEG_ENC_CTRL_EN,
	       jpeg->encoder + JPEG_ENC_CTRL);
	udelay(10);

	ret = n32h7xx_jpeg_configure_rbc(jpeg, enc->width, enc->height);
	if (ret)
		goto abort;

	n32h7xx_jpeg_program_p2h_list(jpeg, output_dma,
				      enc->output_capacity);
	n32h7xx_jpeg_program_sgdma(jpeg->encoder_sgdma, input_dma, input_size);

	timeout_us = n32h7xx_jpeg_timeout_us(enc->width, enc->height);
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev, "start: %ux%u input=%zu output=%u\n",
			 enc->width, enc->height, input_size,
			 enc->output_capacity);
	writel(readl(jpeg->decoder_sgdma + JPEG_DMA_CTRL) |
	       JPEG_DMA_CTRL_START,
	       jpeg->decoder_sgdma + JPEG_DMA_CTRL);
	writel(readl(jpeg->encoder_sgdma + JPEG_DMA_CTRL) |
	       JPEG_DMA_CTRL_START,
	       jpeg->encoder_sgdma + JPEG_DMA_CTRL);

	ret = n32h7xx_jpeg_wait_transfer(jpeg, timeout_us, &h2p_sts,
					 &p2h_sts);
	if (ret) {
		if (ret == -ENOSPC) {
			if (n32h7xx_jpeg_debug)
				dev_info(jpeg->dev,
					 "JPEG output capacity exhausted: h2p=%#x p2h=%#x\n",
					 h2p_sts, p2h_sts);
		} else {
			dev_err(jpeg->dev,
				"JPEG DMA failed: %d, h2p=%#x p2h=%#x\n",
				ret, h2p_sts, p2h_sts);
			n32h7xx_jpeg_dump_dma(jpeg, "fail");
		}
		goto abort;
	}
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev, "p2h complete: status=%#x\n", p2h_sts);

	if (!(h2p_sts & JPEG_DMA_STS_EOFIN) ||
	    !(p2h_sts & JPEG_DMA_STS_EOFOUT)) {
		dev_err(jpeg->dev, "JPEG DMA ended without EOF: h2p=%#x p2h=%#x\n",
			h2p_sts, p2h_sts);
		ret = -EIO;
		goto abort;
	}

	dma_sync_single_for_cpu(jpeg->dev, jpeg->p2h_desc_dma,
				sizeof(*jpeg->p2h_desc) *
				N32H7XX_JPEG_P2H_DESC_COUNT, DMA_FROM_DEVICE);
	dma_rmb();
	output_flags = jpeg->p2h_desc[0].flags;
	output_used = jpeg->p2h_desc[0].block_used;
	output_tail = readl(jpeg->decoder_sgdma + JPEG_DMA_SGLTAIL);
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev,
			 "GREY8 %ux%u: h2p=%#x p2h=%#x desc0=%#x used=%u tail=%u pipes=%#x\n",
			 enc->width, enc->height, h2p_sts, p2h_sts,
			 output_flags, output_used, output_tail,
			 readl(jpeg->encoder + JPEG_ENC_PIPESTS));

	if (output_tail > 1 || output_used > enc->output_capacity ||
	    !(output_flags & JPEG_DMA_DESC_END)) {
		if (n32h7xx_jpeg_debug)
			dev_info(jpeg->dev,
				 "JPEG output capacity exhausted: %u bytes, flags=%#x tail=%u spill used=%u\n",
				 enc->output_capacity, output_flags, output_tail,
				 jpeg->p2h_desc[1].block_used);
		ret = -ENOSPC;
		goto abort;
	}
	if (!output_used) {
		dev_err(jpeg->dev,
			"JPEG output length is zero: h2p=%#x p2h=%#x descf=%#x\n",
			h2p_sts, p2h_sts,
			readl(jpeg->decoder_sgdma + JPEG_DMA_DESCF));
		n32h7xx_jpeg_dump_dma(jpeg, "zero-length");
		ret = -EIO;
		goto abort;
	}

	dma_sync_single_for_cpu(jpeg->dev, output_dma,
				enc->output_capacity, DMA_FROM_DEVICE);
	ret = n32h7xx_jpeg_find_eoi(enc, output_used, &enc->output_size);
	if (ret)
		goto abort;
	if (n32h7xx_jpeg_debug)
		dev_info(jpeg->dev, "GREY8 %ux%u encoded %u bytes\n",
			 enc->width, enc->height, enc->output_size);

	writel(0, jpeg->encoder_rbc + JPEG_RBC_EN);
	writel(0, jpeg->encoder + JPEG_ENC_CTRL);
	dma_unmap_single(jpeg->dev, output_dma, enc->output_capacity,
			 DMA_FROM_DEVICE);
	dma_unmap_single(jpeg->dev, input_dma, input_size, DMA_TO_DEVICE);
	return 0;

abort:
	n32h7xx_jpeg_abort(jpeg);
	dma_unmap_single(jpeg->dev, output_dma, enc->output_capacity,
			 DMA_FROM_DEVICE);
unmap_input:
	dma_unmap_single(jpeg->dev, input_dma, input_size, DMA_TO_DEVICE);
	return ret;
}

static int n32h7xx_jpeg_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct n32h7xx_jpeg *jpeg = container_of(misc, struct n32h7xx_jpeg,
						 misc);

	if (atomic_cmpxchg(&jpeg->open, 0, 1))
		return -EBUSY;

	file->private_data = jpeg;
	return 0;
}

static int n32h7xx_jpeg_release(struct inode *inode, struct file *file)
{
	struct n32h7xx_jpeg *jpeg = file->private_data;

	atomic_set(&jpeg->open, 0);
	return 0;
}

static long n32h7xx_jpeg_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	struct n32h7xx_jpeg *jpeg = file->private_data;
	void __user *argp = (void __user *)arg;
	int ret = 0;

	if (_IOC_TYPE(cmd) != N32JPEG_IOC_MAGIC)
		return -ENOTTY;

	mutex_lock(&jpeg->lock);
	switch (cmd) {
	case N32JPEG_QUERY: {
		struct n32jpeg_query query = {
			.abi_version = N32JPEG_ABI_VERSION,
			.formats = BIT(N32JPEG_FORMAT_GREY8),
			.min_width = N32H7XX_JPEG_DIMENSION_ALIGNMENT,
			.min_height = N32H7XX_JPEG_DIMENSION_ALIGNMENT,
			.max_width = N32H7XX_JPEG_MAX_WIDTH,
			.max_height = N32H7XX_JPEG_MAX_HEIGHT,
			.input_alignment = N32H7XX_JPEG_IO_ALIGNMENT,
			.output_alignment = N32H7XX_JPEG_IO_ALIGNMENT,
		};

		if (copy_to_user(argp, &query, sizeof(query)))
			ret = -EFAULT;
		break;
	}
	case N32JPEG_ENCODE: {
		struct n32jpeg_encode encode;

		if (copy_from_user(&encode, argp, sizeof(encode))) {
			ret = -EFAULT;
			break;
		}
		encode.output_size = 0;
		ret = n32h7xx_jpeg_encode(jpeg, &encode);
		if (!ret && copy_to_user(argp, &encode, sizeof(encode)))
			ret = -EFAULT;
		break;
	}
	default:
		ret = -ENOTTY;
		break;
	}
	mutex_unlock(&jpeg->lock);

	return ret;
}

static const struct file_operations n32h7xx_jpeg_fops = {
	.owner = THIS_MODULE,
	.open = n32h7xx_jpeg_open,
	.release = n32h7xx_jpeg_release,
	.unlocked_ioctl = n32h7xx_jpeg_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static int n32h7xx_jpeg_probe(struct platform_device *pdev)
{
	static const char *const names[] = {
		"encoder", "encoder-rbc", "encoder-sgdma", "decoder",
		"decoder-brc", "decoder-sgdma", "common",
	};
	void __iomem **regs[7] = {
		NULL,
	};
	struct device *dev = &pdev->dev;
	struct n32h7xx_jpeg *jpeg;
	struct clk *encoder_clk, *decoder_clk;
	unsigned int i;
	int ret;

	jpeg = devm_kzalloc(dev, sizeof(*jpeg), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;
	jpeg->dev = dev;
	mutex_init(&jpeg->lock);
	atomic_set(&jpeg->open, 0);

	regs[0] = &jpeg->encoder;
	regs[1] = &jpeg->encoder_rbc;
	regs[2] = &jpeg->encoder_sgdma;
	regs[3] = &jpeg->decoder;
	regs[4] = &jpeg->decoder_brc;
	regs[5] = &jpeg->decoder_sgdma;
	regs[6] = &jpeg->common;
	for (i = 0; i < ARRAY_SIZE(names); i++) {
		*regs[i] = devm_platform_ioremap_resource_byname(pdev, names[i]);
		if (IS_ERR(*regs[i]))
			return PTR_ERR(*regs[i]);
	}

	encoder_clk = devm_clk_get_enabled(dev, "encoder");
	if (IS_ERR(encoder_clk))
		return dev_err_probe(dev, PTR_ERR(encoder_clk),
				     "failed to enable encoder clock\n");
	decoder_clk = devm_clk_get_enabled(dev, "decoder");
	if (IS_ERR(decoder_clk))
		return dev_err_probe(dev, PTR_ERR(decoder_clk),
				     "failed to enable decoder clock\n");

	jpeg->encoder_reset = devm_reset_control_get_exclusive(dev, "encoder");
	if (IS_ERR(jpeg->encoder_reset))
		return dev_err_probe(dev, PTR_ERR(jpeg->encoder_reset),
				     "failed to get encoder reset\n");
	jpeg->decoder_reset = devm_reset_control_get_exclusive(dev, "decoder");
	if (IS_ERR(jpeg->decoder_reset))
		return dev_err_probe(dev, PTR_ERR(jpeg->decoder_reset),
				     "failed to get decoder reset\n");

	ret = n32h7xx_jpeg_soc_init(dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialise JPEG power\n");

	reset_control_assert(jpeg->encoder_reset);
	reset_control_assert(jpeg->decoder_reset);
	udelay(10);
	reset_control_deassert(jpeg->encoder_reset);
	reset_control_deassert(jpeg->decoder_reset);

	ret = devm_of_reserved_mem_device_init(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to claim JPEG DMA pool\n");

	jpeg->block_line_bytes = 64 * DIV_ROUND_UP(N32H7XX_JPEG_MAX_WIDTH, 8);
	jpeg->scratch = dmam_alloc_coherent(dev,
					    jpeg->block_line_bytes * JPEG_RBC_LINES,
					   &jpeg->scratch_dma, GFP_KERNEL);
	if (!jpeg->scratch)
		return -ENOMEM;

	jpeg->p2h_desc = dmam_alloc_coherent(dev,
					     sizeof(*jpeg->p2h_desc) *
					    N32H7XX_JPEG_P2H_DESC_COUNT,
					    &jpeg->p2h_desc_dma, GFP_KERNEL);
	if (!jpeg->p2h_desc)
		return -ENOMEM;
	jpeg->p2h_spill = dmam_alloc_coherent(dev,
					      N32H7XX_JPEG_P2H_SPILL_BYTES,
					     &jpeg->p2h_spill_dma, GFP_KERNEL);
	if (!jpeg->p2h_spill)
		return -ENOMEM;

	ret = n32h7xx_jpeg_build_tables(jpeg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to build baseline tables\n");

	jpeg->misc.minor = MISC_DYNAMIC_MINOR;
	jpeg->misc.name = "jpeg";
	jpeg->misc.fops = &n32h7xx_jpeg_fops;
	jpeg->misc.parent = dev;
	ret = misc_register(&jpeg->misc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register /dev/jpeg\n");

	platform_set_drvdata(pdev, jpeg);
	dev_info(dev, "GREY8 encoder ready, max %ux%u, scratch %zu KiB, P2H list %u\n",
		 N32H7XX_JPEG_MAX_WIDTH, N32H7XX_JPEG_MAX_HEIGHT,
		 jpeg->block_line_bytes * JPEG_RBC_LINES / 1024,
		 N32H7XX_JPEG_P2H_DESC_COUNT);
	return 0;
}

static void n32h7xx_jpeg_remove(struct platform_device *pdev)
{
	struct n32h7xx_jpeg *jpeg = platform_get_drvdata(pdev);

	misc_deregister(&jpeg->misc);
	writel(0, jpeg->encoder_rbc + JPEG_RBC_EN);
	writel(0, jpeg->encoder + JPEG_ENC_CTRL);
}

static const struct of_device_id n32h7xx_jpeg_of_match[] = {
	{ .compatible = "nsing,n32h7xx-jpeg" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32h7xx_jpeg_of_match);

static struct platform_driver n32h7xx_jpeg_driver = {
	.probe = n32h7xx_jpeg_probe,
	.remove = n32h7xx_jpeg_remove,
	.driver = {
		.name = "n32h7xx-jpeg",
		.of_match_table = n32h7xx_jpeg_of_match,
	},
};
module_platform_driver(n32h7xx_jpeg_driver);

MODULE_DESCRIPTION("NSING N32H7xx hardware JPEG encoder");
MODULE_LICENSE("GPL");
