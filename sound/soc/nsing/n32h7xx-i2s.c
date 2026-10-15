// SPDX-License-Identifier: GPL-2.0-only
/*
 * I2S1 as a WM8978 slave, with I2S4 generating the codec MCLK.
 *
 * The I2S1 instance is a pair of blocks that share CK and WS: the main block
 * owns SD (PG9) and the I2S1_EXT block owns SD_EXT (PD7).  On this board the
 * codec sends its ADC data on SD and takes its DAC data from SD_EXT, so
 * capture runs on the main block as a slave receiver and playback runs on the
 * extension as a slave transmitter.  That is the vendor's slave full-duplex
 * pairing, so both directions can be open at the same time.
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define CR2 0x04
#define SR 0x08
#define DR 0x0c
#define I2SCFG 0x1c
#define I2SDIV 0x20
#define I2S_ENABLE BIT(0)
#define I2S_RX_DMA BIT(1)
#define I2S_TX_DMA BIT(2)
/*
 * MODE[5:4] is 00 for slave transmit and 01 for slave receive, and CHLEN
 * (bit 8) makes each 16-bit sample occupy a 32-bit slot, which is what the
 * codec drives: WM8978 is the clock master at 256fs with BCLKDIV = 2.
 */
#define I2S_SLAVE_TX (BIT(1) | BIT(8))
#define I2S_SLAVE_RX (BIT(1) | BIT(4) | BIT(8))
#define I2S_MASTER_RX (BIT(1) | BIT(4) | BIT(5))

struct n32_i2s {
	/* Main block: slave RX on SD (PG9).  Extension: slave TX on SD_EXT (PD7). */
	void __iomem *rx, *tx, *mclk;
	void __iomem *ws;
	u32 ws_mask;
	struct clk_bulk_data clocks[2];
	struct reset_control_bulk_data resets[2];
	struct snd_dmaengine_dai_dma_data playback;
	struct snd_dmaengine_dai_dma_data capture;
};

static bool is_playback(struct snd_pcm_substream *substream)
{
	return substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
}

static int n32_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static int n32_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct n32_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = is_playback(substream);
	void __iomem *block = playback ? i2s->tx : i2s->rx;
	int other;

	if (params_format(params) != SNDRV_PCM_FORMAT_S16_LE ||
	    params_channels(params) != 2)
		return -EINVAL;
	/*
	 * Each direction owns its own block, so a stream only ever programs its
	 * own half.  The reset line is shared, though, so it is pulsed only
	 * while the opposite direction is idle - reconfiguring one half while
	 * the other one is streaming would wipe it.
	 */
	other = playback ? SNDRV_PCM_STREAM_CAPTURE : SNDRV_PCM_STREAM_PLAYBACK;
	if (!snd_soc_dai_stream_active(dai, other) &&
	    reset_control_reset(i2s->resets[0].rstc))
		return -EIO;
	writew(0, block + CR2);
	writew(playback ? I2S_SLAVE_TX : I2S_SLAVE_RX, block + I2SCFG);
	/* The extension block has no divider register, and slaves ignore it. */
	if (!playback)
		writew(2, block + I2SDIV);
	/* Clear stale data and the overrun/underrun status before arming DMA. */
	readw(block + DR);
	readw(block + SR);
	return 0;
}

/*
 * Analog settle delay for capture start.  DAPM powers the microphone bias and
 * the ADC up inside snd_pcm_prepare(), and this board's microphone coupling
 * network recharges with a ~150 ms time constant, so a cold front end steps at
 * the head of the recording.  Peak of the first 100 ms after 15 s of idle,
 * measured here: 300 ms -> 2400 LSB, 500 ms -> 650 LSB, 800 ms -> noise floor.
 */
static unsigned int settle_ms = 800;
module_param(settle_ms, uint, 0644);
MODULE_PARM_DESC(settle_ms, "capture start analog settle delay in ms");

/*
 * DAPM switches the codec's microphone bias and ADC on inside
 * snd_pcm_prepare(), and it does so after the DAI .prepare hook has already
 * returned, so a settle delay placed there elapses with the front end still
 * off.  This callback is the first point in the prepare path that runs after
 * that power up and may still sleep - the PCM trigger is not, it holds the
 * stream lock with interrupts disabled.  The receiver is armed only once this
 * returns, so capturing starts on a quiet front end.
 */
static int n32_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	if (!mute && stream == SNDRV_PCM_STREAM_CAPTURE)
		msleep(settle_ms);

	return 0;
}

static int n32_wait_ws(struct n32_i2s *i2s, bool high)
{
	unsigned int limit = 100000;

	/* Keep the edge-to-enable path short enough for the first I2S bit. */
	do {
		if (!!(readl(i2s->ws) & i2s->ws_mask) == high)
			return 0;
		cpu_relax();
	} while (--limit);
	return -ETIMEDOUT;
}

static int n32_trigger(struct snd_pcm_substream *substream, int cmd,
		       struct snd_soc_dai *dai)
{
	struct n32_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = is_playback(substream);
	void __iomem *block = playback ? i2s->tx : i2s->rx;
	u16 cfg = playback ? I2S_SLAVE_TX : I2S_SLAVE_RX;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		writew(playback ? I2S_TX_DMA : I2S_RX_DMA, block + CR2);
		/*
		 * Both halves are slaves and the codec never stops CK/WS, so
		 * both are armed on a frame boundary: a receiver enabled
		 * mid-frame drops its first slot, and a transmitter only starts
		 * on the left channel if it is enabled at the head of a frame.
		 */
		if (n32_wait_ws(i2s, false) || n32_wait_ws(i2s, true)) {
			writew(0, block + CR2);
			return -ETIMEDOUT;
		}
		writew(cfg | I2S_ENABLE, block + I2SCFG);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		writew(0, block + CR2);
		writew(cfg, block + I2SCFG);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int n32_dai_probe(struct snd_soc_dai *dai)
{
	struct n32_i2s *i2s = dev_get_drvdata(dai->dev);

	snd_soc_dai_set_drvdata(dai, i2s);
	snd_soc_dai_init_dma_data(dai, &i2s->playback, &i2s->capture);
	return 0;
}

static const struct snd_soc_dai_ops n32_dai_ops = {
	.probe = n32_dai_probe,
	.set_fmt = n32_set_fmt,
	.hw_params = n32_hw_params,
	.mute_stream = n32_mute_stream,
	.trigger = n32_trigger,
};

static struct snd_soc_dai_driver n32_dai = {
	.name = "n32h7xx-i2s",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &n32_dai_ops,
};

static const struct snd_soc_component_driver n32_component = {
	.name = "n32h7xx-i2s",
};

static const struct snd_pcm_hardware n32_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	/*
	 * Both directions are clock slaves: CK and WS come from the WM8978,
	 * whose PLL the codec driver reprograms for whatever rate the stream
	 * asks for, and BCLK stays at 64fs.  So this side has no rate of its
	 * own and offers what the codec offers - the same 8000..48000 set,
	 * including the 44.1 kHz family (11025/22050/44100) that the narrower
	 * list used to reject before hw_params was ever reached.
	 */
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 65536,
	.period_bytes_min = 256,
	.period_bytes_max = 4096,
	.periods_min = 2,
	.periods_max = 64,
};

static const struct snd_dmaengine_pcm_config n32_pcm_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &n32_pcm_hw,
	.prealloc_buffer_size = 65536,
};

static int n32_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32_i2s *i2s;
	struct resource *res;
	struct of_phandle_args monitor;
	u32 gpio_base;
	int ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;
	i2s->rx = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2s->rx))
		return PTR_ERR(i2s->rx);
	i2s->tx = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(i2s->tx))
		return PTR_ERR(i2s->tx);
	i2s->mclk = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(i2s->mclk))
		return PTR_ERR(i2s->mclk);
	i2s->clocks[0].id = "rx";
	i2s->clocks[1].id = "mclk";
	ret = devm_clk_bulk_get(dev, 2, i2s->clocks);
	if (ret)
		return ret;
	i2s->resets[0].id = "rx";
	i2s->resets[1].id = "mclk";
	ret = devm_reset_control_bulk_get_exclusive(dev, 2, i2s->resets);
	if (ret)
		return ret;
	ret = of_parse_phandle_with_args(dev->of_node, "nsing,ws-monitor",
					 "#gpio-cells", 0, &monitor);
	if (ret)
		return ret;
	ret = of_property_read_u32(monitor.np, "reg", &gpio_base);
	of_node_put(monitor.np);
	if (ret || monitor.args_count != 2 || monitor.args[0] >= 16 ||
	    monitor.args[1] || gpio_base < 0x58032800 || gpio_base > 0x58035000 ||
	    (gpio_base - 0x58032800) % 0x400)
		return -EINVAL;
	/* Read-only monitor: WS stays muxed to I2S, without a GPIO request. */
	i2s->ws = devm_ioremap(dev, gpio_base + 0x10, sizeof(u32));
	if (!i2s->ws)
		return -ENOMEM;
	i2s->ws_mask = BIT(monitor.args[0]);
	ret = clk_bulk_prepare_enable(2, i2s->clocks);
	if (ret)
		return ret;
	ret = reset_control_bulk_reset(2, i2s->resets);
	if (ret)
		goto disable_clocks;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto disable_clocks;
	/* RX master keeps MCLK running even without data, unlike TX master. */
	writew(0, i2s->mclk + CR2);
	writew(2 | BIT(11), i2s->mclk + I2SDIV);
	writew(I2S_MASTER_RX | I2S_ENABLE, i2s->mclk + I2SCFG);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	i2s->playback.addr = res->start + DR;
	i2s->playback.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->playback.maxburst = 1;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	i2s->capture.addr = res->start + DR;
	i2s->capture.addr_width = DMA_SLAVE_BUSWIDTH_2_BYTES;
	i2s->capture.maxburst = 1;
	platform_set_drvdata(pdev, i2s);
	ret = devm_snd_soc_register_component(dev, &n32_component, &n32_dai, 1);
	if (ret)
		goto stop_mclk;
	ret = devm_snd_dmaengine_pcm_register(dev, &n32_pcm_config, 0);
	if (ret)
		goto stop_mclk;
	dev_info(dev, "capture on SD, playback on SD_EXT, MCLK %lu Hz\n",
		 clk_get_rate(i2s->clocks[1].clk) / 4);
	return 0;
stop_mclk:
	writew(0, i2s->mclk + I2SCFG);
disable_clocks:
	clk_bulk_disable_unprepare(2, i2s->clocks);
	return ret;
}

static void n32_i2s_remove(struct platform_device *pdev)
{
	struct n32_i2s *i2s = platform_get_drvdata(pdev);

	writew(0, i2s->rx + CR2);
	writew(0, i2s->rx + I2SCFG);
	writew(0, i2s->mclk + I2SCFG);
	clk_bulk_disable_unprepare(2, i2s->clocks);
}

static const struct of_device_id n32_i2s_match[] = {
	{ .compatible = "nsing,n32h7xx-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32_i2s_match);
static struct platform_driver n32_i2s_driver = {
	.probe = n32_i2s_probe,
	.remove = n32_i2s_remove,
	.driver = { .name = "n32h7xx-i2s", .of_match_table = n32_i2s_match },
};
module_platform_driver(n32_i2s_driver);
MODULE_DESCRIPTION("N32H7xx I2S slave capture and playback");
MODULE_LICENSE("GPL");
