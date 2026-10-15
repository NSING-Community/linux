// SPDX-License-Identifier: GPL-2.0-only
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../codecs/wm8978.h"

struct n32_audio_card {
	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_dai_link_component cpu, codec, platform;
	u32 mclk;
};

static int n32_card_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct n32_audio_card *priv = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *codec = snd_soc_rtd_to_codec(rtd, 0);
	int ret;

	ret = snd_soc_dai_set_sysclk(codec, WM8978_PLL, priv->mclk,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;
	/* 256fs / 4: 64 BCLKs per stereo frame, 16 valid bits in each slot. */
	return snd_soc_dai_set_clkdiv(codec, WM8978_BCLKDIV, 2 << 2);
}

static int n32_card_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *codec = snd_soc_rtd_to_codec(rtd, 0)->component;
	int ret;

	/* Differential electret microphone; PGA +24 dB, boost +20 dB. */
	ret = snd_soc_component_write(codec, WM8978_LEFT_INP_PGA_CONTROL, 48);
	if (ret)
		return ret;
	ret = snd_soc_component_write(codec, WM8978_RIGHT_INP_PGA_CONTROL, 48 | 0x100);
	if (ret)
		return ret;
	ret = snd_soc_component_update_bits(codec, WM8978_LEFT_ADC_BOOST_CONTROL,
					    0x100, 0x100);
	if (ret < 0)
		return ret;
	ret = snd_soc_component_update_bits(codec, WM8978_RIGHT_ADC_BOOST_CONTROL,
					    0x100, 0x100);
	return ret < 0 ? ret : 0;
}

static const struct snd_soc_ops n32_card_ops = {
	.hw_params = n32_card_hw_params,
};

static const struct snd_soc_dapm_widget n32_widgets[] = {
	SND_SOC_DAPM_MIC("Board Mic", NULL),
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
};

static const struct snd_soc_dapm_route n32_routes[] = {
	/* Keep MICBIAS in the capture path so DAPM powers the electret mic. */
	{ "Mic Bias", NULL, "Board Mic" },
	{ "LMICP", NULL, "Mic Bias" },
	{ "LMICN", NULL, "Mic Bias" },
	{ "RMICP", NULL, "Mic Bias" },
	{ "RMICN", NULL, "Mic Bias" },

	/*
	 * LOUT1/ROUT1 (LO1/RO1) go to the headphone contacts of J70 and
	 * LOUT2/ROUT2 (SPK_P/SPK_N) go to the two-pin speaker header J19.
	 */
	{ "Headphone", NULL, "LHP" },
	{ "Headphone", NULL, "RHP" },
	{ "Speaker", NULL, "LSPK" },
	{ "Speaker", NULL, "RSPK" },
};

static void n32_put_nodes(void *data)
{
	struct n32_audio_card *priv = data;

	of_node_put(priv->cpu.of_node);
	of_node_put(priv->codec.of_node);
}

static int n32_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct n32_audio_card *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->cpu.of_node = of_parse_phandle(dev->of_node, "audio-cpu", 0);
	priv->codec.of_node = of_parse_phandle(dev->of_node, "audio-codec", 0);
	ret = devm_add_action_or_reset(dev, n32_put_nodes, priv);
	if (ret)
		return ret;
	if (!priv->cpu.of_node || !priv->codec.of_node)
		return -EINVAL;
	ret = of_property_read_u32(dev->of_node, "nsing,mclk-frequency", &priv->mclk);
	if (ret)
		return ret;
	priv->cpu.dai_name = "n32h7xx-i2s";
	priv->codec.dai_name = "wm8978-hifi";
	priv->platform.of_node = priv->cpu.of_node;
	priv->link = (struct snd_soc_dai_link) {
		.name = "WM8978", .stream_name = "WM8978",
		.cpus = &priv->cpu, .num_cpus = 1,
		.codecs = &priv->codec, .num_codecs = 1,
		.platforms = &priv->platform, .num_platforms = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBP_CFP,
		.init = n32_card_init, .ops = &n32_card_ops,
	};
	priv->card = (struct snd_soc_card) {
		.name = "N32H787 WM8978", .owner = THIS_MODULE, .dev = dev,
		.dai_link = &priv->link, .num_links = 1,
		.dapm_widgets = n32_widgets, .num_dapm_widgets = ARRAY_SIZE(n32_widgets),
		.dapm_routes = n32_routes, .num_dapm_routes = ARRAY_SIZE(n32_routes),
	};
	snd_soc_card_set_drvdata(&priv->card, priv);
	return devm_snd_soc_register_card(dev, &priv->card);
}

static const struct of_device_id n32_card_match[] = {
	{ .compatible = "nsing,n32h787-wm8978" },
	{ }
};
MODULE_DEVICE_TABLE(of, n32_card_match);
static struct platform_driver n32_card_driver = {
	.probe = n32_card_probe,
	.driver = { .name = "n32h787-wm8978", .of_match_table = n32_card_match },
};
module_platform_driver(n32_card_driver);
MODULE_DESCRIPTION("N32H787 HMI WM8978 audio card");
MODULE_LICENSE("GPL");
