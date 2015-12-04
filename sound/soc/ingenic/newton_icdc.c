 /*
 * Copyright (C) 2014 Ingenic Semiconductor Co., Ltd.
 *	http://www.ingenic.com
 * Author: cli <chen.li@ingenic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/gpio.h>
#include "icodec/icdc_d1.h"

static struct snd_soc_ops newton_i2s_ops = {

};

static const struct snd_soc_dapm_widget newton_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

static struct snd_soc_jack newton_icdc_d1_hp_jack;
static struct snd_soc_jack_pin newton_icdc_d1_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

/* newton machine audio_map */
static const struct snd_soc_dapm_route audio_map[] = {
	/* headphone connected to AOHPL/R */
	{"Headphone Jack", NULL, "AOHPL"},
	{"Headphone Jack", NULL, "AOHPR"},
};

static int newton_dlv_dai_link_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = rtd->card;
	int err;

	err = snd_soc_dapm_new_controls(dapm, newton_dapm_widgets,
			ARRAY_SIZE(newton_dapm_widgets));
	if (err)
		return err;

	err = snd_soc_dapm_add_routes(dapm, audio_map,
			ARRAY_SIZE(audio_map));
	if (err)
		return err;

	icdc_d1_hp_detect(codec, NULL, SND_JACK_HEADPHONE);
	snd_soc_dapm_force_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_sync(dapm);
	return 0;
}

static struct snd_soc_dai_link newton_dais[] = {
	[0] = {
		.name = "NEWTON ICDC",
		.stream_name = "NEWTON ICDC",
		.platform_name = "jz-asoc-aic-dma",
		.cpu_dai_name = "jz-asoc-aic-i2s",
		.init = newton_dlv_dai_link_init,
		.codec_dai_name = "icdc-d1-hifi",
		.codec_name = "icdc-d1",
		.ops = &newton_i2s_ops,
	},
	[1] = {
		.name = "NEWTON PCMBT",
		.stream_name = "NEWTON PCMBT",
		.platform_name = "jz-asoc-pcm-dma",
		.cpu_dai_name = "jz-asoc-pcm",
		.codec_dai_name = "dump dai",
		.codec_name = "dump",
	},
};

static struct snd_soc_card newton = {
	.name = "newton",
	.owner = THIS_MODULE,
	.dai_link = newton_dais,
	.num_links = ARRAY_SIZE(newton_dais),
};

static int snd_newton_probe(struct platform_device *pdev)
{
	int ret = 0;
	newton.dev = &pdev->dev;
	ret = snd_soc_register_card(&newton);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed %d\n", ret);
	return ret;
}

static int snd_newton_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&newton);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver snd_newton_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ingenic-newton",
		.pm = &snd_soc_pm_ops,
	},
	.probe = snd_newton_probe,
	.remove = snd_newton_remove,
};
module_platform_driver(snd_newton_driver);

MODULE_AUTHOR("cli<chen.li@ingenic.com>");
MODULE_DESCRIPTION("ALSA SoC Dorado Snd Card");
MODULE_LICENSE("GPL");
