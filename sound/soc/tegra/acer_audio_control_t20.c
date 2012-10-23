/*
 * acer_audio_control.c - for WM8903 codec and fm2018 voice processor.
 *
 * Copyright (C) 2011 Acer, Inc.
 * Author: Andyl Liu <Andyl_Liu@acer.com.tw>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/kthread.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc-dapm.h>
#include <sound/soc-dai.h>
#include <sound/control.h>
#include <sound/tlv.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>

#include "acer_audio_control_t20.h"
#include "../codecs/wm8903.h"

#define AUDIO_CONTROL_DRIVER_NAME "acer-audio-control"

/* Enable log or not */
#if 1
#define ACER_DBG(fmt, arg...) printk(KERN_INFO "[AudioControl](%d): " fmt "\n", __LINE__, ## arg)
#else
#define ACER_DBG(fmt, arg...) do {} while (0)
#endif

/* Module function */
static int acer_audio_control_probe(struct platform_device *pdev);
static int acer_audio_control_remove(struct platform_device *pdev);

#ifdef CONFIG_ACER_FM_SINGLE_MIC
static int switch_audio_table_single(int control_mode, bool fromAP);
#else
static int switch_audio_table_dual(int control_mode, bool fromAP);
#endif

/* extern */
extern int getAudioTable(void);
extern void setAudioTable(int table_value);
extern int get_headset_state(void);

extern struct acer_audio_data audio_data;

enum headset_state {
	BIT_NO_HEADSET = 0,
	BIT_HEADSET = (1 << 0),
	BIT_HEADSET_NO_MIC = (1 << 1),
};

int snd_soc_dapm_get_iconia_param(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	const char *pin = (const char *)kcontrol->private_value;

	mutex_lock(&codec->mutex);

	/* TODO: get iconia param. */
	audio_data.pin = pin;

	mutex_unlock(&codec->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_get_iconia_param);

int snd_soc_dapm_put_iconia_param(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	const char *pin = (const char *)kcontrol->private_value;
	int is_mode_new = ucontrol->value.integer.value[0];

	mutex_lock(&codec->mutex);

	audio_data.pin = pin;

	switch_audio_table(is_mode_new, false);

	mutex_unlock(&codec->mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_put_iconia_param);

int snd_soc_dapm_info_iconia_param(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_dapm_info_iconia_param);

void set_voip_hp_gain(struct snd_soc_codec* codec)
{
#ifdef CONFIG_ACER_FM_SINGLE_MIC
	snd_soc_update_bits(audio_data.codec, WM8903_POWER_MANAGEMENT_3,
					WM8903_INL_ENA_MASK, WM8903_INL_ENA);
	snd_soc_update_bits(audio_data.codec, WM8903_POWER_MANAGEMENT_3,
					WM8903_INR_ENA_MASK, WM8903_INR_ENA);

	snd_soc_update_bits(audio_data.codec, WM8903_AUDIO_INTERFACE_0,
					WM8903_DAC_BOOST_MASK, R24_DAC_BOOST_6dB << WM8903_DAC_BOOST_SHIFT);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_LEFT_INPUT_0,
					WM8903_LIN_VOL_MASK, R44_HP_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
					WM8903_RIN_VOL_MASK, R44_HP_LRIN_VOL);
#else
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_LEFT_INPUT_0,
					WM8903_LIN_VOL_MASK, R44_SPK_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
					WM8903_RIN_VOL_MASK, R44_SPK_LRIN_VOL);
#endif
}

void set_voip_spk_gain(struct snd_soc_codec* codec)
{
#ifdef CONFIG_ACER_FM_SINGLE_MIC
	snd_soc_update_bits(audio_data.codec, WM8903_AUDIO_INTERFACE_0,
					WM8903_DAC_BOOST_MASK, R24_DAC_BOOST_0dB << WM8903_DAC_BOOST_SHIFT);
#endif
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_LEFT_INPUT_0,
					WM8903_LIN_VOL_MASK, R44_SPK_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
					WM8903_RIN_VOL_MASK, R44_SPK_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_OUT2_LEFT,
					WM8903_LINEOUTL_VOL_MASK, R59_SPK_LINEOUT_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_OUT2_RIGHT,
					WM8903_LINEOUTR_VOL_MASK, R59_SPK_LINEOUT_VOL);
}

void set_cts_spk_gain(struct snd_soc_codec* codec)
{
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_LEFT_INPUT_0,
					WM8903_LIN_VOL_MASK, R44_CTS_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
					WM8903_RIN_VOL_MASK, R44_CTS_LRIN_VOL);
}

void set_asr_spk_gain(struct snd_soc_codec* codec)
{
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_LEFT_INPUT_0,
					WM8903_LIN_VOL_MASK, R44_ASR_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
					WM8903_RIN_VOL_MASK, R44_ASR_LRIN_VOL);

	snd_soc_update_bits(audio_data.codec, WM8903_DRC_2,
					WM8903_DRC_R0_SLOPE_COMP_MASK,
					R42_COMPRESSOR_SLOP_R0 << WM8903_DRC_R0_SLOPE_COMP_SHIFT);

	snd_soc_update_bits(audio_data.codec, WM8903_DRC_3,
					WM8903_DRC_THRESH_COMP_MASK,
					R43_COMPRESSOR_THRESSHOLD_T << WM8903_DRC_THRESH_COMP_SHIFT);
	snd_soc_update_bits(audio_data.codec, WM8903_DRC_3,
					WM8903_DRC_AMP_COMP_MASK,
					R43_COMPRESSOR_THRESSHOLD_YT << WM8903_DRC_AMP_COMP_SHIFT);
}

void set_drc_gain(struct snd_soc_codec* codec)
{
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_LEFT_INPUT_0,
					WM8903_LIN_VOL_MASK, R44_SPK_LRIN_VOL);
	snd_soc_update_bits(audio_data.codec, WM8903_ANALOGUE_RIGHT_INPUT_0,
					WM8903_RIN_VOL_MASK, R44_SPK_LRIN_VOL);

	snd_soc_update_bits(audio_data.codec, WM8903_DRC_2,
					WM8903_DRC_R0_SLOPE_COMP_MASK,
					R42_COMPRESSOR_SLOP_R0 << WM8903_DRC_R0_SLOPE_COMP_SHIFT);

	snd_soc_update_bits(audio_data.codec, WM8903_DRC_3,
					WM8903_DRC_THRESH_COMP_MASK,
					R43_COMPRESSOR_THRESSHOLD_T << WM8903_DRC_THRESH_COMP_SHIFT);
	snd_soc_update_bits(audio_data.codec, WM8903_DRC_3,
					WM8903_DRC_AMP_COMP_MASK,
					R43_COMPRESSOR_THRESSHOLD_YT << WM8903_DRC_AMP_COMP_SHIFT);
}

int tune_codec_setting(int control_mode)
{
	int state = get_headset_state();
	audio_data.mode.control = control_mode;

	switch (audio_data.mode.control) {
		case VOICE_COMMUNICATION: /* For VOIP */
			if (state == BIT_HEADSET)
				set_voip_hp_gain(audio_data.codec);
			else
				set_voip_spk_gain(audio_data.codec);
			break;

		case VOICE_RECOGNITION: /* For CTS */
			if (state == BIT_HEADSET_NO_MIC)
				set_cts_spk_gain(audio_data.codec);
			else
				set_asr_spk_gain(audio_data.codec);
			break;

		case CAMCORDER:
			set_drc_gain(audio_data.codec);
			break;

		case MIC: /* For RECORD */
		case DEFAULT:
			set_drc_gain(audio_data.codec);
			break;
	}

	return 1;
}

#ifdef CONFIG_ACER_FM_SINGLE_MIC
void set_int_mic_state(bool state)
{
#ifdef CONFIG_MACH_PICASSO_E
	/*
	 * Add for solve that recording has high frequency noise.
	 * When internal MIC is on, disable CABC.
	 * When internal MIC is off, enable CABC.
	 */
	if (state) {
		setAudioCABC(0);
	} else {
		setAudioCABC(1);
	}
#endif

	audio_data.state.int_mic = state;
}

void set_ext_mic_state(bool state)
{
	audio_data.state.ext_mic = state;
}

static void fm2018_switch(struct tegra_asoc_platform_data *pdata)
{
	if (!audio_data.state.int_mic && !audio_data.state.ext_mic)
		gpio_set_value_cansleep(pdata->gpio_int_mic_en, 0);
	else
		gpio_set_value_cansleep(pdata->gpio_int_mic_en, 1);

	ACER_DBG("FM2018_EN = %d", gpio_get_value_cansleep(audio_data.gpio.int_mic_en));
}

void mic_switch(struct tegra_asoc_platform_data *pdata)
{
	switch_audio_table(audio_data.mode.control, false);
	fm2018_switch(pdata);
}

static int switch_audio_table_single(int control_mode, bool fromAP)
{
	int state = get_headset_state();
	bool state_change = false;
	audio_data.mode.control = control_mode;

	if ((audio_data.state.old != state) && audio_data.AP_Lock) {
		state_change = true;
		audio_data.state.old = state;
		if(audio_data.mode.ap_control != ACOUSTIC_DEVICE_MIC_RECORDING_TABLE)
			control_mode = audio_data.mode.ap_control;
	}

	if (!audio_data.AP_Lock && !fromAP) {
		switch (audio_data.mode.control) {
			case VOICE_COMMUNICATION: /* For VOIP */
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_VOIP_TABLE;
				else
					audio_data.table.input = ACOUSTIC_DEVICE_MIC_VOIP_TABLE;
				break;

			case VOICE_RECOGNITION: /* For CTS */
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_RECORDING_TABLE;
				else if (state == BIT_HEADSET_NO_MIC)
					audio_data.table.input = ACOUSTIC_CTS_VERIFIER_TABLE;
				else
					audio_data.table.input = ACOUSTIC_SPEECH_RECOGNITION_TABLE;
				break;

			case CAMCORDER:
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_RECORDING_TABLE;
				else
					audio_data.table.input = ACOUSTIC_CAMCORDER_RECORDER_TABLE;
				break;

			case MIC: /* For RECORD */
			case DEFAULT:
			default:
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_RECORDING_TABLE;
				else
					audio_data.table.input = ACOUSTIC_DEVICE_MIC_RECORDING_TABLE;
		}
	}

	if (fromAP || state_change) {
		audio_data.mode.ap_control = control_mode;

		switch (control_mode) {
			case ACOUSTIC_DEVICE_MIC_MUSIC_RECOGNITION_TABLE: /* For MusicA or SoundHound music recognition */
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_MUSIC_RECOGNITION_TABLE;
				else
					audio_data.table.input = control_mode;
				break;

			case ACOUSTIC_CAMCORDER_RECORDER_TABLE: /* For Recorder */
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_RECORDING_TABLE;
				else
					audio_data.table.input = control_mode;
				break;

			case ACOUSTIC_SPEECH_RECOGNITION_TABLE: /* For MusicA or SoundHound voice recognition */
				if (state == BIT_HEADSET)
					audio_data.table.input = ACOUSTIC_HEADSET_MIC_RECORDING_TABLE;
				else
					audio_data.table.input = control_mode;
				break;

			default:
				audio_data.table.input = control_mode;
				ACER_DBG("Do not re-assign audio table!");
				break;
		}

		if (audio_data.mode.ap_control == ACOUSTIC_DEVICE_MIC_RECORDING_TABLE)
			audio_data.AP_Lock = false;
		else
			audio_data.AP_Lock = true;
	}

	if (audio_data.state.int_mic || audio_data.state.ext_mic) {
		ACER_DBG("audio source = %d, pre_table = %d, cur_table %d!",
				audio_data.mode.input_source,
				getAudioTable(), audio_data.table.input);
		setAudioTable(audio_data.table.input);
	}

	return 1;
}
#else
static int switch_audio_table_dual(int control_mode, bool fromAP)
{
	audio_data.mode.control = control_mode;

	switch (audio_data.mode.control) {
		case VOICE_COMMUNICATION: /* For VOIP */
			audio_data.table.input = ACOUSTIC_DEVICE_MIC_VOIP_TABLE;
			break;

		case VOICE_RECOGNITION: /* For CTS */
			audio_data.table.input = ACOUSTIC_SPEECH_RECOGNITION_TABLE;
			break;

		case CAMCORDER:
			audio_data.table.input = ACOUSTIC_REAR_CAMCORDER_TABLE;
			break;

		case MIC: /* For RECORD */
		case DEFAULT:
		default:
			audio_data.table.input = ACOUSTIC_DEVICE_MIC_RECORDING_TABLE;
	}

	if (audio_data.state.int_mic || audio_data.state.ext_mic) {
		ACER_DBG("audio source = %d, pre_table = %d, cur_table %d!",
				audio_data.mode.input_source,
				getAudioTable(), audio_data.table.input);
		setAudioTable(audio_data.table.input);
	}
	return 1;
}
#endif

int switch_audio_table(int control_mode, bool fromAP)
{
	if (!fromAP) {
		audio_data.mode.input_source = control_mode;
	}
	tune_codec_setting(control_mode);

#ifdef CONFIG_ACER_FM_SINGLE_MIC
	return switch_audio_table_single(control_mode, fromAP);
#else
	return switch_audio_table_dual(control_mode, fromAP);
#endif
}

void acer_volume_setting(struct snd_soc_codec *codec, struct snd_pcm_substream *substream)
{
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_LEFT, HPOUT_VOL);
		snd_soc_write(codec, WM8903_ANALOGUE_OUT1_RIGHT, HPOUT_VOL);

		snd_soc_write(codec, WM8903_ANALOGUE_OUT2_LEFT, LINEOUT_VOL);
		snd_soc_write(codec, WM8903_ANALOGUE_OUT2_RIGHT, LINEOUT_VOL);

#ifdef CONFIG_ACER_FM_SINGLE_MIC
		snd_soc_update_bits(codec, WM8903_AUDIO_INTERFACE_0,
						WM8903_DAC_BOOST_MASK, R24_DAC_BOOST_0dB << WM8903_DAC_BOOST_SHIFT);
#endif
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		snd_soc_update_bits(codec, WM8903_DRC_1,
						WM8903_DRC_MAXGAIN_MASK, R41_DRC_MAXGAIN_38dB << WM8903_DRC_MAXGAIN_SHIFT);

		snd_soc_update_bits(audio_data.codec, WM8903_DRC_2,
						WM8903_DRC_R0_SLOPE_COMP_MASK,
						R42_COMPRESSOR_SLOP_DEFAULT_R0 << WM8903_DRC_R0_SLOPE_COMP_SHIFT);

		snd_soc_update_bits(audio_data.codec, WM8903_DRC_3,
						WM8903_DRC_THRESH_COMP_MASK,
						R43_COMPRESSOR_THRESSHOLD_DEFAULT_T << WM8903_DRC_THRESH_COMP_SHIFT);
		snd_soc_update_bits(audio_data.codec, WM8903_DRC_3,
						WM8903_DRC_AMP_COMP_MASK,
						R43_COMPRESSOR_THRESSHOLD_DEFAULT_YT << WM8903_DRC_AMP_COMP_SHIFT);
	}
}

/* platform driver register */
static struct platform_driver acer_audio_control_driver = {
	.probe  = acer_audio_control_probe,
	.remove = acer_audio_control_remove,
	.driver = {
		.name = AUDIO_CONTROL_DRIVER_NAME
	},
};

/* platform device register */
static struct platform_device acer_audio_control_device = {
	.name = AUDIO_CONTROL_DRIVER_NAME,
};

static int acer_audio_control_probe(struct platform_device *pdev)
{
	audio_data.table.input = ACOUSTIC_DEVICE_MIC_RECORDING_TABLE;
	audio_data.mode.control = DEFAULT;

	audio_data.state.int_mic = false;
	audio_data.state.ext_mic = false;
	audio_data.state.old = BIT_NO_HEADSET;
	audio_data.mode.ap_control = ACOUSTIC_DEVICE_MIC_RECORDING_TABLE;
	audio_data.AP_Lock = false;
	pr_info("[AudioControl] probe done.\n");
	return 0;
}

static int acer_audio_control_remove(struct platform_device *pdev)
{
	ACER_DBG("%s\n", __func__);
	return 0;
}

static int __init acer_audio_control_init(void)
{
	int ret;
	ACER_DBG("%s\n", __func__);
	ret = platform_driver_register(&acer_audio_control_driver);
	if (ret) {
		pr_err("[acer_audio_control_driver] failed to register!!\n");
		return ret;
	}
	return platform_device_register(&acer_audio_control_device);
}

static void __exit acer_audio_control_exit(void)
{
	platform_device_unregister(&acer_audio_control_device);
	platform_driver_unregister(&acer_audio_control_driver);
}

module_init(acer_audio_control_init);
module_exit(acer_audio_control_exit);

MODULE_DESCRIPTION("ACER AUDIO CONTROL DRIVER");
MODULE_LICENSE("GPL");
