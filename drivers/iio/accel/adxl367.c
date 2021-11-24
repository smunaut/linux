// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include "adxl367.h"

#define ADXL367_REG_DEVID		0x00
#define ADXL367_DEVID_AD		0xAD

#define ADXL367_REG_STATUS		0x0B
#define ADXL367_STATUS_FIFO_FULL_MASK	BIT(2)
#define ADXL367_STATUS_IS_FIFO_FULL(x)	((x) & ADXL367_STATUS_FIFO_FULL_MASK)
#define ADXL367_STATUS_ACT_MASK		BIT(4)
#define ADXL367_STATUS_IS_ACT(x)	((x) & ADXL367_STATUS_ACT_MASK)
#define ADXL367_STATUS_INACT_MASK	BIT(5)
#define ADXL367_STATUS_IS_INACT(x)	((x) & ADXL367_STATUS_INACT_MASK)

#define ADXL367_REG_FIFO_ENT_L		0x0C
#define ADXL367_REG_FIFO_ENT_H		0x0D
#define ADXL367_FIFO_ENT_H_MASK		GENMASK(1, 0)

#define ADXL367_REG_X_DATA_H		0x0E
#define ADXL367_REG_X_DATA_L		0x0F
#define ADXL367_REG_Y_DATA_H		0x10
#define ADXL367_REG_Y_DATA_L		0x11
#define ADXL367_REG_Z_DATA_H		0x12
#define ADXL367_REG_Z_DATA_L		0x13
#define ADXL367_REG_TEMP_DATA_H		0x14
#define ADXL367_REG_TEMP_DATA_L		0x15
#define ADXL367_REG_EX_ADC_DATA_H	0x16
#define ADXL367_REG_EX_ADC_DATA_L	0x17

#define ADXL367_REG_RESET		0x1F
#define ADXL367_RESET_CODE		0x52

#define ADXL367_REG_THRESH_ACT_H	0x20
#define ADXL367_REG_THRESH_ACT_L	0x21
#define ADXL367_REG_THRESH_INACT_H	0x23
#define ADXL367_REG_THRESH_INACT_L	0x24
#define ADXL367_THRESH_MAX		GENMASK(12, 0)
#define ADXL367_THRESH_H_MASK		GENMASK(6, 0)
#define ADXL367_THRESH_L_MASK		GENMASK(7, 2)
#define ADXL367_THRESH_VAL_TO_H(x)	(((x) >> 6) & ADXL367_THRESH_H_MASK)
#define ADXL367_THRESH_VAL_TO_L(x)	(((x) << 2) & ADXL367_THRESH_L_MASK)

#define ADXL367_REG_TIME_ACT		0x22
#define ADXL367_REG_TIME_INACT_H	0x25
#define ADXL367_REG_TIME_INACT_L	0x26
#define ADXL367_TIME_ACT_MAX		GENMASK(7, 0)
#define ADXL367_TIME_INACT_MAX		GENMASK(15, 0)
#define ADXL367_TIME_INACT_VAL_TO_H(x)	(((x) >> 8) & 0xFF)
#define ADXL367_TIME_INACT_VAL_TO_L(x)	((x) & 0xFF)

#define ADXL367_REG_ACT_INACT_CTL	0x27
#define ADXL367_ACT_EN_MASK		GENMASK(1, 0)
#define ADXL367_ACT_LINKLOOP_MASK	GENMASK(5, 4)
#define ADXL367_ACT_LINKLOOP(x)		FIELD_PREP(ADXL367_ACT_LINKLOOP_MASK, x)

#define ADXL367_REG_FIFO_CTL		0x28
#define ADXL367_FIFO_CTL_FORMAT_MASK	GENMASK(6, 3)
#define ADXL367_FIFO_CTL_FORMAT(x)	FIELD_PREP(ADXL367_FIFO_CTL_FORMAT_MASK, x)
#define ADXL367_FIFO_CTL_MODE_MASK	GENMASK(2, 0)
#define ADXL367_FIFO_CTL_MODE(x)	FIELD_PREP(ADXL367_FIFO_CTL_MODE_MASK, x)
#define ADXL367_FIFO_CTL_SAMPLES_MASK	BIT(2)
#define ADXL367_FIFO_CTL_SAMPLES(x)	((((x) >> 8) & 0x1) << 2)

#define ADXL367_REG_FIFO_SAMPLES	0x29
#define ADXL367_FIFO_SAMPLES_L(x)	((x) & 0xFF)
#define ADXL367_FIFO_SIZE		512
#define ADXL367_FIFO_MAX_WATERMARK	511

#define ADXL367_REG_INT1_MAP		0x2A
#define ADXL367_INT_INACT_MASK		BIT(5)
#define ADXL367_INT_ACT_MASK		BIT(4)
#define ADXL367_INT_FIFO_FULL_MASK	BIT(2)

#define ADXL367_REG_FILTER_CTL		0x2C
#define ADXL367_FILTER_CTL_RANGE_MASK	GENMASK(7, 6)
#define ADXL367_FILTER_CTL_RANGE(x)	FIELD_PREP(ADXL367_FILTER_CTL_RANGE_MASK, x)
#define ADXL367_2G_RANGE_1G		4095
#define ADXL367_2G_RANGE_100MG		409
#define ADXL367_FILTER_CTL_ODR_MASK	GENMASK(2, 0)
#define ADXL367_FILTER_CTL_ODR(x)	FIELD_PREP(ADXL367_FILTER_CTL_ODR_MASK, x)

#define ADXL367_REG_POWER_CTL		0x2D
#define ADXL367_POWER_CTL_MODE_MASK	GENMASK(1, 0)
#define ADXL367_POWER_CTL_MODE(x)	FIELD_PREP(ADXL367_POWER_CTL_MODE_MASK, (x))

#define ADXL367_REG_ADC_CTL		0x3C
#define ADXL367_REG_TEMP_CTL		0x3D
#define ADXL367_ADC_EN_MASK		BIT(0)
#define ADXL367_ADC_EN(x)		FIELD_PREP(ADXL367_ADC_EN_MASK, x)

#define ADXL367_IIO_ACCEL_EVENT(ev_dir) 				\
	IIO_MOD_EVENT_CODE(IIO_ACCEL, 0, IIO_MOD_X_OR_Y_OR_Z,		\
			   IIO_EV_TYPE_THRESH, ev_dir)

enum adxl367_adc_mode {
	ADXL367_ADC_MODE_NONE,
	ADXL367_ADC_MODE_TEMP,
	ADXL367_ADC_MODE_EX,
};

enum adxl367_range {
	ADXL367_2G_RANGE,
	ADXL367_4G_RANGE,
	ADXL367_8G_RANGE,
};

enum adxl367_fifo_mode {
	ADXL367_FIFO_MODE_DISABLED = 0b00,
	ADXL367_FIFO_MODE_STREAM = 0b10,
};

enum adxl367_fifo_format {
	ADXL367_FIFO_FORMAT_XYZ,
	ADXL367_FIFO_FORMAT_X,
	ADXL367_FIFO_FORMAT_Y,
	ADXL367_FIFO_FORMAT_Z,
	ADXL367_FIFO_FORMAT_XYZT,
	ADXL367_FIFO_FORMAT_XT,
	ADXL367_FIFO_FORMAT_YT,
	ADXL367_FIFO_FORMAT_ZT,
	ADXL367_FIFO_FORMAT_XYZA,
	ADXL367_FIFO_FORMAT_XA,
	ADXL367_FIFO_FORMAT_YA,
	ADXL367_FIFO_FORMAT_ZA,
};

enum adxl367_op_mode {
	ADXL367_OP_STANDBY = 0b00,
	ADXL367_OP_MEASURE = 0b10,
};

enum adxl367_act_proc_mode {
	ADXL367_LOOPED = 0b11,
};

enum adxl367_act_en_mode {
	ADXL367_ACT_DISABLED = 0b00,
	ADCL367_ACT_REF_ENABLED = 0b11,
};

enum adxl367_activity_type {
	ADXL367_ACTIVITY,
	ADXL367_INACTIVITY,
};

enum adxl367_odr {
	ADXL367_ODR_12P5HZ,
	ADXL367_ODR_25HZ,
	ADXL367_ODR_50HZ,
	ADXL367_ODR_100HZ,
	ADXL367_ODR_200HZ,
	ADXL367_ODR_400HZ,
};

struct adxl367_state {
	const struct adxl367_ops	*ops;
	void				*context;

	struct device		*dev;
	struct regmap		*regmap;
	struct iio_trigger	*dready_trig;

	struct mutex		lock;

	enum adxl367_odr	odr;
	enum adxl367_range	range;
	enum adxl367_adc_mode	adc_mode;

	unsigned int	act_threshold;
	unsigned int	act_time_ms;
	unsigned int	inact_threshold;
	unsigned int	inact_time_ms;

	unsigned int	fifo_set_size;
	unsigned int	fifo_watermark;

	__be16		fifo_buf[ADXL367_FIFO_SIZE] ____cacheline_aligned;
	__be16		sample_buf ____cacheline_aligned;
	u8		status_buf[3] ____cacheline_aligned;
};

static const unsigned int adxl367_threshold_h_reg_tbl[] = {
	[ADXL367_ACTIVITY]   = ADXL367_REG_THRESH_ACT_H,
	[ADXL367_INACTIVITY] = ADXL367_REG_THRESH_INACT_H,
};

static const unsigned int adxl367_act_en_shift_tbl[] = {
	[ADXL367_ACTIVITY]   = 0,
	[ADXL367_INACTIVITY] = 2,
};

static const unsigned int adxl367_act_int_mask_tbl[] = {
	[ADXL367_ACTIVITY]   = ADXL367_INT_ACT_MASK,
	[ADXL367_INACTIVITY] = ADXL367_INT_INACT_MASK,
};

static const int adxl367_samp_freq_tbl[][2] = {
	[ADXL367_ODR_12P5HZ] = {12, 500000},
	[ADXL367_ODR_25HZ]   = {25, 0},
	[ADXL367_ODR_50HZ]   = {50, 0},
	[ADXL367_ODR_100HZ]  = {100, 0},
	[ADXL367_ODR_200HZ]  = {200, 0},
	[ADXL367_ODR_400HZ]  = {400, 0},
};

static const int adxl367_time_scale_tbl[] = {
	[ADXL367_ODR_12P5HZ] = 1,
	[ADXL367_ODR_25HZ]   = 2,
	[ADXL367_ODR_50HZ]   = 4,
	[ADXL367_ODR_100HZ]  = 8,
	[ADXL367_ODR_200HZ]  = 16,
	[ADXL367_ODR_400HZ]  = 32,
};

/* (g * 2) * 9.80665 * 1000000 / (2^14 - 1) */
static const int adxl367_range_scale_tbl[][2] = {
	[ADXL367_2G_RANGE] = {0, 2394347},
	[ADXL367_4G_RANGE] = {0, 4788695},
	[ADXL367_8G_RANGE] = {0, 9577391},
};

static const int adxl367_range_scale_factor_tbl[] = {
	[ADXL367_2G_RANGE] = 1,
	[ADXL367_4G_RANGE] = 2,
	[ADXL367_8G_RANGE] = 4,
};

static const unsigned long adxl367_channel_masks[] = {
	[ADXL367_FIFO_FORMAT_XYZ]  = BIT(0) | BIT(1) | BIT(2),
	[ADXL367_FIFO_FORMAT_X]    = BIT(0),
	[ADXL367_FIFO_FORMAT_Y]    = BIT(1),
	[ADXL367_FIFO_FORMAT_Z]    = BIT(2),
	[ADXL367_FIFO_FORMAT_XYZT] = BIT(0) | BIT(1) | BIT(2) | BIT(3),
	[ADXL367_FIFO_FORMAT_XT]   = BIT(0) | BIT(3),
	[ADXL367_FIFO_FORMAT_YT]   = BIT(1) | BIT(3),
	[ADXL367_FIFO_FORMAT_ZT]   = BIT(2) | BIT(3),
	[ADXL367_FIFO_FORMAT_XYZA] = BIT(0) | BIT(1) | BIT(2) | BIT(4),
	[ADXL367_FIFO_FORMAT_XA]   = BIT(0) | BIT(4),
	[ADXL367_FIFO_FORMAT_YA]   = BIT(1) | BIT(4),
	[ADXL367_FIFO_FORMAT_ZA]   = BIT(2) | BIT(4),
};

static int adxl367_set_measure_en(struct adxl367_state *st, bool en)
{
	enum adxl367_op_mode op_mode = en ? ADXL367_OP_MEASURE
					  : ADXL367_OP_STANDBY;
	int ret;

	ret = regmap_update_bits(st->regmap, ADXL367_REG_POWER_CTL,
				 ADXL367_POWER_CTL_MODE_MASK,
				 ADXL367_POWER_CTL_MODE(op_mode));
	if (ret)
		return ret;

	/*
	 * Wait for acceleration output to settle after entering
	 * measure mode.
	 */
	if (en)
		msleep(100);

	return 0;
}

static void adxl367_scale_act_thresholds(struct adxl367_state *st,
					 enum adxl367_range old_range,
					 enum adxl367_range new_range)
{
	st->act_threshold = st->act_threshold
			    * adxl367_range_scale_factor_tbl[old_range]
			    / adxl367_range_scale_factor_tbl[new_range];
	st->inact_threshold = st->inact_threshold
			      * adxl367_range_scale_factor_tbl[old_range]
			      / adxl367_range_scale_factor_tbl[new_range];
}

static int _adxl367_set_act_threshold(struct adxl367_state *st,
				      enum adxl367_activity_type act,
				      unsigned int threshold)
{
	u8 reg = adxl367_threshold_h_reg_tbl[act];
	struct reg_sequence reg_seq[] = {
		{ reg },
		{ reg + 1 },
	};
	int ret;

	if (threshold > ADXL367_THRESH_MAX)
		return -EINVAL;

	reg_seq[0].def = ADXL367_THRESH_VAL_TO_H(threshold);
	reg_seq[1].def = ADXL367_THRESH_VAL_TO_L(threshold);

	ret = regmap_multi_reg_write(st->regmap, reg_seq, ARRAY_SIZE(reg_seq));
	if (ret)
		return ret;

	switch (act) {
	case ADXL367_ACTIVITY:
		st->act_threshold = threshold;
		break;
	case ADXL367_INACTIVITY:
		st->inact_threshold = threshold;
		break;
	}

	return 0;
}

static int adxl367_set_act_threshold(struct adxl367_state *st,
				     enum adxl367_activity_type act,
				     unsigned int threshold)
{
	int ret;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = _adxl367_set_act_threshold(st, act, threshold);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_set_act_proc_mode(struct adxl367_state *st,
				     enum adxl367_act_proc_mode mode)
{
	return regmap_update_bits(st->regmap,
				  ADXL367_REG_ACT_INACT_CTL,
				  ADXL367_ACT_LINKLOOP_MASK,
				  ADXL367_ACT_LINKLOOP(mode));
}

static int adxl367_set_act_interrupt_en(struct adxl367_state *st,
					enum adxl367_activity_type act,
					bool en)
{
	unsigned int mask = adxl367_act_int_mask_tbl[act];

	return regmap_update_bits(st->regmap, ADXL367_REG_INT1_MAP,
				  mask, en ? mask : 0);
}

static int adxl367_get_act_interrupt_en(struct adxl367_state *st,
					enum adxl367_activity_type act,
					bool *en)
{
	unsigned int mask = adxl367_act_int_mask_tbl[act];
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, ADXL367_REG_INT1_MAP, &val);
	if (ret)
		return ret;

	*en = !!(val & mask);

	return 0;
}

static int adxl367_set_act_en(struct adxl367_state *st,
			      enum adxl367_activity_type act,
			      enum adxl367_act_en_mode en)
{
	unsigned int ctl_shift = adxl367_act_en_shift_tbl[act];

	return regmap_update_bits(st->regmap, ADXL367_REG_ACT_INACT_CTL,
				  ADXL367_ACT_EN_MASK << ctl_shift,
				  en << ctl_shift);
}

static int adxl367_set_fifo_full_interrupt_en(struct adxl367_state *st, bool en)
{
	return regmap_update_bits(st->regmap, ADXL367_REG_INT1_MAP,
				  ADXL367_INT_FIFO_FULL_MASK,
				  en ? ADXL367_INT_FIFO_FULL_MASK : 0);
}

static int adxl367_get_fifo_mode(struct adxl367_state *st,
				 enum adxl367_fifo_mode *fifo_mode)
{
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, ADXL367_REG_FIFO_CTL, &val);
	if (ret)
		return ret;

	*fifo_mode = FIELD_GET(ADXL367_FIFO_CTL_MODE_MASK, val);

	return 0;
}

static int adxl367_set_fifo_mode(struct adxl367_state *st,
				 enum adxl367_fifo_mode fifo_mode)
{
	return regmap_update_bits(st->regmap, ADXL367_REG_FIFO_CTL,
				  ADXL367_FIFO_CTL_MODE_MASK,
				  ADXL367_FIFO_CTL_MODE(fifo_mode));
}

static int adxl367_set_fifo_format(struct adxl367_state *st,
				   enum adxl367_fifo_format fifo_format)
{
	return regmap_update_bits(st->regmap, ADXL367_REG_FIFO_CTL,
				  ADXL367_FIFO_CTL_FORMAT_MASK,
				  ADXL367_FIFO_CTL_FORMAT(fifo_format));
}

static int adxl367_set_fifo_samples(struct adxl367_state *st,
				    unsigned int fifo_watermark,
				    unsigned int fifo_set_size)
{
	unsigned int fifo_samples = fifo_watermark * fifo_set_size;
	int ret;

	if (fifo_samples > ADXL367_FIFO_MAX_WATERMARK)
		fifo_samples = ADXL367_FIFO_MAX_WATERMARK;

	fifo_samples /= fifo_set_size;

	ret = regmap_update_bits(st->regmap, ADXL367_REG_FIFO_CTL,
				 ADXL367_FIFO_CTL_SAMPLES_MASK,
				 ADXL367_FIFO_CTL_SAMPLES(fifo_samples));
	if (ret)
		return ret;

	return regmap_write(st->regmap, ADXL367_REG_FIFO_SAMPLES,
			    ADXL367_FIFO_SAMPLES_L(fifo_samples));
}

static int adxl367_set_fifo_set_size(struct adxl367_state *st,
				     unsigned int fifo_set_size)
{
	int ret;

	ret = adxl367_set_fifo_samples(st, st->fifo_watermark, fifo_set_size);
	if (ret)
		return ret;

	st->fifo_set_size = fifo_set_size;

	return 0;
}

static int adxl367_set_fifo_watermark(struct adxl367_state *st,
				      unsigned int fifo_watermark)
{
	int ret;

	ret = adxl367_set_fifo_samples(st, fifo_watermark, st->fifo_set_size);
	if (ret)
		return ret;

	st->fifo_watermark = fifo_watermark;

	return 0;
}

static int adxl367_set_range(struct adxl367_state *st,
			     enum adxl367_range range)
{
	int ret;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = regmap_update_bits(st->regmap, ADXL367_REG_FILTER_CTL,
				 ADXL367_FILTER_CTL_RANGE_MASK,
				 ADXL367_FILTER_CTL_RANGE(range));
	if (ret)
		goto out;

	adxl367_scale_act_thresholds(st, st->range, range);

	/* Activity thresholds depend on range */
	ret = _adxl367_set_act_threshold(st, ADXL367_ACTIVITY,
					 st->act_threshold);
	if (ret)
		goto out;

	ret = _adxl367_set_act_threshold(st, ADXL367_INACTIVITY,
					 st->inact_threshold);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);
	if (ret)
		goto out;

	st->range = range;

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_time_ms_to_samples(struct adxl367_state *st, unsigned int ms)
{
	int freq_hz = adxl367_samp_freq_tbl[st->odr][0];
	int freq_microhz = adxl367_samp_freq_tbl[st->odr][1];
	/* Scale to decihertz to prevent precision loss in 12.5Hz case. */
	int freq_dhz = freq_hz * 10 + freq_microhz / 100000;

	return DIV_ROUND_CLOSEST(ms * freq_dhz, 10000);
}

static int _adxl367_set_act_time_ms(struct adxl367_state *st, unsigned int ms)
{
	unsigned int val = adxl367_time_ms_to_samples(st, ms);
	int ret;

	if (val > ADXL367_TIME_ACT_MAX)
		val = ADXL367_TIME_ACT_MAX;

	ret = regmap_write(st->regmap, ADXL367_REG_TIME_ACT, val);
	if (ret)
		return ret;

	st->act_time_ms = ms;

	return 0;
}

static int _adxl367_set_inact_time_ms(struct adxl367_state *st, unsigned int ms)
{
	struct reg_sequence reg_seq[] = {
		{ ADXL367_REG_TIME_INACT_H },
		{ ADXL367_REG_TIME_INACT_L },
	};
	unsigned int val = adxl367_time_ms_to_samples(st, ms);
	int ret;

	if (val > ADXL367_TIME_INACT_MAX)
		val = ADXL367_TIME_INACT_MAX;

	reg_seq[0].def = ADXL367_TIME_INACT_VAL_TO_H(val);
	reg_seq[1].def = ADXL367_TIME_INACT_VAL_TO_L(val);

	ret = regmap_multi_reg_write(st->regmap, reg_seq, ARRAY_SIZE(reg_seq));
	if (ret)
		return ret;

	st->inact_time_ms = ms;

	return 0;
}


static int adxl367_set_act_time_ms(struct adxl367_state *st,
				   enum adxl367_activity_type act,
				   unsigned int ms)
{
	int ret;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	switch (act) {
	case ADXL367_ACTIVITY:
		ret = _adxl367_set_act_time_ms(st, ms);
		break;
	case ADXL367_INACTIVITY:
		ret = _adxl367_set_inact_time_ms(st, ms);
		break;
	}

	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int _adxl367_set_odr(struct adxl367_state *st, enum adxl367_odr odr)
{
	int ret;

	ret = regmap_update_bits(st->regmap, ADXL367_REG_FILTER_CTL,
				 ADXL367_FILTER_CTL_ODR_MASK,
				 ADXL367_FILTER_CTL_ODR(odr));
	if (ret)
		return ret;

	/* Activity timers depend on ODR */
	ret = _adxl367_set_act_time_ms(st, st->act_time_ms);
	if (ret)
		return ret;

	ret = _adxl367_set_inact_time_ms(st, st->inact_time_ms);
	if (ret)
		return ret;

	st->odr = odr;

	return 0;
}

static int adxl367_set_odr(struct adxl367_state *st, enum adxl367_odr odr)
{
	int ret;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = _adxl367_set_odr(st, odr);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_set_adc_ex_en(struct adxl367_state *st, bool en)
{
	return regmap_update_bits(st->regmap, ADXL367_REG_ADC_CTL,
				  ADXL367_ADC_EN_MASK,
				  ADXL367_ADC_EN(en));
}

static int adxl367_set_adc_temp_en(struct adxl367_state *st, bool en)
{
	return regmap_update_bits(st->regmap, ADXL367_REG_TEMP_CTL,
				  ADXL367_ADC_EN_MASK,
				  ADXL367_ADC_EN(en));
}

static int adxl367_get_chan_type_adc_mode(enum iio_chan_type type,
					  enum adxl367_adc_mode *mode)
{
	switch (type) {
	case IIO_TEMP:
		*mode = ADXL367_ADC_MODE_TEMP;
		return 0;
	case IIO_VOLTAGE:
		*mode = ADXL367_ADC_MODE_EX;
		return 0;
	default:
		return -EINVAL;
	}
}

static int adxl367_set_adc_en(struct adxl367_state *st,
			      enum adxl367_adc_mode adc_mode, bool en)
{
	switch (adc_mode) {
	case ADXL367_ADC_MODE_TEMP:
		return adxl367_set_adc_temp_en(st, en);
	case ADXL367_ADC_MODE_EX:
		return adxl367_set_adc_ex_en(st, en);
	default:
		return 0;
	}
}

static int adxl367_set_adc_mode(struct adxl367_state *st,
				enum adxl367_adc_mode adc_mode)
{
	int ret;

	ret = adxl367_set_adc_en(st, st->adc_mode, false);
	if (ret)
		return ret;

	ret = adxl367_set_adc_en(st, adc_mode, true);
	if (ret)
		return ret;

	st->adc_mode = adc_mode;

	return 0;
}

static int adxl367_set_adc_mode_en(struct adxl367_state *st,
				   enum iio_chan_type type,
				   bool en)
{
	enum adxl367_adc_mode adc_mode;
	int ret;

	ret = adxl367_get_chan_type_adc_mode(type, &adc_mode);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	if (!en && st->adc_mode != adc_mode) {
		ret = -EINVAL;
		goto out;
	}

	if (!en)
		adc_mode = ADXL367_ADC_MODE_NONE;

	ret = adxl367_set_adc_mode(st, adc_mode);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_find_odr(struct adxl367_state *st, int val, int val2,
			    enum adxl367_odr *odr)
{
	size_t size = ARRAY_SIZE(adxl367_samp_freq_tbl);
	int i;

	for (i = 0; i < size; i++)
		if (val == adxl367_samp_freq_tbl[i][0] &&
		    val2 == adxl367_samp_freq_tbl[i][1])
			break;

	if (i == size)
		return -EINVAL;

	*odr = i;

	return 0;
}

static int adxl367_find_range(struct adxl367_state *st, int val, int val2,
			      enum adxl367_range *range)
{
	size_t size = ARRAY_SIZE(adxl367_range_scale_tbl);
	int i;

	for (i = 0; i < size; i++)
		if (val == adxl367_range_scale_tbl[i][0] &&
		    val2 == adxl367_range_scale_tbl[i][1])
			break;

	if (i == size)
		return -EINVAL;

	*range = i;

	return 0;
}

static int adxl367_read_sample(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	u16 sample;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	ret = regmap_bulk_read(st->regmap, chan->address, &st->sample_buf,
			       sizeof(st->sample_buf));
	sample = be16_to_cpu(st->sample_buf) >> chan->scan_type.shift;
	*val = sign_extend32(sample, chan->scan_type.realbits - 1);

	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);

	return ret ?: IIO_VAL_INT;
}

static int adxl367_get_status(struct adxl367_state *st, u8 *status,
			      u16 *fifo_entries)
{
	int ret;

	/* Read STATUS, FIFO_ENT_L and FIFO_ENT_H */
	ret = regmap_bulk_read(st->regmap, ADXL367_REG_STATUS,
			       st->status_buf, ARRAY_SIZE(st->status_buf));
	if (ret)
		return ret;

	st->status_buf[2] &= ADXL367_FIFO_ENT_H_MASK;

	*status = st->status_buf[0];
	*fifo_entries = get_unaligned_le16(&st->status_buf[1]);

	return 0;
}

static void adxl367_push_event(struct iio_dev *indio_dev, s64 timestamp,
			       u8 status)
{
	unsigned int ev_dir = IIO_EV_DIR_NONE;

	if (ADXL367_STATUS_IS_ACT(status))
		ev_dir = IIO_EV_DIR_RISING;

	if (ADXL367_STATUS_IS_INACT(status))
		ev_dir = IIO_EV_DIR_FALLING;

	if (ev_dir == IIO_EV_DIR_NONE)
		return;

	iio_push_event(indio_dev, ADXL367_IIO_ACCEL_EVENT(ev_dir), timestamp);
}

static irqreturn_t adxl367_trigger_handler(int irq, void  *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adxl367_state *st = iio_priv(indio_dev);
	u16 fifo_entries;
	u8 status;
	int ret;
	int i;

	ret = adxl367_get_status(st, &status, &fifo_entries);
	if (ret)
		goto out;

	adxl367_push_event(indio_dev, iio_get_time_ns(indio_dev), status);

	if (!ADXL367_STATUS_IS_FIFO_FULL(status))
		goto out;

	ret = st->ops->read_fifo(st->context, st->fifo_buf, fifo_entries);
	if (ret)
		goto out;

	for (i = 0; i < fifo_entries; i += st->fifo_set_size)
		iio_push_to_buffers(indio_dev, &st->fifo_buf[i]);

out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int adxl367_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adxl367_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static int adxl367_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct adxl367_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return adxl367_read_sample(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			mutex_lock(&st->lock);
			*val = adxl367_range_scale_tbl[st->range][0];
			*val2 = adxl367_range_scale_tbl[st->range][1];
			mutex_unlock(&st->lock);
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			*val = 1;
			*val2 = 54;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		*val = -165 + 25 * 54;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		*val = adxl367_samp_freq_tbl[st->odr][0];
		*val2 = adxl367_samp_freq_tbl[st->odr][1];
		mutex_unlock(&st->lock);
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->type) {
		case IIO_TEMP:
			mutex_lock(&st->lock);
			*val = st->adc_mode == ADXL367_ADC_MODE_TEMP;
			mutex_unlock(&st->lock);
			return IIO_VAL_INT;
		case IIO_VOLTAGE:
			mutex_lock(&st->lock);
			*val = st->adc_mode == ADXL367_ADC_MODE_EX;
			mutex_unlock(&st->lock);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int adxl367_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		enum adxl367_odr odr;

		ret = adxl367_find_odr(st, val, val2, &odr);
		if (ret)
			return ret;

		return adxl367_set_odr(st, odr);
	}
	case IIO_CHAN_INFO_SCALE: {
		enum adxl367_range range;

		if (chan->type != IIO_ACCEL)
			return -EINVAL;

		ret = adxl367_find_range(st, val, val2, &range);
		if (ret)
			return ret;

		return adxl367_set_range(st, range);
	}
	case IIO_CHAN_INFO_ENABLE:
		return adxl367_set_adc_mode_en(st, chan->type, val);
	default:
		return -EINVAL;
	}
}

int adxl367_write_raw_get_fmt(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;

		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}
}

static int adxl367_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;

		*vals = (int *)adxl367_range_scale_tbl;
		*type = IIO_VAL_INT_PLUS_NANO;
		*length = ARRAY_SIZE(adxl367_range_scale_tbl) * 2;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = (int *)adxl367_samp_freq_tbl;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(adxl367_samp_freq_tbl) * 2;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int adxl367_read_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int *val, int *val2)
{
	struct adxl367_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE: {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			mutex_lock(&st->lock);
			*val = st->act_threshold;
			mutex_unlock(&st->lock);
			return IIO_VAL_INT;
		case IIO_EV_DIR_FALLING:
			mutex_lock(&st->lock);
			*val = st->inact_threshold;
			mutex_unlock(&st->lock);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	}
	case IIO_EV_INFO_PERIOD:
		switch (dir) {
		case IIO_EV_DIR_RISING:
			mutex_lock(&st->lock);
			*val = st->act_time_ms;
			mutex_unlock(&st->lock);
			*val2 = 1000;
			return IIO_VAL_FRACTIONAL;
		case IIO_EV_DIR_FALLING:
			mutex_lock(&st->lock);
			*val = st->inact_time_ms;
			mutex_unlock(&st->lock);
			*val2 = 1000;
			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int adxl367_write_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int val, int val2)
{
	struct adxl367_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (val < 0)
			return -EINVAL;

		switch (dir) {
		case IIO_EV_DIR_RISING:
			return adxl367_set_act_threshold(st, ADXL367_ACTIVITY,
							 val);
		case IIO_EV_DIR_FALLING:
			return adxl367_set_act_threshold(st, ADXL367_INACTIVITY,
							 val);
		default:
			return -EINVAL;
		}
	case IIO_EV_INFO_PERIOD:
		if (val < 0)
			return -EINVAL;

		val = val * 1000 + DIV_ROUND_UP(val2, 1000);
		switch (dir) {
		case IIO_EV_DIR_RISING:
			return adxl367_set_act_time_ms(st, ADXL367_ACTIVITY,
						       val);
		case IIO_EV_DIR_FALLING:
			return adxl367_set_act_time_ms(st, ADXL367_INACTIVITY,
						       val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int adxl367_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	bool en;
	int ret;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		ret = adxl367_get_act_interrupt_en(st, ADXL367_ACTIVITY, &en);
		return ret ?: en;
	case IIO_EV_DIR_FALLING:
		ret = adxl367_get_act_interrupt_en(st, ADXL367_INACTIVITY, &en);
		return ret ?: en;
	default:
		return -EINVAL;
	}
}

static int adxl367_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	enum adxl367_activity_type act;
	int ret;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		act = ADXL367_ACTIVITY;
		break;
	case IIO_EV_DIR_FALLING:
		act = ADXL367_INACTIVITY;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = adxl367_set_act_interrupt_en(st, act, state);
	if (ret)
		goto out;

	ret = adxl367_set_act_en(st, act, state ? ADCL367_ACT_REF_ENABLED
						: ADXL367_ACT_DISABLED);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static ssize_t adxl367_get_fifo_enabled(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxl367_state *st = iio_priv(indio_dev);
	enum adxl367_fifo_mode fifo_mode;
	int ret;

	ret = adxl367_get_fifo_mode(st, &fifo_mode);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n",
		       fifo_mode != ADXL367_FIFO_MODE_DISABLED);
}

static ssize_t adxl367_get_fifo_watermark(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxl367_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->fifo_watermark);
}

static IIO_CONST_ATTR(hwfifo_watermark_min, "1");
static IIO_CONST_ATTR(hwfifo_watermark_max,
		      __stringify(ADXL367_FIFO_MAX_WATERMARK));
static IIO_DEVICE_ATTR(hwfifo_watermark, 0444,
		       adxl367_get_fifo_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_enabled, 0444,
		       adxl367_get_fifo_enabled, NULL, 0);

static const struct attribute *adxl367_fifo_attributes[] = {
	&iio_const_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	NULL,
};

static int adxl367_set_watermark(struct iio_dev *indio_dev, unsigned int val)
{
	struct adxl367_state *st  = iio_priv(indio_dev);
	int ret;

	if (val > ADXL367_FIFO_MAX_WATERMARK)
		return -EINVAL;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_watermark(st, val);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_find_mask_fifo_format(const unsigned long *scan_mask,
					 enum adxl367_fifo_format *fifo_format)
{
	size_t size = ARRAY_SIZE(adxl367_channel_masks);
	int i;

	for (i = 0; i < size; i++)
		if (*scan_mask == adxl367_channel_masks[i])
			break;

	if (i == size)
		return false;

	*fifo_format = i;

	return true;
}

static bool adxl367_validate_scan_mask(struct iio_dev *indio_dev,
				       const unsigned long *scan_mask)
{
	struct adxl367_state *st  = iio_priv(indio_dev);
	enum adxl367_adc_mode mode;

	mutex_lock(&st->lock);
	mode = st->adc_mode;
	mutex_unlock(&st->lock);

	if ((*scan_mask & BIT(3)) && mode != ADXL367_ADC_MODE_TEMP)
		return false;

	if ((*scan_mask & BIT(4)) && mode != ADXL367_ADC_MODE_EX)
		return false;

	return true;
}

static int adxl367_update_scan_mode(struct iio_dev *indio_dev,
				    const unsigned long *active_scan_mask)
{
	struct adxl367_state *st  = iio_priv(indio_dev);
	enum adxl367_fifo_format fifo_format;
	unsigned int fifo_set_size;
	int ret;

	if (!adxl367_find_mask_fifo_format(active_scan_mask, &fifo_format))
		return -EINVAL;

	fifo_set_size = bitmap_weight(active_scan_mask, indio_dev->masklength);

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_set_size(st, fifo_set_size);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_format(st, fifo_format);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_full_interrupt_en(st, true);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_mode(st, ADXL367_FIFO_MODE_STREAM);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	ret = adxl367_set_measure_en(st, false);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_mode(st, ADXL367_FIFO_MODE_DISABLED);
	if (ret)
		goto out;

	ret = adxl367_set_fifo_full_interrupt_en(st, false);
	if (ret)
		goto out;

	ret = adxl367_set_measure_en(st, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int adxl367_validate_trigger(struct iio_dev *indio_dev,
				    struct iio_trigger *trig)
{
	struct adxl367_state *st = iio_priv(indio_dev);

	if (st->dready_trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_buffer_setup_ops adxl367_buffer_ops = {
	.postenable = adxl367_buffer_postenable,
	.predisable = adxl367_buffer_predisable,
	.validate_scan_mask = adxl367_validate_scan_mask,
};

static const struct iio_trigger_ops adxl367_trigger_ops = {
	.validate_device = &iio_trigger_validate_own_device,
};

static const struct iio_info adxl367_info = {
	.validate_trigger = adxl367_validate_trigger,
	.read_raw = adxl367_read_raw,
	.write_raw = adxl367_write_raw,
	.write_raw_get_fmt = adxl367_write_raw_get_fmt,
	.read_avail = adxl367_read_avail,
	.read_event_config = adxl367_read_event_config,
	.write_event_config = adxl367_write_event_config,
	.read_event_value = adxl367_read_event_value,
	.write_event_value = adxl367_write_event_value,
	.debugfs_reg_access = adxl367_reg_access,
	.hwfifo_set_watermark = adxl367_set_watermark,
	.update_scan_mode = adxl367_update_scan_mode,
};

#define ADXL367_EVENT(_dir) {						\
	.type = IIO_EV_TYPE_THRESH,					\
	.dir = _dir,							\
	.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE) |			\
			      BIT(IIO_EV_INFO_PERIOD) |			\
			      BIT(IIO_EV_INFO_VALUE),			\
}

static const struct iio_event_spec adxl367_events[] = {
	ADXL367_EVENT(IIO_EV_DIR_RISING),
	ADXL367_EVENT(IIO_EV_DIR_FALLING),
};

#define ADXL367_14BIT_SCAN_INFO(index)					\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 14,						\
		.storagebits = 16,					\
		.shift = 2,						\
		.endianness = IIO_BE,					\
	}

#define ADXL367_ACCEL_CHANNEL(index, reg, axis) {				\
	.type = IIO_ACCEL,							\
	.address = reg,								\
	.modified = 1,								\
	.channel2 = IIO_MOD_##axis,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.event_spec = adxl367_events,						\
	.num_event_specs = ARRAY_SIZE(adxl367_events),				\
	ADXL367_14BIT_SCAN_INFO(index),						\
}

static const struct iio_chan_spec adxl367_channels[] = {
	ADXL367_ACCEL_CHANNEL(0, ADXL367_REG_X_DATA_H, X),
	ADXL367_ACCEL_CHANNEL(1, ADXL367_REG_Y_DATA_H, Y),
	ADXL367_ACCEL_CHANNEL(2, ADXL367_REG_Z_DATA_H, Z),
	{
		.type = IIO_TEMP,
		.address = ADXL367_REG_TEMP_DATA_H,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_OFFSET) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		ADXL367_14BIT_SCAN_INFO(3),
	},
	{
		.type = IIO_VOLTAGE,
		.address = ADXL367_REG_EX_ADC_DATA_H,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_ENABLE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		ADXL367_14BIT_SCAN_INFO(4),
	},
};

static int adxl367_setup(struct adxl367_state *st)
{
	unsigned int devid;
	int ret;

	ret = regmap_read(st->regmap, ADXL367_REG_DEVID, &devid);
	if (ret)
		return ret;

	if (devid != ADXL367_DEVID_AD) {
		dev_err(st->dev, "Invalid device id %x\n", devid);
		return -ENODEV;
	}

	ret = regmap_write(st->regmap, ADXL367_REG_RESET, ADXL367_RESET_CODE);
	if (ret)
		return ret;

	ret = _adxl367_set_act_threshold(st, ADXL367_ACTIVITY,
					 ADXL367_2G_RANGE_1G);
	if (ret)
		return ret;

	ret = _adxl367_set_act_threshold(st, ADXL367_ACTIVITY,
					 ADXL367_2G_RANGE_100MG);
	if (ret)
		return ret;

	ret = adxl367_set_act_proc_mode(st, ADXL367_LOOPED);
	if (ret)
		return ret;

	ret = _adxl367_set_odr(st, ADXL367_ODR_400HZ);
	if (ret)
		return ret;

	ret = _adxl367_set_act_time_ms(st, 10);
	if (ret)
		return ret;

	ret = _adxl367_set_inact_time_ms(st, 10000);
	if (ret)
		return ret;

	return adxl367_set_measure_en(st, true);
}

int adxl367_setup_irq(struct iio_dev *indio_dev, int irq)
{
	struct adxl367_state *st = iio_priv(indio_dev);
	int ret;

	if (!irq)
		return 0;

	st->dready_trig = devm_iio_trigger_alloc(st->dev, "%s-dev%d",
						 indio_dev->name,
						 indio_dev->id);
	if (st->dready_trig == NULL)
		return -ENOMEM;

	st->dready_trig->ops = &adxl367_trigger_ops;
	st->dready_trig->dev.parent = st->dev;
	iio_trigger_set_drvdata(st->dready_trig, indio_dev);

	ret = devm_iio_trigger_register(st->dev, st->dready_trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->dready_trig);

	return devm_request_threaded_irq(st->dev, irq,
					 iio_trigger_generic_data_rdy_poll,
					 NULL,
					 IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					 indio_dev->name, st->dready_trig);
}

int adxl367_probe(struct device *dev, const struct adxl367_ops *ops,
		  void *context, struct regmap *regmap, int irq,
		  const char *name)
{
	struct iio_dev *indio_dev;
	struct adxl367_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev = dev;
	st->regmap = regmap;
	st->context = context;
	st->ops = ops;

	mutex_init(&st->lock);

	indio_dev->channels = adxl367_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl367_channels);
	indio_dev->name = name;
	indio_dev->info = &adxl367_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;

	ret = adxl367_setup(st);
	if (ret) {
		dev_err(dev, "ADXL367 setup failed\n");
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(dev,
					      indio_dev, NULL,
					      adxl367_trigger_handler,
					      &adxl367_buffer_ops);
	if (ret)
		return ret;

	iio_buffer_set_attrs(indio_dev->buffer, adxl367_fifo_attributes);

	ret = adxl367_setup_irq(indio_dev, irq);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(adxl367_probe);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXL367 3-axis accelerometer driver");
MODULE_LICENSE("GPL");
