// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * ADAR3000, ADAR3001, ADAR3002, ADAR3003 device driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/iio/sysfs.h>
#include "adar300x.h"

#define ADAR300x_REG_SPI_CONFIG			0x00
#define ADAR300x_REG_1				0x01
#define ADAR300x_REG_CHIPTYPE			0x03
#define ADAR300x_REG_PRODUCT_ID_L		0x04
#define ADAR300x_REG_PRODUCT_ID_H		0x05
#define ADAR300x_REG_ADDRESS_PAGE		0x08
#define ADAR300x_REG_SCRATCHPAD			0x0A
#define ADAR300x_REG_SPI_REV			0x0B
#define	ADAR300x_REG_BEAM0_MAP			0x10
#define	ADAR300x_REG_BEAM1_MAP			0x11
#define	ADAR300x_REG_BEAM2_MAP			0x12
#define	ADAR300x_REG_BEAM3_MAP			0x13
#define ADAR300x_REG_BEAMFORMER_MODE		0x14
#define ADAR300x_REG_BEAMSTATE_MODE		0x15
#define ADAR300x_REG_BEAM_SLEEP			0x16
#define ADAR300x_REG_MEM_SEQPTR(x)		(0x17 + x)
#define ADAR300x_REG_MEM_SEQPTR0_START		0x17
#define ADAR300x_REG_MEM_SEQPTR0_STOP		0x18
#define ADAR300x_REG_MEM_SEQPTR1_START		0x19
#define ADAR300x_REG_MEM_SEQPTR1_STOP		0x1A
#define ADAR300x_REG_MEM_SEQPTR2_START		0x1B
#define ADAR300x_REG_MEM_SEQPTR2_STOP		0x1C
#define ADAR300x_REG_MEM_SEQPTR3_START		0x1D
#define ADAR300x_REG_MEM_SEQPTR3_STOP		0x1E
#define ADAR300x_REG_ADC_CONTROL		0x20
#define ADAR300x_REG_ADC_CONTROL2		0x21
#define ADAR300x_REG_ADC_DATA_OUT		0x22
#define ADAR300x_REG_DAC_DATA_MSB		0x23
#define ADAR300x_REG_DAC_DATA_LSB		0x24
#define ADAR300x_REG_DAC_CONTROL		0x25
#define ADAR300x_REG_PIN_OR_SPI_CTL		0x30
#define ADAR300x_REG_BEAMWISE_UPDATE_CODE	0x32
#define ADAR300x_REG_BEAMWISE_UPDATE		0x33

#define ADAR300x_REG_FIFO_POINTER(x)		(0x50 + x)
#define ADAR300x_REG_FIFO_WRITE_POINTER0	0x50
#define ADAR300x_REG_FIFO_READ_POINTER0		0x51
#define ADAR300x_REG_FIFO_WRITE_POINTER1	0x52
#define ADAR300x_REG_FIFO_READ_POINTER1		0x53
#define ADAR300x_REG_FIFO_WRITE_POINTER2	0x54
#define ADAR300x_REG_FIFO_READ_POINTER2		0x55
#define ADAR300x_REG_FIFO_WRITE_POINTER3	0x56
#define ADAR300x_REG_FIFO_READ_POINTER3		0x57

#define ADAR3002_REG_RESET(x)			(0x080 + x)
#define ADAR3002_REG_MUTE(x)			(0x0A0 + x)
#define ADAR300x_REG_AMP_BIAS(x)		(0xC0 + x)
#define ADAR3002_REG_DRCT_CNTRL(x)		(0x100 + x)

/* ADAR3002 direct control registers H/V BEAM0 to BEAM1 */
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL0H(x)	(0x100 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL0H(x)	(0x101 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL1H(x)	(0x102 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL1H(x)	(0x103 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL2H(x)	(0x104 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL2H(x)	(0x105 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL3H(x)	(0x106 + (x) * (0x03 << 3))
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL3H(x)	(0x107 + (x) * (0x03 << 3))

#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL0V(x)	(0x108 + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL0V(x)	(0x109 + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL1V(x)	(0x10A + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL1V(x)	(0x10B + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL2V(x)	(0x10C + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL2V(x)	(0x10D + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_DELAY_EL3V(x)	(0x10E + (x) * 0x08)
#define ADAR3002_REG_DRCT_CNTRL_ATTEN_EL3V(x)	(0x10F + (x) * 0x08)

/* ADAR3003 direct control registers only one BEAM from EL0 to EL3 */
#define ADAR3003_REG_DRCT_CNTRL_DELAY_ELV(x)	(0x100 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_ATTN_ELV(x)	(0x101 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_DELAY_ELH(x)	(0x102 + (x) * 0x08)
#define ADAR3003_REG_DRCT_CNTRL_ATTN_ELH(x)	(0x103 + (x) * 0x08)


#define ADAR300x_REG_BM0_SEQ_PTR(x)	(0x200 + x)
#define ADAR300x_REG_BM1_SEQ_PTR(x)	(0x240 + x)
#define ADAR300x_REG_BM2_SEQ_PTR(x)	(0x280 + x)
#define ADAR300x_REG_BM3_SEQ_PTR(x)	(0x2C0 + x)

#define ADAR300x_REG_RESET_BM0_STREAM_IN(x)	(0x300 + x)
#define ADAR300x_REG_RESET_BM1_STREAM_IN(x)	(0x306 + x)
#define ADAR300x_REG_RESET_BM2_STREAM_IN(x)	(0x30C + x)
#define ADAR300x_REG_RESET_BM3_STREAM_IN(x)	(0x312 + x)

#define ADAR300x_REG_MUTE_BM0_STREAM_IN(x)	(0x318 + x)
#define ADAR300x_REG_MUTE_BM1_STREAM_IN(x)	(0x31E + x)
#define ADAR300x_REG_MUTE_BM2_STREAM_IN(x)	(0x324 + x)
#define ADAR300x_REG_MUTE_BM3_STREAM_IN(x)	(0x32A + x)

#define ADAR300x_REG_DRCT_CNTRL_BM0_STREAM_IN(x)	(0x330 + x)
#define ADAR300x_REG_DRCT_CNTRL_BM1_STREAM_IN(x)	(0x336 + x)
#define ADAR300x_REG_DRCT_CNTRL_BM2_STREAM_IN(x)	(0x33C + x)
#define ADAR300x_REG_DRCT_CNTRL_BM3_STREAM_IN(x)	(0x342 + x)

/* Beam state RAM
 * state - beam state number 0 - 63
 */
#define ADAR300x_RAM_BEAM_STATE_ADDR(state)		(0x100 + (state * 6))
#define ADAR300x_RAM_MAX_ADDR				0x27F

/* Beam state FIFO load
 * beam - position in memory location 0 - 3
 */
#define ADAR300x_FIFO_LOAD(beam)			(0x100 + (beam * 0x10))

/* ADAR300x_REG_SPI_CONFIG */
#define ADAR300x_SPI_CONFIG_RESET_	BIT(7)
#define ADAR300x_SPI_CONFIG_BIG_ENDIAN_	BIT(5)
#define ADAR300x_SPI_CONFIG_SDOACTIVE_	BIT(4)
#define ADAR300x_SPI_CONFIG_SDOACTIVE	BIT(3)
#define ADAR300x_SPI_CONFIG_BIG_ENDIAN	BIT(2)
#define ADAR300x_SPI_CONFIG_RESET	BIT(0)

/* ADAR300x_REG_ADC_CONTROL */
#define ADAR300x_REG_ADC_CONTROL_RESET		BIT(7)
#define ADAR300x_REG_ADC_CONTROL_CLK_EN		BIT(5)
#define ADAR300x_REG_ADC_CONTROL_EN		BIT(4)
#define ADAR300x_REG_ADC_CONTROL_MUX_SEL_MSK	GENMASK(2, 0)

/* ADAR300x_REG_ADC_CONTROL2 */
#define ADAR300x_REG_ADC_CONTROL2_START		BIT(0)
#define ADAR300x_REG_ADC_CONTROL2_END_CONV	BIT(4)

/* ADAR300x_REG_BEAMSTATE_MODE */
#define ADAR300x_MODE0	0x03
#define ADAR300x_MODE1	0x0C
#define ADAR300x_MODE2	0x30
#define ADAR300x_MODE3	0xC0

#define ADAR300x_ADDRESS_PAGE_MASK	0x0F
#define ADAR300x_SPI_ADDR_MSK		GENMASK(13, 10)
#define ADAR300x_SPI_ADDR(x)		FIELD_PREP(ADAR300x_SPI_ADDR_MSK, x)
#define ADAR300x_REG(st, x)		((st)->dev_addr | (x))

#define ADAR300x_PACKED_BEAMSTATE_LEN	6
#define ADAR300x_UNPACKED_BEAMSTATE_LEN	8
#define ADAR300x_MAX_RAM_STATES	64
#define ADAR300x_MAX_FIFO_STATES	16
#define ADAR300x_MAX_DEV		16
#define ADAR300x_MAX_RAW		0x3f
#define ADAR300x_MAX_GAIN_dB		31
#define ADAR300x_MAX_PHASE_DEGREE	360
#define ADAR300x_CHIP_TYPE		0x01
#define ADAR300x_ADC_NUM_CLOCKS		7

enum adar300x_ADC_sel {
	ADAR300x_ADC_ANALG0,
	ADAR300x_ADC_ANLG1,
	ADAR300x_ADC_TEMPERATURE,
};

enum adar300x_pages {
	ADAR300x_CONFIG_PAGE,
	ADAR300x_BEAM0H_PAGE,
	ADAR300x_BEAM0V_PAGE,
	ADAR300x_BEAM1H_PAGE,
	ADAR300x_BEAM1V_PAGE,
	ADAR300x_FIFO_PAGE,
};

enum adar300x_update_intf_control {
	ADAR300x_UPDATE_PIN_CONTROL,
	ADAR300x_UPDATE_SPI_CONTROL,
};

static const char *const adar300x_update_intf_ctrl[] = {
	[ADAR300x_UPDATE_PIN_CONTROL] = "pin",
	[ADAR300x_UPDATE_SPI_CONTROL] = "SPI",
};

static const char *const adar300x_mode_ctrl[] = {
	[ADAR300x_DIRECT_CTRL] = "direct",
	[ADAR300x_MEMORY_CTRL] = "memory",
	[ADAR300x_FIFO_CTRL] = "fifo",
	[ADAR300x_INST_DIRECT_CTRL] = "instant_direct",
	[ADAR300x_MUTE] = "mute",
	[ADAR300x_RESET] = "reset"
};

static const struct regmap_config adar300x_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int adar300x_reg_access(struct iio_dev *indio_dev, u32 reg, u32 writeval, u32 *readval)
{
	struct adar300x_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, ADAR300x_REG(st, reg), readval);
	else
		return regmap_write(st->regmap, ADAR300x_REG(st, reg), writeval);
}

static int adar300x_adc_read(struct adar300x_state *st, u32 *value)
{
	u32 val;
	int ret, i;

	/* Start conversion */
	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL2),
				 ADAR300x_REG_ADC_CONTROL2_START, 0);
	if (ret < 0)
		return ret;
	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL2),
				 ADAR300x_REG_ADC_CONTROL2_START, 0);
	if (ret < 0)
		return ret;
	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL2),
				 ADAR300x_REG_ADC_CONTROL2_START, 1);
	if (ret < 0)
		return ret;

	/* Generate clocks */
	for (i = 0; i < ADAR300x_ADC_NUM_CLOCKS; i++) {
		ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_SCRATCHPAD), 0xEA);
		if (ret < 0)
			return ret;
	}

	/* Check conversion ready */
	ret = regmap_read_poll_timeout(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL2), val,
				       val & ADAR300x_REG_ADC_CONTROL2_END_CONV, 100, 50000);
	if (ret < 0)
		return ret;

	/* Read result */
	return regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_DATA_OUT), value);
}

static int adar300x_adc_setup(struct adar300x_state *st, enum adar300x_ADC_sel sel)
{
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL),
				 ADAR300x_REG_ADC_CONTROL_RESET, ADAR300x_REG_ADC_CONTROL_RESET);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL),
				 ADAR300x_REG_ADC_CONTROL_RESET, 0);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADC_CONTROL),
				  ADAR300x_REG_ADC_CONTROL_MUX_SEL_MSK,
				  ADAR300x_REG_ADC_CONTROL_CLK_EN |
				  ADAR300x_REG_ADC_CONTROL_EN | sel);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int adar300x_set_page(struct adar300x_state *st, enum adar300x_pages page)
{
	return regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_ADDRESS_PAGE),
				  ADAR300x_ADDRESS_PAGE_MASK, page);
}

static int adar300x_ram_read(struct adar300x_state *st,
			     enum adar300x_pages page, u16 addr, char *data)
{
	u32 val;
	int ret;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, page);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, addr), &val);
	if (ret < 0)
		goto err_unlock;

	/* Second read necessary for valid data */
	ret = regmap_read(st->regmap, ADAR300x_REG(st, addr), &val);
	*data = val;

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static u8 adar300x_get_bem(struct adar300x_state *st, struct iio_chan_spec const *chan)
{
	return chan->address / st->chip_info->unpacked_beamst_len;
}

static void adar300x_unpack_data(char *packed, char *unpacked, int unpacked_len)
{
	int i, j;

	for (i = 0, j = 0; i < unpacked_len;) {
		unpacked[i + 0] = (packed[j + 0] >> 2) & ADAR300x_MAX_RAW;
		unpacked[i + 1] = ((packed[j + 0] << 4) | (packed[j + 1] >> 4)) & ADAR300x_MAX_RAW;
		unpacked[i + 2] = ((packed[j + 1] << 2) | (packed[j + 2] >> 6)) & ADAR300x_MAX_RAW;
		unpacked[i + 3] = (packed[j + 2]) & ADAR300x_MAX_RAW;

		unpacked[i + 4] = (packed[j + 3] >> 2) & ADAR300x_MAX_RAW;
		unpacked[i + 5] = ((packed[j + 3] << 4) | (packed[j + 4] >> 4)) & ADAR300x_MAX_RAW;
		unpacked[i + 6] = ((packed[j + 4] << 2) | (packed[j + 5] >> 6)) & ADAR300x_MAX_RAW;
		unpacked[i + 7] = (packed[j + 5]) & ADAR300x_MAX_RAW;

		i = i + ADAR300x_UNPACKED_BEAMSTATE_LEN;
		j = j + ADAR300x_PACKED_BEAMSTATE_LEN;
	}
}

static void adar300x_pack_data(char *packed, const char *unpacked, int unpacked_len)
{
	int i, j;

	for (i = 0, j = 0; i < unpacked_len;) {
		packed[j + 0] = unpacked[i + 0] << 2 | unpacked[i + 1] >> 4;
		packed[j + 1] = unpacked[i + 1] << 4 | unpacked[i + 2] >> 2;
		packed[j + 2] = unpacked[i + 2] << 6 | unpacked[i + 3];
		packed[j + 3] = unpacked[i + 4] << 2 | unpacked[i + 5] >> 4;
		packed[j + 4] = unpacked[i + 5] << 4 | unpacked[i + 6] >> 2;
		packed[j + 5] = unpacked[i + 6] << 6 | unpacked[i + 7];

		i = i + ADAR300x_UNPACKED_BEAMSTATE_LEN;
		j = j + ADAR300x_PACKED_BEAMSTATE_LEN;
	}
}

static int adar300x_get_mode_address(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    u16 *address)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u8 chan_addr, beam;

	beam = adar300x_get_bem(st, chan);
	if (st->chip_info->chip_id == ID_ADAR3003)
		chan_addr = chan->address + (4 * (chan->address / 4));
	else
		chan_addr = chan->address;


	switch (st->beam_load_mode[beam]) {
	case ADAR300x_DIRECT_CTRL:
	case ADAR300x_INST_DIRECT_CTRL:
		*address = ADAR3002_REG_DRCT_CNTRL(chan_addr);
		return 0;
	case ADAR300x_RESET:
		*address = ADAR3002_REG_RESET(chan_addr);
		return 0;
	case ADAR300x_MUTE:
		*address = ADAR3002_REG_MUTE(chan_addr);
		return 0;
	default:
		return -EINVAL;
	}
}

static int adar300x_get_mode_value(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan, u32 *val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u16 address;
	int ret;

	mutex_lock(&st->lock);
	ret = adar300x_get_mode_address(indio_dev, chan, &address);
	if (ret < 0)
		goto err_unlock;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, address), val);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int adar300x_set_mode_value(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan, u32 val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u16 address;
	int ret;

	mutex_lock(&st->lock);
	ret = adar300x_get_mode_address(indio_dev, chan, &address);
	if (ret < 0)
		goto err_unlock;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_write(st->regmap, ADAR300x_REG(st, address), val);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int adar300x_set_fifo_value(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan, u32 val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	u16 addr, element, beam;
	int ret, i;

	beam = adar300x_get_bem(st, chan);
	element = chan->address % st->chip_info->unpacked_beamst_len;
	st->state_buf[beam][element] = val;

	if (element != (ADAR300x_UNPACKED_BEAMSTATE_LEN - 1))
		return 0;

	mutex_lock(&st->lock);
	adar300x_pack_data(packed, st->state_buf[beam], ADAR300x_UNPACKED_BEAMSTATE_LEN);
	addr = ADAR300x_FIFO_LOAD(beam);
	ret = adar300x_set_page(st, ADAR300x_FIFO_PAGE);
	if (ret < 0)
		goto err_unlock;

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = regmap_write(st->regmap, ADAR300x_REG(st, addr + i), packed[i]);
		if (ret < 0)
			goto err_unlock;
	}

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int adar300x_get_fifo_value(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan, u32 *val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u8 beam, element;

	beam = adar300x_get_bem(st, chan);
	element = chan->address % st->chip_info->unpacked_beamst_len;
	*val = st->state_buf[beam][element];

	return 0;
}

static int adar300x_set_mem_value(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, u32 val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	char unpacked[ADAR300x_UNPACKED_BEAMSTATE_LEN];
	u16 beamst_addr;
	u8 beamst_no, beam;
	int ret, i;

	mutex_lock(&st->lock);
	beam = adar300x_get_bem(st, chan);
	beamst_no = st->beam_index[beam];
	beamst_addr = ADAR300x_RAM_BEAM_STATE_ADDR(beamst_no);

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = adar300x_ram_read(st, ADAR300x_BEAM0H_PAGE + beam,
					beamst_addr + i, &packed[i]);
		if (ret < 0)
			goto err_unlock;
	}

	adar300x_unpack_data(packed, unpacked, ADAR300x_UNPACKED_BEAMSTATE_LEN);

	unpacked[chan->address] = val;

	adar300x_pack_data(packed, unpacked, ADAR300x_UNPACKED_BEAMSTATE_LEN);

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = regmap_write(st->regmap, ADAR300x_REG(st, beamst_addr + i), packed[i]);
		if (ret < 0)
			goto err_unlock;
	}

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int adar300x_get_mem_value(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan, u32 *val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	char unpacked[ADAR300x_UNPACKED_BEAMSTATE_LEN];
	u16 addr;
	u8 beamst_no, beam;
	int ret, i;

	mutex_lock(&st->lock);
	beam = adar300x_get_bem(st, chan);
	beamst_no = st->beam_index[beam];
	addr = ADAR300x_RAM_BEAM_STATE_ADDR(beamst_no);

	for (i = 0; i < ADAR300x_PACKED_BEAMSTATE_LEN; i++) {
		ret = adar300x_ram_read(st, ADAR300x_BEAM0H_PAGE + beam, addr + i, &packed[i]);
		if (ret < 0)
			goto err_unlock;
	}

	adar300x_unpack_data(packed, unpacked, ADAR300x_UNPACKED_BEAMSTATE_LEN);

	*val = unpacked[chan->address];

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int adar300x_read_beamstate_elem(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan, int *val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u16 beam;
	int ret;

	beam = adar300x_get_bem(st, chan);
	if (st->beam_load_mode[beam] == ADAR300x_MEMORY_CTRL)
		ret = adar300x_get_mem_value(indio_dev, chan, val);
	else if (st->beam_load_mode[beam] == ADAR300x_FIFO_CTRL)
		ret = adar300x_get_fifo_value(indio_dev, chan, val);
	else
		ret = adar300x_get_mode_value(indio_dev, chan, val);

	return ret;
}

static int adar300x_write_beamstate_elem(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan, int val)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u16 beam;
	int ret;

	if (val > ADAR300x_MAX_RAW)
		return -EINVAL;

	beam = adar300x_get_bem(st, chan);
	if (st->beam_load_mode[beam] == ADAR300x_MEMORY_CTRL)
		ret = adar300x_set_mem_value(indio_dev, chan, val);
	else if (st->beam_load_mode[beam] == ADAR300x_FIFO_CTRL)
		ret = adar300x_set_fifo_value(indio_dev, chan, val);
	else
		ret = adar300x_set_mode_value(indio_dev, chan, val);

	return ret;
}

static int adar300x_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_POWER:
		case IIO_PHASE:
			ret = adar300x_read_beamstate_elem(indio_dev, chan, val);
			if (ret)
				return ret;

			return IIO_VAL_INT;
		case IIO_TEMP:
			ret = adar300x_adc_read(st, val);
			if (ret)
				return ret;

			return IIO_VAL_INT;
		break;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_POWER:
			*val = ADAR300x_MAX_GAIN_dB;
			*val2 = ADAR300x_MAX_RAW;

			return IIO_VAL_FRACTIONAL;
		case IIO_PHASE:
			*val = ADAR300x_MAX_PHASE_DEGREE;
			*val2 = ADAR300x_MAX_RAW;

			return IIO_VAL_FRACTIONAL;
		case IIO_TEMP:
			*val = 0;
			*val2 = 911500;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val = 86;
			*val2 = 362000;

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
};

static int adar300x_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_POWER:
		case IIO_PHASE:
			return adar300x_write_beamstate_elem(indio_dev, chan, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	};
}

ssize_t adar300x_amp_en_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u8 readval;

	mutex_lock(&st->lock);
	ret = kstrtou8(buf, 10, &readval);
	if (ret < 0)
		goto err_unlock;

	ch = (u32)this_attr->address;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_AMP_BIAS(ch)),
				 BIT(3), (readval << 3));

err_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

ssize_t adar300x_amp_en_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u32 readval;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ch = (u32)this_attr->address;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_AMP_BIAS(ch)), &readval);
	if (ret < 0)
		goto err_unlock;

	ret = sprintf(buf, "%d\n", ((readval >> 3) & 0x01));

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

ssize_t adar300x_amp_bias_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u8 readval;

	mutex_lock(&st->lock);
	ret = kstrtou8(buf, 10, &readval);
	if (ret < 0)
		goto err_unlock;

	ch = (u32)this_attr->address;

	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_AMP_BIAS(ch)),
				 0x07, (readval & 0x07));

err_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

ssize_t adar300x_amp_bias_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, ch;
	u32 readval;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ch = (u32)this_attr->address;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_AMP_BIAS(ch)), &readval);
	if (ret < 0)
		goto err_unlock;

	ret = sprintf(buf, "%d\n", (readval & 0x07));

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

ssize_t adar300x_update_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, beam;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	beam = (u32)this_attr->address;
	if (beam >= ADAR300x_BEAMS_PER_DEVICE) {
		ret = -EINVAL;
		goto err_unlock;
	}

	ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_BEAMWISE_UPDATE), BIT(beam));

err_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

ssize_t adar300x_update_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, beam;
	u32 readval;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	beam = (u32)this_attr->address;
	if (beam >= ADAR300x_BEAMS_PER_DEVICE) {
		ret = -EINVAL;
		goto err_unlock;
	}

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_BEAMWISE_UPDATE), &readval);
	if (ret < 0)
		goto err_unlock;

	ret = sprintf(buf, "%d\n", readval);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

ssize_t adar300x_ram_index_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u8 readval;
	int ret = 0, beam;

	beam = this_attr->address;
	ret = kstrtou8(buf, 10, &readval);
	if (readval > (ADAR300x_MAX_RAM_STATES - 1))
		return -EINVAL;

	st->beam_index[beam] = readval;

	return len;
}

ssize_t adar300x_ram_index_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 readval;

	readval = st->beam_index[this_attr->address];

	return sprintf(buf, "%d\n", readval);
}

ssize_t adar300x_ram_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u8 readval;
	int ret = 0, beam;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	beam = this_attr->address;
	ret = kstrtou8(buf, 10, &readval);
	if (ret < 0)
		goto err_unlock;

	if (readval > (ADAR300x_MAX_RAM_STATES - 1)) {
		ret = -EINVAL;
		goto err_unlock;
	}

	ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_MEM_SEQPTR(beam)), readval);

err_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

ssize_t adar300x_ram_range_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	int ret = 0;
	u16 beam;
	u32 readval;

	mutex_lock(&st->lock);
	beam = this_attr->address;
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_read(st->regmap,
			  ADAR300x_REG(st, ADAR300x_REG_MEM_SEQPTR0_START + beam),
			  &readval);
	if (ret < 0)
		goto err_unlock;

	ret = sprintf(buf, "%d\n", readval);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

ssize_t adar300x_fifo_ptr_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	int ret = 0;
	u16 fifo_attr, fifo_ptr;
	u32 readval;

	mutex_lock(&st->lock);
	fifo_attr = this_attr->address;
	fifo_ptr = ADAR300x_REG_FIFO_POINTER(fifo_attr);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, fifo_ptr), &readval);
	if (ret < 0)
		goto err_unlock;

	ret = sprintf(buf, "%d\n", readval);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

ssize_t adar300x_show_update_intf_ctrl_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(adar300x_update_intf_ctrl); ++i) {
		if (adar300x_update_intf_ctrl[i])
			len += sprintf(buf + len, "%s ", adar300x_update_intf_ctrl[i]);
	}

	return len;
}

ssize_t adar300x_update_intf_ctrl_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	unsigned int mode = 0, i;
	int ret;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	for (i = 0; i < ARRAY_SIZE(adar300x_update_intf_ctrl); ++i) {
		if (adar300x_update_intf_ctrl[i] && sysfs_streq(buf, adar300x_update_intf_ctrl[i])) {
			mode = i;
			break;
		}
	}

	ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_PIN_OR_SPI_CTL), mode);

err_unlock:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

ssize_t adar300x_update_intf_ctrl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 readval;
	int ret;

	mutex_lock(&st->lock);
	ret = adar300x_set_page(st, ADAR300x_CONFIG_PAGE);
	if (ret < 0)
		goto err_unlock;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_PIN_OR_SPI_CTL), &readval);
	if (ret < 0)
		goto err_unlock;

	ret = sprintf(buf, "%s\n", adar300x_update_intf_ctrl[readval]);

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

ssize_t adar300x_load_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	unsigned int mode = 0, i;

	for (i = 0; i < ARRAY_SIZE(adar300x_mode_ctrl); ++i) {
		if (adar300x_mode_ctrl[i] && sysfs_streq(buf, adar300x_mode_ctrl[i])) {
			mode = i;
			break;
		}
	}

	st->beam_load_mode[(u32)this_attr->address] = mode;

	return len;
}

ssize_t adar300x_load_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 readval;

	readval = st->beam_load_mode[(u32)this_attr->address];

	return sprintf(buf, "%s\n", adar300x_mode_ctrl[readval]);
}

ssize_t adar300x_show_mode_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(adar300x_mode_ctrl); ++i) {
		if (adar300x_mode_ctrl[i])
			len += sprintf(buf + len, "%s ", adar300x_mode_ctrl[i]);
	}

	return len;
}

ssize_t adar300x_mode_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	unsigned int mode = 0, i;
	int ret = 0, ch, beam, beam_mask;

	for (i = 0; i < ARRAY_SIZE(adar300x_mode_ctrl); ++i) {
		if (adar300x_mode_ctrl[i] && sysfs_streq(buf, adar300x_mode_ctrl[i])) {
			mode = i;
			break;
		}
	}

	ch = this_attr->address;
	beam_mask = ADAR300x_MODE0 << (ch * 2);
	if (mode <= ADAR300x_INST_DIRECT_CTRL) {
		beam = 0;
		ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_BEAMSTATE_MODE),
					 beam_mask, mode << (2 * ch));
		if (ret < 0)
			return ret;
	} else {
		/* In Instantaneous Direct Control, the ADAR3002 cannot go into Reset or Mute */
		if (st->beam_mode[ch] == ADAR300x_INST_DIRECT_CTRL)
			return -EINVAL;

		beam = BIT(ch * 2) << (mode == ADAR300x_MUTE);
	}

	ret = regmap_update_bits(st->regmap, ADAR300x_REG(st, ADAR300x_REG_BEAMWISE_UPDATE_CODE),
				 beam_mask, beam);
	if (ret < 0)
		return ret;

	st->beam_mode[ch] = mode;

	return len;
}

ssize_t adar300x_mode_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar300x_state *st = iio_priv(indio_dev);
	u16 ch;

	ch = this_attr->address;

	return sprintf(buf, "%s\n", adar300x_mode_ctrl[st->beam_mode[ch]]);
}

static bool adar300x_is_beam_active(u8 beam, u32 mask)
{
	/* Only one channel needs to be tested since we have: adar300x_available_scan_masks */
	return (mask & (1 << (ADAR300x_CHANNELS_PER_BEAM * beam)));
}

static int adar300x_ram_write(struct adar300x_state *st,
			      struct iio_dma_buffer_block *block,
			      u32 mask)
{
	char *data = block->vaddr;
	char packed[ADAR300x_PACKED_BEAMSTATE_LEN];
	int ret;
	int i, j, beam, ram_beam_state;
	u16 addr, unp_bst_len, p_bst_len;

	mutex_lock(&st->lock);
	unp_bst_len = st->chip_info->unpacked_beamst_len;
	p_bst_len = st->chip_info->packed_beamst_len;

	for (i = 0; i < block->block.bytes_used; i++)
		data[i] &= ADAR300x_MAX_RAW;

	for (i = 0, beam = 0, ram_beam_state = 0;
	     ((i + unp_bst_len) <= block->block.bytes_used) &&
	     ram_beam_state < ADAR300x_MAX_RAM_STATES;) {

		if (beam == ADAR300x_BEAMS_PER_DEVICE)
			ram_beam_state++;

		beam %= ADAR300x_BEAMS_PER_DEVICE;
		if (!adar300x_is_beam_active(beam, mask)) {
			beam++;
			continue;
		}
		adar300x_pack_data(packed, &data[i], ADAR300x_UNPACKED_BEAMSTATE_LEN);

		if (st->beam_load_mode[beam] == ADAR300x_MEMORY_CTRL) {
			ret = adar300x_set_page(st, beam + 1);
			if (ret < 0)
				goto err_unlock;

			addr = ADAR300x_RAM_BEAM_STATE_ADDR(ram_beam_state);
			addr += st->beam_index[beam];
			// ret = adar300x_bulk_write(st, addr, (data + i), ADAR300x_BEAMSTATE_LEN);
			if ((addr + ADAR300x_PACKED_BEAMSTATE_LEN - 1) > ADAR300x_RAM_MAX_ADDR)
				return -EINVAL;

		} else if (st->beam_load_mode[beam] == ADAR300x_FIFO_CTRL) {
			ret = adar300x_set_page(st, ADAR300x_FIFO_PAGE);
			if (ret < 0)
				goto err_unlock;

			addr = ADAR300x_FIFO_LOAD(beam);
		} else
			return -EINVAL;

		for (j = 0; j < p_bst_len; j++) {
			ret = regmap_write(st->regmap, ADAR300x_REG(st, addr + j), packed[j]);
			if (ret < 0)
				goto err_unlock;
		}

		i += unp_bst_len;
		beam++;
	}

err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

/* Will be called only when blocksize if full */
static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct adar300x_state *st = queue->driver_data;
	int ret;
	u32 ch_mask = *queue->buffer.scan_mask;

	block->block.bytes_used = block->block.size;
	ret = adar300x_ram_write(st, block, ch_mask);
	if (ret < 0)
		return ret;

	iio_dma_buffer_block_done(block);

	return 0;
}

static void hw_abort(struct iio_dma_buffer_queue *queue)
{
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = hw_abort,
};

static void iio_dmaengine_buffer_release(struct iio_buffer *buf)
{
}

static const struct iio_buffer_access_funcs iio_dmaengine_buffer_ops = {
	.read = iio_dma_buffer_read,
	.write = iio_dma_buffer_write,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.set_length = iio_dma_buffer_set_length,
	.enable = iio_dma_buffer_enable,
	.disable = iio_dma_buffer_disable,
	.data_available = iio_dma_buffer_data_available,
	.space_available = iio_dma_buffer_space_available,
	.release = iio_dmaengine_buffer_release,

	.modes = INDIO_BUFFER_HARDWARE,
	.flags = INDIO_BUFFER_FLAG_FIXED_WATERMARK,
};

static int adar300x_setup_buffer(struct device *dev, struct iio_dev *indio_dev,
				int irq)
{
	struct adar300x_state *st;
	int ret;

	st = iio_priv(indio_dev);
	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	ret = iio_dma_buffer_init(&st->queue, dev, &dma_buffer_ops, st);
	if (ret)
		return ret;

	st->queue.buffer.access = &iio_dmaengine_buffer_ops;
	st->dma_buffer = iio_buffer_get(&st->queue.buffer);
	indio_dev->buffer = st->dma_buffer;

	return 0;
}

static const unsigned long adar300x_available_scan_masks[] = {
	0x000000FF, 0x0000FF00, 0x0000FFFF, 0x00FF0000,
	0x00FF00FF, 0x00FFFF00, 0x00FFFFFF, 0xFF000000,
	0xFF0000FF, 0xFF00FF00, 0xFF00FFFF, 0xFFFF0000,
	0xFFFF00FF, 0xFFFFFF00, 0xFFFFFFFF, 0x00000000,
};

static struct iio_info adar300x_info = {
	.read_raw = &adar300x_read_raw,
	.write_raw = &adar300x_write_raw,
	.debugfs_reg_access = &adar300x_reg_access,
};

static int adar300x_setup(struct iio_dev *indio_dev)
{
	struct adar300x_state *st = iio_priv(indio_dev);
	u32 val = 0;
	int ret;

	/* Software reset and activate SDO */
	ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_SPI_CONFIG),
			   ADAR300x_SPI_CONFIG_RESET_ |
			   ADAR300x_SPI_CONFIG_SDOACTIVE_ |
			   ADAR300x_SPI_CONFIG_SDOACTIVE |
			   ADAR300x_SPI_CONFIG_RESET);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_SCRATCHPAD), 0xAD);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_SCRATCHPAD), &val);
	if (ret < 0)
		return ret;

	if (val != 0xAD) {
		dev_err(indio_dev->dev.parent, "Failed to read/write scratchpad");
		return -EIO;
	}

	ret = regmap_write(st->regmap, ADAR300x_REG(st, ADAR300x_REG_SCRATCHPAD), 0xEA);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_SCRATCHPAD), &val);
	if (ret < 0)
		return ret;

	if (val != 0xEA) {
		dev_err(indio_dev->dev.parent, "Failed to read/write scratchpad");
		return -EIO;
	}

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_CHIPTYPE), &val);
	if (ret < 0)
		return ret;

	if (val != ADAR300x_CHIP_TYPE) {
		dev_err(indio_dev->dev.parent, "Failed to read ADAR300x_REG_CHIPTYPE %x", val);
		return -EIO;
	}

	ret = regmap_read(st->regmap, ADAR300x_REG(st, ADAR300x_REG_PRODUCT_ID_L), &val);
	if (ret < 0)
		return ret;

	if (val != st->chip_info->product_id) {
		dev_err(indio_dev->dev.parent, "Failed to read PRODUCT_ID_L %x", val);
		return -EIO;
	}

	return adar300x_adc_setup(st, ADAR300x_ADC_TEMPERATURE);
}

static int ad300x_reset(struct gpio_desc *gpio_reset)
{
	if (!IS_ERR(gpio_reset)) {
		gpiod_set_value_cansleep(gpio_reset, 0);
		ndelay(100);
		gpiod_set_value_cansleep(gpio_reset, 1);
		ndelay(100);
	}

	return 0;
}

int adar300x_probe(struct spi_device *spi, const struct attribute_group *attr_group)
{
	struct device_node		*child, *np = spi->dev.of_node;
	int				ret, cnt = 0, num_dev;
	struct iio_dev			*indio_dev;
	struct regmap			*regmap;
	struct adar300x_state		*st;
	const struct adar300x_chip_info *info;
	struct gpio_desc		*gpio_reset;
	u32				tmp;

	num_dev = of_get_available_child_count(np);
	if (num_dev < 1 || num_dev > ADAR300x_MAX_DEV) {
		dev_err(&spi->dev, "Number of devices is incorrect (%d)\n", num_dev);
		return -ENODEV;
	}

	regmap = devm_regmap_init_spi(spi, &adar300x_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	gpio_reset = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (!IS_ERR(gpio_reset))
		ad300x_reset(gpio_reset);

	for_each_available_child_of_node(np, child) {
		indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
		if (!indio_dev)
			return -ENOMEM;

		info = of_device_get_match_data(&spi->dev);
		if (!info)
			return -ENODEV;

		st = iio_priv(indio_dev);
		st->spi = spi;
		dev_set_drvdata(&spi->dev, st);
		st->chip_info = info;
		st->regmap = regmap;
		st->gpio_reset = gpio_reset;
		ret = of_property_read_u32(child, "reg", &tmp);
		if (ret < 0)
			return ret;

		st->dev_addr = ADAR300x_SPI_ADDR(tmp);
		indio_dev->dev.parent = &spi->dev;
		indio_dev->name = child->name;
		indio_dev->channels = st->chip_info->channels;
		indio_dev->num_channels = st->chip_info->num_channels;
		adar300x_info.attrs = attr_group;
		indio_dev->info = &adar300x_info;
		indio_dev->modes = INDIO_DIRECT_MODE;
		indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
		indio_dev->available_scan_masks = adar300x_available_scan_masks;
		ret = adar300x_setup(indio_dev);

		if (ret < 0) {
			dev_err(&spi->dev, "Setup failed (%d), dev: %d, cnt: %d\n", ret, tmp, cnt);
		} else {
			/* Do setup for each device */
			ret = adar300x_setup_buffer(&spi->dev, indio_dev, spi->irq);
			if (ret) {
				dev_err(&spi->dev, "Error buffer setup\n");
				return ret;
			}

			ret = devm_iio_device_register(&spi->dev, indio_dev);
			if (ret < 0)
				return ret;
		}
		cnt++;
	}

	return 0;
}
