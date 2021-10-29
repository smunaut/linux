// SPDX-License-Identifier: GPL-2.0
/*
 * ADAR3000, ADAR3001, ADAR3002, ADAR3003 device driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

/* ADMV4420 Register Map */
#define ADMV4420_SPI_CONFIG_1			0x00
#define ADMV4420_SPI_CONFIG_2			0x01
#define ADMV4420_CHIPTYPE			0x03
#define ADMV4420_PRODUCT_ID_L			0x04
#define ADMV4420_PRODUCT_ID_H			0x05
#define ADMV4420_SCRATCHPAD			0x0A
#define ADMV4420_SPI_REV			0x0B
#define ADMV4420_ENABLES			0x103
#define ADMV4420_SDO_LEVEL			0x108
#define ADMV4420_INT_L				0x200
#define ADMV4420_INT_H				0x201
#define ADMV4420_FRAC_L				0x202
#define ADMV4420_FRAC_M				0x203
#define ADMV4420_FRAC_H				0x204
#define ADMV4420_MOD_L				0x208
#define ADMV4420_MOD_M				0x209
#define ADMV4420_MOD_H				0x20A
#define ADMV4420_R_DIV_L			0x20C
#define ADMV4420_R_DIV_H			0x20D
#define ADMV4420_REFERENCE			0x20E
#define ADMV4420_VCO_DATA_READBACK1		0x211
#define ADMV4420_VCO_DATA_READBACK2		0x212
#define ADMV4420_PLL_MUX_SEL			0x213
#define ADMV4420_LOCK_DETECT			0x214
#define ADMV4420_BAND_SELECT			0x215
#define ADMV4420_VCO_ALC_TIMEOUT		0x216
#define ADMV4420_VCO_MANUAL			0x217
#define ADMV4420_ALC				0x219
#define ADMV4420_VCO_TIMEOUT1			0x21C
#define ADMV4420_VCO_TIMEOUT2			0x21D
#define ADMV4420_VCO_BAND_DIV			0x21E
#define ADMV4420_VCO_READBACK_SEL		0x21F
#define ADMV4420_AUTOCAL			0x226
#define ADMV4420_CP_STATE			0x22C
#define ADMV4420_CP_BLEED_EN			0x22D
#define ADMV4420_CP_CURRENT			0x22E
#define ADMV4420_CP_BLEED			0x22F

enum adar3000_ids {
	ID_ADAR3000,
	ID_ADAR3002,
	ID_ADAR3003,
};

/* Beam state composed of 48 bits ADAR3000/3001/3002
 * For ADAR3003 only 23 bits are used */
struct adar3000_beam_state {
	u8 delay0 : 6;
	u8 atten0 : 6;
	u8 delay1 : 6;
	u8 atten1 : 6;
	u8 delay2 : 6;
	u8 atten2 : 6;
	u8 delay3 : 6;
	u8 atten3 : 6;
};

#define ADAR300x_MAX_RAM_STATES 4
struct adar3000_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	u8				dev_addr;
	struct adar3000_beam_state	*beam_st;
	u8				beam_index[ADAR300x_MAX_RAM_STATES];
};

static const struct regmap_config adar3000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int adar3000_reg_read(struct adar3000_state *st, u32 reg, u32 *val)
{
	return regmap_read(st->regmap, st->dev_addr | reg, val);
}

static int adar3000_reg_write(struct adar3000_state *st, u32 reg, u32 val)
{
	return regmap_write(st->regmap, st->dev_addr | reg, val);
}

static int adar3000_reg_update(struct adar3000_state *st, u32 reg, u32 mask, u32 val)
{
	int ret;
	u32 readval;

	ret = adar3000_reg_read(st, reg, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	readval |= val;

	return adar3000_reg_write(st, reg, val);
}

static int adar3000_reg_access(struct iio_dev *indio_dev,
			       u32 reg, u32 writeval,
			       u32 *readval)
{
	struct adar3000_state *st = iio_priv(indio_dev);

	if (readval)
		return adar3000_reg_read(st, reg, readval);
	else
		return adar3000_reg_write(st, reg, writeval);
}

static int adar3000_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	// struct adar3000_state *st = iio_priv(indio_dev);
	// int ret, ch;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
};

static int adar3000_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	// struct adar3000_state *st = iio_priv(indio_dev);
	// u32 code;
	// int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
	case IIO_CHAN_INFO_PHASE:
	default:
		return -EINVAL;
	};
}

static int adar3000_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

#define ADAR3000_CHANNEL(_num, name)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE),			\
	.extend_name = name,					\
}


static ssize_t adar300x_update_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adar3000_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret, beam;
	u32 readval;

	beam = this_attr->address;
	readval = st->beam_index[beam];

	ret = sprintf(buf, "%d\n", readval);

	return ret;
}

static ssize_t adar300x_update_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct adar3000_state *st = iio_priv(indio_dev);
	u8 readval;
	int ret = 0, beam;

	beam = this_attr->address;
	ret = kstrtou8(buf, 10, &readval);
	if (readval > (ADAR300x_MAX_RAM_STATES - 1))
		return -EINVAL;

	st->beam_index[beam] = readval;

	return len;
}

static IIO_DEVICE_ATTR(beam0_update, 0644,
		       adar300x_update_show, adar300x_update_store, 0);

static struct attribute *adar3000_attributes[] = {
	&iio_dev_attr_beam0_update.dev_attr.attr,
	NULL,
};

static const struct attribute_group adar3000_attribute_group = {
	.attrs = adar3000_attributes,
};
/* Maybe ditch the entire channel to element mapping and just use the
 * beamstates as a reference attribute. Suggestion may include a new type of
 * raw value similar go hardwaregain, phase -> beamstate */
#define DECLARE_ADAR3000_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR3000_CHANNEL(0, "BEAM0_EL0"),			\
	ADAR3000_CHANNEL(1, "BEAM0_EL1"),			\
	ADAR3000_CHANNEL(2, "BEAM0_EL2"),			\
	ADAR3000_CHANNEL(3, "BEAM0_EL3"),			\
	ADAR3000_CHANNEL(4, "BEAM1_EL0"),			\
	ADAR3000_CHANNEL(5, "BEAM1_EL1"),			\
	ADAR3000_CHANNEL(6, "BEAM1_EL2"),			\
	ADAR3000_CHANNEL(7, "BEAM1_EL3"),			\
	ADAR3000_CHANNEL(8, "BEAM2_EL0"),			\
	ADAR3000_CHANNEL(9, "BEAM2_EL1"),			\
	ADAR3000_CHANNEL(10, "BEAM2_EL2"),			\
	ADAR3000_CHANNEL(11, "BEAM2_EL3"),			\
	ADAR3000_CHANNEL(12, "BEAM3_EL0"),			\
	ADAR3000_CHANNEL(13, "BEAM3_EL1"),			\
	ADAR3000_CHANNEL(14, "BEAM3_EL2"),			\
	ADAR3000_CHANNEL(15, "BEAM3_EL3"),			\
};

#define DECLARE_ADAR3002_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR3000_CHANNEL(0, "BEAM0_EL0H"),			\
	ADAR3000_CHANNEL(1, "BEAM0_EL1H"),			\
	ADAR3000_CHANNEL(2, "BEAM0_EL2H"),			\
	ADAR3000_CHANNEL(3, "BEAM0_EL3H"),			\
	ADAR3000_CHANNEL(4, "BEAM0_EL0V"),			\
	ADAR3000_CHANNEL(5, "BEAM0_EL1V"),			\
	ADAR3000_CHANNEL(6, "BEAM0_EL2V"),			\
	ADAR3000_CHANNEL(7, "BEAM0_EL3V"),			\
	ADAR3000_CHANNEL(8, "BEAM1_EL0V"),			\
	ADAR3000_CHANNEL(9, "BEAM1_EL1V"),			\
	ADAR3000_CHANNEL(10, "BEAM1_EL2V"),			\
	ADAR3000_CHANNEL(11, "BEAM1_EL3V"),			\
	ADAR3000_CHANNEL(12, "BEAM1_EL0H"),			\
	ADAR3000_CHANNEL(13, "BEAM1_EL1H"),			\
	ADAR3000_CHANNEL(14, "BEAM1_EL2H"),			\
	ADAR3000_CHANNEL(15, "BEAM1_EL3H"),			\
};

#define DECLARE_ADAR3003_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR3000_CHANNEL(0, "BEAM_EL0V"),			\
	ADAR3000_CHANNEL(1, "BEAM_EL0H"),			\
	ADAR3000_CHANNEL(2, "BEAM_EL1V"),			\
	ADAR3000_CHANNEL(3, "BEAM_EL1H"),			\
	ADAR3000_CHANNEL(4, "BEAM_EL2V"),			\
	ADAR3000_CHANNEL(5, "BEAM_EL2H"),			\
	ADAR3000_CHANNEL(6, "BEAM_EL3V"),			\
	ADAR3000_CHANNEL(7, "BEAM_EL3H"),			\
};

DECLARE_ADAR3000_CHANNELS(adar3000_channels);
DECLARE_ADAR3002_CHANNELS(adar3002_channels);
DECLARE_ADAR3003_CHANNELS(adar3003_channels);

enum adar3000_beamstate_mode_ctrl {
	ADAR3000_DIRECT_CTRL,
	ADAR3000_MEMORY_CTRL,
	ADAR3000_FIFO_CTRL,
	ADAR3000_INST_DIRECT_CTRL
};

static const char *const adar3000_mode_ctrl[] = {
	[ADAR3000_DIRECT_CTRL] = "direct",
	[ADAR3000_MEMORY_CTRL] = "memory",
	[ADAR3000_FIFO_CTRL] = "fifo",
	[ADAR3000_INST_DIRECT_CTRL] = "instant_direct"
};

enum adar1000_iio_dev_attr {
	ADAR3000_MODE_CTRL,
};

static ssize_t adar3000_beam_mode_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	struct adar3000_state *st = iio_device_get_drvdata(indio_dev);
	u32 readval;
	int ret;

	ret = adar3000_reg_read(st, 0x0a, &readval);
	if (ret <0)
		return ret;

	return sprintf(buf, "%d\n", readval);
}

static ssize_t adar3000_beam_mode_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct adar3000_state *st = iio_priv(indio_dev);
	u8 readin;
	int ret;
	ret = kstrtou8(buf, 10, &readin);
	if (ret)
		return ret;

	ret = adar3000_reg_write(st, 0x0a, readin); 

	return ret ? ret : len;
}

// static struct iio_chan_spec_ext_info adar3000_ext_info[] = {
// 	{
// 	 .name = "mode_ctrl",
// 	 .read = adar3000_beam_mode_read,
// 	 .write = adar3000_beam_mode_write,
// 	 .shared = IIO_SHARED_BY_TYPE,
// 	 },
// 	{
// 	 .name = "mode_ctrl_available",
// 	 .read = adar3000_beam_mode_available,
// 	 .shared = IIO_SHARED_BY_TYPE,
// 	 },
// 	{},
// };

static const struct iio_info adar3000_info = {
	.read_raw = &adar3000_read_raw,
	.write_raw = &adar3000_write_raw,
	.write_raw_get_fmt = &adar3000_write_raw_get_fmt,
	.debugfs_reg_access = &adar3000_reg_access,
	.attrs = &adar3000_attribute_group,
};

static int adar3000_setup(struct iio_dev *indio_dev)
{
	struct adar3000_state *st = iio_priv(indio_dev);
	u32 val = 0;
	int ret;
	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	/* Software reset and activate SDO */
	
	ret = regmap_write(st->regmap, 0x0A, 0xAD);
	if (ret < 0)
		return ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = regmap_read(st->regmap, 0x0A, &val);
	if (ret < 0)
		return ret;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

	if (val != 0xAD) {
		dev_err(indio_dev->dev.parent, "Failed ADMV4420 to read/write scratchpad %x ", val);
		return -EIO;
	}

	ret = regmap_write(st->regmap, 0x0A, 0x5A);
	if (ret < 0)
		return ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = regmap_read(st->regmap, 0x0A, &val);
	if (ret < 0)
		return ret;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

	if (val != 0x5A) {
		dev_err(indio_dev->dev.parent, "Failed ADMV4420 to read/write scratchpad %x ", val);
		return -EIO;
	}
	dev_err(indio_dev->dev.parent, "ADMV4420 to read/write scratchpad %x ", val);
	
	return 0;
}

static int adar3000_probe(struct spi_device *spi)
{

	struct iio_dev *indio_dev;
	struct adar3000_state *st;
	struct regmap *regmap;
	int ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);	
	regmap = devm_regmap_init_spi(spi, &adar3000_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error  ADMV4420 initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	
	st = iio_priv(indio_dev);
	st->spi = spi;
	st->regmap = regmap;
	
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "admv4420";
	indio_dev->info = &adar3000_info;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = adar3000_setup(indio_dev);

	if (ret < 0) {
		dev_err(&spi->dev, "Setup ADMV4420 failed (%d)\n", ret);
		return ret;
	}
	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id adar3000_of_match[] = {
	{ .compatible = "adi,admv4420" },
	{ },
};
MODULE_DEVICE_TABLE(of, adar3000_of_match);

static struct spi_driver adar3000_driver = {
	.driver = {
		.name	= "admv4420",
		.of_match_table = adar3000_of_match,
	},
	.probe		= adar3000_probe,
};
module_spi_driver(adar3000_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADMV44200 K Band Downconverter");
MODULE_LICENSE("GPL v2");
