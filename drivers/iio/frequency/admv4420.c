// SPDX-License-Identifier: GPL-2.0
/*
 * ADMV4420
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

#define ADAR300x_MAX_RAM_STATES 4

#define ADMV4420_REFERENCE_IN_MODE(x)		(x << 1)
#define ADMV4420_REFERENCE_DOUBLER(x)		(x << 2)

#define ENABLE_PLL				BIT(6)
#define ENABLE_LO				BIT(5)
#define ENABLE_VCO				BIT(3)
#define ENABLE_IFAMP				BIT(2)
#define ENABLE_MIXER				BIT(1)
#define ENABLE_LNA				BIT(0)

enum admv4420_option_st {
	ADMV4420_DISABLED,
	ADMV4420_ENABLED,
};

enum admv4420_ref_op {
	ADMV4420_DOUBLER,
	ADMV4420_DIVIDE_BY_2,
};

enum admv4420_mux_sel {
	ADMV4420_LOW = 0,
	ADMV4420_LOCK_DTCT = 1,
	ADMV4420_R_COUNTER_PER_2 = 4,
	ADMV4420_N_CONUTER_PER_2 = 5,
	ADMV4420_HIGH = 8,
};

enum admv4420_ref_type {
	ADMV4420_XTAL,
	ADMV4420_SINGLE_ENDED,
};

enum admv4420_charge_pump_st {
	ADMV4420_CHARGE_PUMP_HIGHZ,
	ADMV4420_CHARGE_PUMP_FORCE_UP,
	ADMV4420_CHARGE_PUMP_FORCE_DOWN,
	ADMV4420_CHARGE_PUMP_NORMAL_OP,
};

enum admv4420_n_counter_par {
	ADMV4420_N_COUNTER_INT,
	ADMV4420_N_COUNTER_FRAC,
	ADMV4420_N_COUNTER_MOD,
};

struct admv4420_reference_block {
	bool doubler_en;
	bool divede_by_2_en;
	enum admv4420_ref_type type;
	u32 freq_hz;
	u32 diveder;
};

struct admv4420_n_counter {
	u32 int_val;
	u32 frac_val;
	u32 mod_val;
	u32 n_counter;
};

struct admv4420_charge_pump {
	bool bleed_en;
	enum admv4420_charge_pump_st state;
	u32 current_uA;
	u32 bleed_current_uA;
};

struct admv4420_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	unsigned long int 		pfd_freq_hz;
	unsigned long int 		vco_freq_hz;
	unsigned long int 		lo_freq_hz;
	struct admv4420_reference_block ref_block;
	struct admv4420_n_counter	n_counter;
	struct admv4420_charge_pump	pump;
	enum admv4420_mux_sel		mux_sel;
	u8				beam_index[ADAR300x_MAX_RAM_STATES];
};

static const char *const admv4420_op_state[] = {
	[ADMV4420_DISABLED] = "disabled",
	[ADMV4420_ENABLED] = "enabled",
};

static const char *const admv4420_ref_type_str[] = {
	[ADMV4420_XTAL] = "XTAL",
	[ADMV4420_SINGLE_ENDED] = "single_ended",
};

static const char *const admv4420_mux_sel_str[] = {
	[ADMV4420_LOW] = "LOW",
	[ADMV4420_LOCK_DTCT] = "LOCK_DTCT",
	[ADMV4420_R_COUNTER_PER_2] = "R_COUNTER_PER_2",
	[ADMV4420_N_CONUTER_PER_2] = "N_CONUTER_PER_2",
	[ADMV4420_HIGH] = "HIGH",
};

static const struct regmap_config admv4420_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int admv4420_reg_access(struct iio_dev *indio_dev,
			       u32 reg, u32 writeval,
			       u32 *readval)
{
	struct admv4420_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

#define ADMV4420_CHANNEL(_num, name)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE),			\
	.extend_name = name,					\
}

static ssize_t admv4420_mux_sel_show_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(admv4420_mux_sel_str); ++i) {
		if (admv4420_mux_sel_str[i])
			len += sprintf(buf + len, "%s ", admv4420_mux_sel_str[i]);
	}

	return len;
}

static ssize_t admv4420_mux_sel_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	
	return sprintf(buf, "%s\n", admv4420_mux_sel_str[st->mux_sel]);
}

static ssize_t admv4420_mux_sel_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	int i, mode;

	for (i = 0; i < ARRAY_SIZE(admv4420_mux_sel_str); ++i) {
		if (admv4420_mux_sel_str[i] && sysfs_streq(buf, admv4420_mux_sel_str[i])) {
			mode = i;
			break;
		}
	}
	st->mux_sel = mode;

	return len;
}

static ssize_t admv4420_ref_type_show_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(admv4420_ref_type_str); ++i) {
		if (admv4420_ref_type_str[i])
			len += sprintf(buf + len, "%s ", admv4420_ref_type_str[i]);
	}

	return len;
}

static ssize_t admv4420_ref_type_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	
	return sprintf(buf, "%s\n", admv4420_ref_type_str[st->ref_block.type]);
}

static ssize_t admv4420_ref_type_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	int i, mode;

	for (i = 0; i < ARRAY_SIZE(admv4420_ref_type_str); ++i) {
		if (admv4420_ref_type_str[i] && sysfs_streq(buf, admv4420_ref_type_str[i])) {
			mode = i;
			break;
		}
	}
	st->ref_block.type = mode;

	return len;
}

static ssize_t admv4420_ref_doubler_show_available(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(admv4420_op_state); ++i) {
		if (admv4420_op_state[i])
			len += sprintf(buf + len, "%s ", admv4420_op_state[i]);
	}

	return len;
}

static ssize_t admv4420_ref_doubler_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv4420_state *st = iio_priv(indio_dev);
	switch(this_attr->address) {
	case ADMV4420_DOUBLER:
		return sprintf(buf, "%s\n", admv4420_op_state[st->ref_block.doubler_en]);
	case ADMV4420_DIVIDE_BY_2:
		return sprintf(buf, "%s\n", admv4420_op_state[st->ref_block.divede_by_2_en]);
	default:
		return -EINVAL;
	}
}

static ssize_t admv4420_ref_doubler_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct admv4420_state *st = iio_priv(indio_dev);
	int i, mode;

	for (i = 0; i < ARRAY_SIZE(admv4420_op_state); ++i) {
		if (admv4420_op_state[i] && sysfs_streq(buf, admv4420_op_state[i])) {
			mode = i;
			break;
		}
	}
	switch(this_attr->address) {
	case ADMV4420_DOUBLER:
		st->ref_block.doubler_en = mode;
		break;
	case ADMV4420_DIVIDE_BY_2:
		st->ref_block.divede_by_2_en = mode;
		break;
	default:
		return -EINVAL;
	}

	return len;
}

static ssize_t admv4420_n_counter_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);

	switch(this_attr->address) {
	case ADMV4420_N_COUNTER_INT:
		return sprintf(buf, "%d\n", st->n_counter.int_val);
	case ADMV4420_N_COUNTER_FRAC:
		return sprintf(buf, "%d\n", st->n_counter.frac_val);
	case ADMV4420_N_COUNTER_MOD:
		return sprintf(buf, "%d\n", st->n_counter.mod_val);
	default:
		return -EINVAL;
	}
}

static ssize_t admv4420_n_counter_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	u32 readval;
	int ret = 0;

	ret = kstrtou32(buf, 10, &readval);
	if (ret)
		return ret;

	switch(this_attr->address) {
	case ADMV4420_N_COUNTER_INT:
		st->n_counter.int_val = readval;
		break;
	case ADMV4420_N_COUNTER_FRAC:
		st->n_counter.frac_val = readval;
		break;
	case ADMV4420_N_COUNTER_MOD:
		st->n_counter.mod_val = readval;
		break;
	default:
		return -EINVAL;
	}

	return len;
}
	// of_property_read_u32(spi->dev.of_node, "adi,ref_diveder", &st->ref_block.diveder);

static ssize_t admv4420_ref_divider_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->ref_block.diveder);
}

static ssize_t admv4420_ref_divider_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	u32 readval;
	int ret = 0;

	ret = kstrtou32(buf, 10, &readval);
	if (ret)
		return ret;

	st->ref_block.diveder = readval;

	return len;
}

static ssize_t admv4420_ref_freq_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	
	return sprintf(buf, "%d\n", st->ref_block.freq_hz);
}

static ssize_t admv4420_ref_freq_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);
	u32 readval;
	int ret = 0;

	ret = kstrtou32(buf, 10, &readval);
	if (ret)
		return ret;

	st->ref_block.freq_hz = readval;

	return len;
}

static ssize_t admv4420_vco_freq_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%ld\n", st->vco_freq_hz);
}

static ssize_t admv4420_lo_freq_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv4420_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%ld\n", st->lo_freq_hz);
}

static IIO_DEVICE_ATTR(ref_freq, 0644, admv4420_ref_freq_show, admv4420_ref_freq_store, 0);
static IIO_DEVICE_ATTR(vco_freq, 0444, admv4420_vco_freq_show, NULL, 0);
static IIO_DEVICE_ATTR(lo_freq, 0444, admv4420_lo_freq_show, NULL, 0);
static IIO_DEVICE_ATTR(ref_doubler, 0644, admv4420_ref_doubler_show, admv4420_ref_doubler_store, ADMV4420_DOUBLER);
static IIO_DEVICE_ATTR(ref_divide_by_2, 0644, admv4420_ref_doubler_show, admv4420_ref_doubler_store, ADMV4420_DIVIDE_BY_2);
static IIO_DEVICE_ATTR(ref_type, 0644, admv4420_ref_type_show, admv4420_ref_type_store, ADMV4420_DIVIDE_BY_2);
static IIO_DEVICE_ATTR(ref_divider, 0644, admv4420_ref_divider_show, admv4420_ref_divider_store, 0);
static IIO_DEVICE_ATTR(mux_sel, 0644, admv4420_mux_sel_show, admv4420_mux_sel_store, 0);
static IIO_DEVICE_ATTR(n_counter_int_val, 0644, admv4420_n_counter_show, admv4420_n_counter_store, ADMV4420_N_COUNTER_INT);
static IIO_DEVICE_ATTR(n_counter_frac_val, 0644, admv4420_n_counter_show, admv4420_n_counter_store, ADMV4420_N_COUNTER_FRAC);
static IIO_DEVICE_ATTR(n_counter_mod_val, 0644, admv4420_n_counter_show, admv4420_n_counter_store, ADMV4420_N_COUNTER_MOD);
static IIO_DEVICE_ATTR(ref_doubler_available, 0444, admv4420_ref_doubler_show_available, NULL, 0);
static IIO_DEVICE_ATTR(ref_divider_available, 0444, admv4420_ref_doubler_show_available, NULL, 0);
static IIO_DEVICE_ATTR(ref_type_available, 0444, admv4420_ref_type_show_available, NULL, 0);
static IIO_DEVICE_ATTR(mux_sel_available, 0444, admv4420_mux_sel_show_available, NULL, 0);

static struct attribute *admv4420_attributes[] = {
	&iio_dev_attr_ref_freq.dev_attr.attr,
	&iio_dev_attr_vco_freq.dev_attr.attr,
	&iio_dev_attr_lo_freq.dev_attr.attr,
	&iio_dev_attr_ref_doubler.dev_attr.attr,
	&iio_dev_attr_ref_divide_by_2.dev_attr.attr,
	&iio_dev_attr_ref_type.dev_attr.attr,
	&iio_dev_attr_mux_sel.dev_attr.attr,
	&iio_dev_attr_ref_divider.dev_attr.attr,
	&iio_dev_attr_n_counter_int_val.dev_attr.attr,
	&iio_dev_attr_n_counter_frac_val.dev_attr.attr,
	&iio_dev_attr_n_counter_mod_val.dev_attr.attr,
	&iio_dev_attr_ref_doubler_available.dev_attr.attr,
	&iio_dev_attr_ref_divider_available.dev_attr.attr,
	&iio_dev_attr_ref_type_available.dev_attr.attr,
	&iio_dev_attr_mux_sel_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group admv4420_attribute_group = {
	.attrs = admv4420_attributes,
};

static const struct iio_info admv4420_info = {
	.debugfs_reg_access = &admv4420_reg_access,
	.attrs = &admv4420_attribute_group,
};

static int admv4420_dt_parse(struct admv4420_state *st)
{
	struct spi_device *spi = st->spi;

	of_property_read_u32(spi->dev.of_node, "adi,ref_freq_hz", &st->ref_block.freq_hz);
	of_property_read_u32(spi->dev.of_node, "adi,ref_type", &st->ref_block.type);
	st->ref_block.doubler_en = of_property_read_bool(spi->dev.of_node, "adi,ref_doubler_en");
	st->ref_block.divede_by_2_en = of_property_read_bool(spi->dev.of_node,
							     "adi,ref_divede_by_2_en");
	of_property_read_u32(spi->dev.of_node, "adi,ref_diveder", &st->ref_block.diveder);
	
	of_property_read_u32(spi->dev.of_node, "adi,N_counter_int_val", &st->n_counter.int_val);
	of_property_read_u32(spi->dev.of_node, "adi,N_counter_frac_val", &st->n_counter.frac_val);
	of_property_read_u32(spi->dev.of_node, "adi,N_counter_mod_val", &st->n_counter.mod_val);

	of_property_read_u32(spi->dev.of_node, "adi,mux_sel", &st->mux_sel);

	return 0;
}

static int admv4420_setup(struct iio_dev *indio_dev)
{
	struct admv4420_state *st = iio_priv(indio_dev);
	u32 val = 0;
	int ret;
	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	/* Software reset and activate SDO */
	ret = regmap_write(st->regmap, ADMV4420_SPI_CONFIG_1, 0x81);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_SCRATCHPAD, 0xAD);
	if (ret < 0)
		return ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = regmap_read(st->regmap, ADMV4420_SCRATCHPAD, &val);
	if (ret < 0)
		return ret;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

	if (val != 0xAD) {
		dev_err(indio_dev->dev.parent, "Failed ADMV4420 to read/write scratchpad %x ", val);
		return -EIO;
	}

	ret = regmap_write(st->regmap, ADMV4420_SCRATCHPAD, 0x5A);
	if (ret < 0)
		return ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = regmap_read(st->regmap, ADMV4420_SCRATCHPAD, &val);
	if (ret < 0)
		return ret;

pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);

	if (val != 0x5A) {
		dev_err(indio_dev->dev.parent, "Failed ADMV4420 to read/write scratchpad %x ", val);
		return -EIO;
	}
	dev_err(indio_dev->dev.parent, "ADMV4420 to read/write scratchpad %x ", val);

	st->ref_block.freq_hz = 50000000;
	st->ref_block.type = ADMV4420_XTAL;
	st->ref_block.doubler_en = false;
	st->ref_block.divede_by_2_en = false;
	st->ref_block.diveder = 1;

	st->n_counter.int_val = 0xA7;
	st->n_counter.frac_val = 0x02;
	st->n_counter.mod_val = 0x04;

	st->pump.state = ADMV4420_CHARGE_PUMP_NORMAL_OP;
	st->pump.current_uA = 3;
	st->pump.bleed_en = 1;
	st->pump.bleed_current_uA = 0x0C;
	st->mux_sel = ADMV4420_LOCK_DTCT;

	admv4420_dt_parse(st);

	pr_err("%s: %d: admv4420_setup %x\n", __func__, __LINE__, st->ref_block.diveder);
	ret = regmap_write(st->regmap, ADMV4420_R_DIV_L,
			   FIELD_GET(0xFF, st->ref_block.diveder));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_R_DIV_H,
			   FIELD_GET(0xFF00, st->ref_block.diveder));
	if (ret < 0)
		return ret;
	pr_err("%s: %d: admv4420_setup %x %x %x %x\n", __func__, __LINE__, st->ref_block.divede_by_2_en, ADMV4420_REFERENCE_IN_MODE(st->ref_block.type), ADMV4420_REFERENCE_DOUBLER(st->ref_block.doubler_en),
		st->ref_block.divede_by_2_en |
			   ADMV4420_REFERENCE_IN_MODE(st->ref_block.type) |
			   ADMV4420_REFERENCE_DOUBLER(st->ref_block.doubler_en)
	);
	ret = regmap_write(st->regmap, ADMV4420_REFERENCE,
			   st->ref_block.divede_by_2_en |
			   ADMV4420_REFERENCE_IN_MODE(st->ref_block.type) |
			   ADMV4420_REFERENCE_DOUBLER(st->ref_block.doubler_en));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_FRAC_L,
			   FIELD_GET(0xFF, st->n_counter.frac_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_FRAC_M,
			   FIELD_GET(0xFF00, st->n_counter.frac_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_FRAC_H,
			   FIELD_GET(0xFF0000, st->n_counter.frac_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_MOD_L,
			   FIELD_GET(0xFF, st->n_counter.mod_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_MOD_H,
			   FIELD_GET(0x0300, st->n_counter.mod_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_INT_L,
			   FIELD_GET(0xFF, st->n_counter.int_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_INT_H,
			   FIELD_GET(0xFF00, st->n_counter.int_val));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_PLL_MUX_SEL, st->mux_sel);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, ADMV4420_CP_STATE, 0x03, st->pump.state);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_CP_BLEED_EN, st->pump.bleed_en);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_CP_CURRENT, st->pump.current_uA);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_CP_BLEED, st->pump.bleed_current_uA);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, ADMV4420_ENABLES, ENABLE_PLL | ENABLE_LO | ENABLE_VCO |
			   ENABLE_IFAMP | ENABLE_MIXER | ENABLE_LNA);
	if (ret < 0)
		return ret;

	return 0;
}

static int admv4420_probe(struct spi_device *spi)
{

	struct iio_dev *indio_dev;
	struct admv4420_state *st;
	struct regmap *regmap;
	int ret;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);	
	regmap = devm_regmap_init_spi(spi, &admv4420_regmap_config);
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
	indio_dev->info = &admv4420_info;
pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	ret = admv4420_setup(indio_dev);

	if (ret < 0) {
		dev_err(&spi->dev, "Setup ADMV4420 failed (%d)\n", ret);
		return ret;
	}
	pr_err("%s: %d: Enter ADMV4420 probe\n", __func__, __LINE__);
	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id admv4420_of_match[] = {
	{ .compatible = "adi,admv4420" },
	{ },
};
MODULE_DEVICE_TABLE(of, admv4420_of_match);

static struct spi_driver admv4420_driver = {
	.driver = {
		.name	= "admv4420",
		.of_match_table = admv4420_of_match,
	},
	.probe		= admv4420_probe,
};
module_spi_driver(admv4420_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADMV44200 K Band Downconverter");
MODULE_LICENSE("GPL v2");
