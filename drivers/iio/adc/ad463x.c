// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD463X SPI ADC driver
 *
 * Copyright 2021 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>

/* Register addresses */
#define AD463X_REG_INTERFACE_CONFIG_A	0x00
#define AD463X_REG_INTERFACE_CONFIG_B	0x01
#define AD463X_REG_DEVICE_CONFIG	0x02
#define AD463X_REG_CHIP_TYPE		0x03
#define AD463X_REG_PRODUCT_ID_L		0x04
#define AD463X_REG_PRODUCT_ID_H		0x05
#define AD463X_REG_CHIP_GRADE		0x06
#define AD463X_REG_SCRATCH_PAD		0x0A
#define AD463X_REG_SPI_REVISION		0x0B
#define AD463X_REG_VENDOR_L		0x0C
#define AD463X_REG_VENDOR_H		0x0D
#define AD463X_REG_STREAM_MODE		0x0E
#define AD463X_REG_EXIT_CFG_MODE	0x14
#define AD463X_REG_AVG			0x15
#define AD463X_REG_OFFSET_BASE		0x16
#define AD463X_REG_OFFSET_X0_0		0x16
#define AD463X_REG_OFFSET_X0_1		0x17
#define AD463X_REG_OFFSET_X0_2		0x18
#define AD463X_REG_OFFSET_X1_0		0x19
#define AD463X_REG_OFFSET_X1_1		0x1A
#define AD463X_REG_OFFSET_X1_2		0x1B
#define AD463X_REG_GAIN_BASE		0x1C
#define AD463X_REG_GAIN_X0_LSB		0x1C
#define AD463X_REG_GAIN_X0_MSB		0x1D
#define AD463X_REG_GAIN_X1_LSB		0x1E
#define AD463X_REG_GAIN_X1_MSB		0x1F
#define AD463X_REG_MODES		0x20
#define AD463X_REG_OSCILATOR		0x21
#define AD463X_REG_IO			0x22
#define AD463X_REG_PAT0			0x23
#define AD463X_REG_PAT1			0x24
#define AD463X_REG_PAT2			0x25
#define AD463X_REG_PAT3			0x26
#define AD463X_REG_DIG_DIAG		0x34
#define AD463X_REG_DIG_ERR		0x35
/* MODES */
#define AD463X_LANE_MODE_MSK		GENMASK(7, 6)
#define AD463X_CLK_MODE_MSK		GENMASK(5, 4)
#define AD463X_DATA_RATE_MODE_MSK	BIT(3)
#define AD463X_OUT_DATA_MODE_MSK	GENMASK(2, 0)
/* EXIT_CFG_MD */
#define AD463X_EXIT_CFG_MODE		BIT(0)
/* AVG */
#define AD463X_AVG_FILTER_RESET		BIT(7)
#define AD463X_AVG_LEN_DEFAULT		0x06

#define AD463X_REG_CHAN_OFFSET(ch, pos)	(AD463X_REG_OFFSET_BASE + (3*ch) + pos)
#define AD463X_REG_CHAN_GAIN(ch, pos)	(AD463X_REG_GAIN_BASE + (2 * ch) + pos)

#define AD463X_CONFIG_TIMING		0x2000
#define AD463X_REG_READ_DUMMY		0x00
#define AD463X_REG_WRITE_MASK(x)	(x & 0x7FFF)
#define AD463X_REG_READ_MASK(x)		(x | BIT(15))

#define AD463X_SPI_REG_ACCESS_SPEED	40000000UL
#define AD463X_SPI_SAMPLING_SPEED	80000000UL
#define AD463X_SPI_WIDTH(mode, width)	(width >> (mode >> 6))

#define AD463X_FREQ_TO_PERIOD(f)	DIV_ROUND_UP(USEC_PER_SEC, \
						     (f / NSEC_PER_USEC))
#define AD463X_TRIGGER_PULSE_WIDTH_NS	10

#define AD463X_T_CONV_HI		10
#define AD463X_T_CONV			300

#define AD4630_24_CHANNEL(_name, _idx, _sidx,				\
			  _storagebits, _realbits, _shift)		\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN)|	\
				      BIT(IIO_CHAN_INFO_OFFSET),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.indexed = 1,						\
		.extend_name = _name,					\
		.channel = _idx,					\
		.scan_index = _sidx,					\
		.ext_info = ad463x_ext_info,				\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = _storagebits,			\
			.realbits = _realbits,				\
			.shift = _shift,				\
		},							\
	}

enum ad463x_id {
	ID_AD4630_24,
	ID_AD4630_20,
	ID_AD4630_16,
	ID_AD4631_24,
	ID_AD4631_20,
	ID_AD4631_16,
	ID_AD4632_24,
	ID_AD4632_20,
	ID_AD4632_16,
};

enum ad463x_lane_mode {
	AD463X_ONE_LANE_PER_CH	= 0x00,
	AD463X_TWO_LANES_PER_CH	= BIT(6),
	AD463X_FOUR_LANES_PER_CH = BIT(7),
	AD463X_SHARED_TWO_CH = (BIT(6) | BIT(7)),
};

enum ad463x_clock_mode {
	AD463X_SPI_COMPATIBLE_MODE = 0x00,
	AD463X_ECHO_CLOCK_MODE = BIT(4),
	AD463X_CLOCK_MASTER_MODE = BIT(5),
};

enum ad463x_data_rate_mode {
	AD463X_SINGLE_DATA_RATE = 0x00,
	AD463X_DUAL_DATA_RATE = BIT(3),
};

enum ad463x_out_data_mode {
	AD463X_24_DIFF = 0x00,
	AD463X_16_DIFF_8_COM = 0x01,
	AD463X_24_DIFF_8_COM = 0x02,
	AD463X_30_AVERAGED_DIFF = 0x03,
	AD463X_32_PATTERN = 0x04
};

enum ad463x_power_mode {
	AD463X_NORMAL_OPERATING_MODE = 0,
	AD463X_LOW_POWER_MODE = (BIT(0) | BIT(1)),
};

static const char * const ad463x_lane_mdoes[] = {
	[AD463X_SHARED_TWO_CH] = "one-lane-shared",
	[AD463X_ONE_LANE_PER_CH] = "one-lane-per-ch",
	[AD463X_TWO_LANES_PER_CH] = "two-lanes-per-ch",
	[AD463X_FOUR_LANES_PER_CH] = "four-lanes-per-ch",
};

static const char * const ad463x_clock_mdoes[] = {
	[AD463X_SPI_COMPATIBLE_MODE] = "spi-compatible",
	[AD463X_ECHO_CLOCK_MODE] = "echo-clock",
	[AD463X_CLOCK_MASTER_MODE] = "clock-master",
};

static const char * const ad463x_data_rates[] = {
	[AD463X_SINGLE_DATA_RATE] = "single",
	[AD463X_DUAL_DATA_RATE] = "dual"
};

static const char * const ad463x_out_data_modes[] = {
	[AD463X_16_DIFF_8_COM] = "16diff-8com",
	[AD463X_24_DIFF] = "24diff",
	[AD463X_24_DIFF_8_COM] = "24diff-8com",
	[AD463X_30_AVERAGED_DIFF] = "30diff-avg",
	[AD463X_32_PATTERN] = "32pat"
};

static const unsigned int ad463x_data_widths[] = {
	[AD463X_16_DIFF_8_COM] = 24,
	[AD463X_24_DIFF] = 24,
	[AD463X_24_DIFF_8_COM] = 32,
	[AD463X_30_AVERAGED_DIFF] = 32,
	[AD463X_32_PATTERN] = 32
};

static const char * const ad463x_power_modes[] = {
	[AD463X_LOW_POWER_MODE] = "low_power_mode",
	[AD463X_NORMAL_OPERATING_MODE] = "normal_operating_mode",
};

static const char * const ad463x_average_modes[] = {
	"OFF", "2", "4", "8", "16", "32", "64", "128", "256", "512",
	"1024", "2048", "4096", "8192", "16384", "32768", "65536",
};

static const unsigned int ad463x_sampling_rates[] = {
	10000, 50000, 100000, 200000, 500000, 1000000, 1750000, 2000000,
};

struct ad463x_phy_config {
	enum ad463x_data_rate_mode	data_rate_mode;
	enum ad463x_out_data_mode	out_data_mode;
	enum ad463x_clock_mode		clock_mode;
	enum ad463x_lane_mode		lane_mode;
	int				num_avg_samples;
};

struct ad463x_channel_config {
	unsigned int offset;
	unsigned short gain;
};

struct ad463x_state {
	struct spi_device		*spi;
	struct pwm_device		*conversion_trigger;
	struct pwm_device		*spi_engine_trigger;
	struct gpio_desc		*gpio_reset;
	struct ad463x_phy_config	phy;
	struct ad463x_channel_config	channel_cfg[2];
	unsigned int			sampling_frequency;

	union {
		unsigned char		buff[3];
		struct {
			unsigned short	reg_addr;
			unsigned char	reg_data;
		};
	} adc_data[2] ____cacheline_aligned;
};

static int ad463x_spi_read_reg(struct ad463x_state *st,
			       unsigned int reg_addr,
			       unsigned int *reg_data)
{
	struct spi_transfer xfer = {
		.tx_buf = st->adc_data[0].buff,
		.rx_buf = st->adc_data[1].buff,
		.len = 3,
		.bits_per_word = 8,
		.speed_hz = AD463X_SPI_REG_ACCESS_SPEED
	};
	int ret;

	st->adc_data[0].reg_addr = cpu_to_be16(AD463X_REG_READ_MASK(reg_addr));
	st->adc_data[0].reg_data = AD463X_REG_READ_DUMMY;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	*reg_data = (unsigned int)st->adc_data[1].reg_data;

	return ret;
}

static int ad463x_spi_write_reg(struct ad463x_state *st,
				unsigned int reg_addr,
				unsigned int reg_data)
{
	struct spi_transfer xfer = {
		.tx_buf = st->adc_data[0].buff,
		.len = 3,
		.bits_per_word = 8,
		.speed_hz = AD463X_SPI_REG_ACCESS_SPEED
	};

	st->adc_data[0].reg_addr = cpu_to_be16(AD463X_REG_WRITE_MASK(reg_addr));
	st->adc_data[0].reg_data = reg_data;

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad463x_spi_write_reg_masked(struct ad463x_state *st,
				       unsigned int reg_addr,
				       unsigned int mask,
				       unsigned int reg_data)
{
	unsigned int temp;
	int ret;

	ret = ad463x_spi_read_reg(st, reg_addr, &temp);
	if (ret < 0)
		return ret;

	return ad463x_spi_write_reg(st, reg_addr, ((temp & ~mask) | reg_data));
}

static int ad463x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	int ret;
	struct ad463x_state *st = iio_priv(indio_dev);

	if (readval) {
		ret = ad463x_spi_read_reg(st, reg, readval);
	} else {
		// if (reg == AD463X_EXIT_CFG_MODE ||
		//     reg == AD463X_REG_AVG ||
		//     reg == AD463X_REG_MODES)
		// 	return -EINVAL;
		ret = ad463x_spi_write_reg(st, reg, writeval);
	}

	return ret;
}

static int ad463x_set_reg_access(struct ad463x_state *st, bool state)
{
	unsigned int dummy;

	if (state)
		/* Send a sequence starting with "1 0 1" to the SPI bus*/
		return ad463x_spi_read_reg(st, AD463X_CONFIG_TIMING, &dummy);
	else
		return ad463x_spi_write_reg(st, AD463X_REG_EXIT_CFG_MODE,
					AD463X_EXIT_CFG_MODE);
}

static ssize_t ad463x_sampling_freq_avail(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad463x_sampling_rates); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
				 ad463x_sampling_rates[i]);
	buf[len - 1] = '\n';

	return len;
}
static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(ad463x_sampling_freq_avail);

static int ad463x_set_sampling_freq(struct ad463x_state *st,
				    unsigned int freq)
{
	struct pwm_state conversion_state, spi_trigger_state;
	int ret;

	conversion_state.period = AD463X_FREQ_TO_PERIOD(freq);
	conversion_state.duty_cycle = AD463X_TRIGGER_PULSE_WIDTH_NS;
	conversion_state.offset = 0;
	conversion_state.enabled = true;
	
	ret = pwm_apply_state(st->conversion_trigger, &conversion_state);
	if (ret < 0)
		return ret;

	spi_trigger_state.duty_cycle = conversion_state.duty_cycle ;
	spi_trigger_state.period = conversion_state.period;
	spi_trigger_state.offset = 10;
	spi_trigger_state.enabled = true;

	if (st->phy.out_data_mode == AD463X_30_AVERAGED_DIFF)
		spi_trigger_state.period *= st->phy.num_avg_samples;

	ret = pwm_apply_state(st->spi_engine_trigger, &spi_trigger_state);
	if (ret < 0)
		return ret;

	st->sampling_frequency = freq;

	return 0;
}

static int ad463x_set_conversion(struct ad463x_state *st, bool enabled)
{
	struct pwm_state conversion_state, spi_trigger_state;
	int ret;

	pwm_get_state(st->conversion_trigger, &conversion_state);
	conversion_state.enabled = enabled;

	ret = pwm_apply_state(st->conversion_trigger, &conversion_state);
	if (ret < 0)
		return ret;

	pwm_get_state(st->spi_engine_trigger, &spi_trigger_state);
	spi_trigger_state.enabled = enabled;

	ret = pwm_apply_state(st->spi_engine_trigger, &spi_trigger_state);
	if (ret < 0)
		return ret;		

	return 0;
}

static int ad463x_get_avg_frame_len(struct iio_dev *dev,
				    const struct iio_chan_spec *chan)
{
	struct ad463x_state *st = iio_priv(dev);
	unsigned int avg_len;
	int ret;

	if (st->phy.out_data_mode != AD463X_30_AVERAGED_DIFF) {
		return 0;
	} else {
		ret = ad463x_spi_read_reg(st, AD463X_REG_AVG, &avg_len);
		if (ret < 0)
			return ret;
	}

	return avg_len;
}

static int ad463x_set_avg_frame_len(struct iio_dev *dev,
				    const struct iio_chan_spec *chan,
				    unsigned int avg_len)
{
	int ret;
	struct ad463x_state *st = iio_priv(dev);

	if (st->phy.out_data_mode != AD463X_30_AVERAGED_DIFF || avg_len == 0)
		return -EINVAL;
	
	st->phy.num_avg_samples = 1 << avg_len;

	// ad463x_set_conversion(st, false);

	// ret = ad463x_spi_write_reg(st, AD463X_REG_AVG, AD463X_AVG_FILTER_RESET);
	// if (ret < 0)
	// 	return ret;

	ret = ad463x_set_sampling_freq(st, st->sampling_frequency);
	if (ret < 0)
		return ret;

	ret = ad463x_spi_write_reg(st, AD463X_REG_AVG, avg_len);
	if (ret < 0)
		return ret;

	// ad463x_set_conversion(st, true);
	
	return ret;
}

static int ad463x_get_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan)
{
	unsigned int mode;
	struct ad463x_state *st = iio_priv(dev);

	ad463x_spi_read_reg(st, AD463X_REG_DEVICE_CONFIG, &mode);

	return mode;
}

static int ad463x_set_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan,
			       unsigned int mode)
{
	struct ad463x_state *st = iio_priv(dev);

	return ad463x_spi_write_reg(st, AD463X_REG_DEVICE_CONFIG, mode);
}

static int ad463x_phy_init(struct ad463x_state *st)
{
	int ret;

	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_LANE_MODE_MSK,
					  st->phy.lane_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_CLK_MODE_MSK,
					  st->phy.clock_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_DATA_RATE_MODE_MSK,
					  st->phy.data_rate_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg_masked(st, AD463X_REG_MODES,
					  AD463X_OUT_DATA_MODE_MSK,
					  st->phy.out_data_mode);
	if (ret < 0)
		return ret;

	if (st->phy.out_data_mode == AD463X_30_AVERAGED_DIFF) {
		ret = ad463x_spi_write_reg(st, AD463X_REG_AVG,
					   AD463X_AVG_LEN_DEFAULT);
		if (ret < 0)
			return ret;
		st->phy.num_avg_samples = 64;
	}

	return 0;
}

static int ad463x_get_string_index(const char *string,
				   const char * const *string_array,
				   size_t array_size,
				   unsigned int *idx)
{
	int i;

	for (i = 0; i < array_size; i++) {
		if (string_array[i]) {
			if (!strcmp(string, string_array[i])) {
				*idx = i;
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int ad463x_parse_dt(struct ad463x_state *st)
{
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	struct fwnode_handle *fwnode;
	const char *data_rate_mode;
	const char *out_data_mode;
	const char *clock_mode;
	const char *lane_mode;
	int ret;

	fwnode = dev_fwnode(indio_dev->dev.parent);

	ret = fwnode_property_read_string(fwnode, "adi,lane-mode",
					  &lane_mode);
	if (ret < 0)
		return ret;
	ret = fwnode_property_read_string(fwnode, "adi,clock-mode",
					  &clock_mode);
	if (ret < 0)
		return ret;
	ret = fwnode_property_read_string(fwnode, "adi,data-rate-mode",
					  &data_rate_mode);
	if (ret < 0)
		return ret;
	ret = fwnode_property_read_string(fwnode, "adi,out-data-mode",
					  &out_data_mode);
	if (ret < 0)
		return ret;

	ret = ad463x_get_string_index(lane_mode, ad463x_lane_mdoes,
				      ARRAY_SIZE(ad463x_lane_mdoes),
				      &st->phy.lane_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_get_string_index(clock_mode, ad463x_clock_mdoes,
				      ARRAY_SIZE(ad463x_clock_mdoes),
				      &st->phy.clock_mode);
	if (ret < 0)
		return ret;
	ret = ad463x_get_string_index(data_rate_mode, ad463x_data_rates,
				      ARRAY_SIZE(ad463x_data_rates),
				      &st->phy.data_rate_mode);
	if (ret < 0)
		return ret;

	return ad463x_get_string_index(out_data_mode, ad463x_out_data_modes,
				       ARRAY_SIZE(ad463x_out_data_modes),
				       &st->phy.out_data_mode);
}

static int ad463x_set_chan_offset(struct ad463x_state *st, int chan_idx, unsigned int offset)
{
	int ret;

	ret = ad463x_spi_write_reg(st, AD463X_REG_CHAN_OFFSET(chan_idx, 0), offset);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg(st, AD463X_REG_CHAN_OFFSET(chan_idx, 1), offset >> 8);
	if (ret < 0)
		return ret;
	ret = ad463x_spi_write_reg(st, AD463X_REG_CHAN_OFFSET(chan_idx, 2), offset >> 16);
	if (ret < 0)
		return ret;

	st->channel_cfg[chan_idx].offset = offset;

	return 0;
}

static int ad463x_set_chan_gain(struct ad463x_state *st, int chan_idx, int gain_int, int gain_frac)
{
	int ret;
	unsigned int gain;

	gain = ((abs(gain_int) * 10000) + (abs(gain_frac) / 100));
	gain *= 0x8000;
	gain /= 10000;

	ret = ad463x_spi_write_reg(st, AD463X_REG_CHAN_GAIN(chan_idx, 0), gain);
	if (ret < 0)
		return ret;

	ret = ad463x_spi_write_reg(st, AD463X_REG_CHAN_GAIN(chan_idx, 1), gain >> 8);
	if (ret < 0)
		return ret;

	st->channel_cfg[chan_idx].gain = gain;

	return 0;
}

static int ad463x_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long info)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	unsigned int temp;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_frequency;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		temp = st->channel_cfg[chan->scan_index].gain * 10000;
		temp /= 0x8000;
		*val = temp / 10000;
		*val2 = (temp - (*val * 10000)) * 100;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		*val = st->channel_cfg[chan->scan_index].offset;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad463x_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	int ret;
	struct ad463x_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad463x_set_sampling_freq(st, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val > 1)
			return -EINVAL;

		return ad463x_set_chan_gain(st, chan->channel, val, val2);
	case IIO_CHAN_INFO_OFFSET:
		return ad463x_set_chan_offset(st, chan->channel, val);
	default:
		return -EINVAL;
	}

	return ret;
}

static int ad463x_setup(struct ad463x_state *st)
{
	int ret;

	st->gpio_reset = devm_gpiod_get(&st->spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	gpiod_direction_output(st->gpio_reset, 1);
	gpiod_set_value(st->gpio_reset, 0);

	ret = ad463x_set_reg_access(st, true);
	if (ret < 0)
		return ret;

	ret = ad463x_phy_init(st);
	if (ret < 0)
		return ret;

	ret = ad463x_set_chan_gain(st, 0, 1, 0);
	if (ret < 0)
		return ret;

	ret = ad463x_set_chan_gain(st, 1, 1, 0);
	if (ret < 0)
		return ret;

	ret = ad463x_set_chan_offset(st, 0, 0);
	if (ret < 0)
		return ret;

	ret = ad463x_set_chan_offset(st, 1, 0);
	if (ret < 0)
		return ret;

	ad463x_set_conversion(st, false);
	return ad463x_set_sampling_freq(st, 1000000);
}

static int ad463x_dma_buffer_submit_block(struct iio_dma_buffer_queue *queue,
					  struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static int ad463x_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	struct spi_message msg;
	unsigned int data[4];
	int ret;

	struct spi_transfer xfer = {
		.tx_buf = NULL,
		.rx_buf = data,
		.len = 1,
		.speed_hz = AD463X_SPI_SAMPLING_SPEED,
	};

	ret = ad463x_set_reg_access(st, false);
	if (ret < 0)
		return ret;

	ad463x_set_conversion(st, true);

	xfer.bits_per_word = AD463X_SPI_WIDTH(
			st->phy.lane_mode,
			ad463x_data_widths[st->phy.out_data_mode]);
	
	if (st->phy.data_rate_mode == AD463X_DUAL_DATA_RATE) 
		xfer.bits_per_word /= 2;

	ret = ad463x_set_reg_access(st, false);
	if (ret < 0)
		return ret;
	ad463x_set_conversion(st, true);

	spi_bus_lock(st->spi->master);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	spi_engine_offload_load_msg(st->spi, &msg);
	spi_engine_offload_enable(st->spi, true);

	return ret;
}

static int ad463x_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad463x_state *st = iio_priv(indio_dev);
	int ret;

	spi_engine_offload_enable(st->spi, false);
	spi_bus_unlock(st->spi->master);

	ret = ad463x_set_reg_access(st, true);
	if (ret < 0)
		return ret;

	return ad463x_set_conversion(st, false);
	return ret;
}

static void ad463x_cnv_diasble(void *data)
{
	pwm_disable(data);
}

static void ad463x_avg_trigger_disable(void *data)
{
	pwm_disable(data);
}

static const struct iio_enum ad463x_avg_frame_len_enum = {
	.items = ad463x_average_modes,
	.num_items = ARRAY_SIZE(ad463x_average_modes),
	.set = ad463x_set_avg_frame_len,
	.get = ad463x_get_avg_frame_len,
};

static const struct iio_enum ad463x_power_mode_enum = {
	.items = ad463x_power_modes,
	.num_items = ARRAY_SIZE(ad463x_power_modes),
	.set = ad463x_set_pwr_mode,
	.get = ad463x_get_pwr_mode,
};

static struct iio_chan_spec_ext_info ad463x_ext_info[] = {
	IIO_ENUM("sample_averaging",
		 IIO_SHARED_BY_ALL,
		 &ad463x_avg_frame_len_enum),
	IIO_ENUM_AVAILABLE_SHARED("sample_averaging",
				  IIO_SHARED_BY_ALL,
				  &ad463x_avg_frame_len_enum),
	IIO_ENUM("operating_mode",
		 IIO_SHARED_BY_ALL,
		 &ad463x_power_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("operating_mode",
				  IIO_SHARED_BY_ALL,
				  &ad463x_power_mode_enum),
	{ },
};

static const struct iio_chan_spec ad463x_channels[][5][4] = {
	[ID_AD4630_24] = {
		[AD463X_16_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential_0",   0, 0, 32, 16, 8),
			AD4630_24_CHANNEL("common_voltage_0", 1, 0, 32, 8,  0),
			AD4630_24_CHANNEL("differential_1",   2, 1, 32, 16, 8),
			AD4630_24_CHANNEL("common_voltage_1", 3, 1, 32, 8,  0),
		},
		[AD463X_24_DIFF] = {
			AD4630_24_CHANNEL("differential_0", 0, 0, 32, 24, 0),
			AD4630_24_CHANNEL("differential_1", 1, 1, 32, 24, 0),
		},
		[AD463X_24_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential_0",   0, 0, 32, 24, 8),
			AD4630_24_CHANNEL("common_voltage_0", 1, 0, 32, 8,  0),
			AD4630_24_CHANNEL("differential_1",   2, 1, 32, 24, 8),
			AD4630_24_CHANNEL("common_voltage_1", 3, 1, 32, 8,  0),
		},
		[AD463X_30_AVERAGED_DIFF] = {
			AD4630_24_CHANNEL("differential_0", 0, 0, 32, 30, 2),
			AD4630_24_CHANNEL("differential_1", 1, 1, 32, 30, 2),
		},
		[AD463X_32_PATTERN] = {
			AD4630_24_CHANNEL("pattern_0", 0, 0, 32, 32, 0),
			AD4630_24_CHANNEL("pattern_1", 1, 1, 32, 32, 0),
		},
	},
};

static const struct iio_dma_buffer_ops ad463x_dma_buffer_ops = {
	.submit = ad463x_dma_buffer_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad463x_buffer_setup_ops = {
	.preenable = &ad463x_buffer_preenable,
	.postdisable = &ad463x_buffer_postdisable,
};

static const struct spi_device_id ad463x_id_table[] = {
	{"ad4630-24",	ID_AD4630_24},
	{}
};
MODULE_DEVICE_TABLE(spi, ad463x_id_table);

static const struct of_device_id ad463x_of_match[] = {
	{ .compatible = "adi,ad4630-24" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad463x_of_match);

static struct attribute *ad463x_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad463x_group = {
	.attrs = ad463x_attributes,
};

static const struct iio_info ad463x_infos[] = {
	[ID_AD4630_24] = {
		.attrs = &ad463x_group,
		.read_raw = &ad463x_read_raw,
		.write_raw = &ad463x_write_raw,
		.debugfs_reg_access = &ad463x_reg_access,
	}
};

static int ad463x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad463x_state *st;
	unsigned long device_id;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;
	st->conversion_trigger = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(st->conversion_trigger))
		return PTR_ERR(st->conversion_trigger);

	ret = devm_add_action_or_reset(&spi->dev, ad463x_cnv_diasble,
				       st->conversion_trigger);
	if (ret < 0)
		return ret;

	st->spi_engine_trigger = devm_pwm_get(&spi->dev, "spi_trigger");
	if (IS_ERR(st->spi_engine_trigger))
		return PTR_ERR(st->spi_engine_trigger);

	ret = devm_add_action_or_reset(&spi->dev,
					ad463x_avg_trigger_disable,
					st->spi_engine_trigger);

	device_id = spi_get_device_id(st->spi)->driver_data;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad463x_infos[device_id];
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad463x_buffer_setup_ops;

	ret = ad463x_parse_dt(st);
	if (ret < 0) {
		dev_err(&spi->dev, "%s invalid devicetree configuration\n",
			indio_dev->name);
		return -EINVAL;
	}

	indio_dev->channels =
		ad463x_channels[device_id][st->phy.out_data_mode];
	indio_dev->num_channels = 2;
	if (st->phy.out_data_mode == AD463X_16_DIFF_8_COM ||
	    st->phy.out_data_mode == AD463X_24_DIFF_8_COM)
		indio_dev->num_channels *= 2;

	ret = ad463x_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "%s setup failed\n", indio_dev->name);
		return -ENOEXEC;
	}

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						 &ad463x_dma_buffer_ops,
						 indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad463x_driver = {
	.driver = {
		.name = "ad463x",
		.of_match_table = ad463x_of_match,
	},
	.probe = ad463x_probe,
	.id_table = ad463x_id_table,
};
module_spi_driver(ad463x_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD463x ADC family driver");
MODULE_LICENSE("GPL v2");
