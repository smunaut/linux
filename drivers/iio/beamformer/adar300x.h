// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * header file for ADAV80X parts
 *
 * Copyright 2011 Analog Devices Inc.
 */

#ifndef _ADAR300X_H
#define _ADAR300X_H

#include <linux/iio/buffer-dma.h>

#define ADAR300x_BEAMS_PER_DEVICE	4
#define ADAR300x_ELEMENTS_PER_BEAM	4
#define ADAR300x_CHANNELS_PER_BEAM	8

#define ADAR300x_DELAY_CH(_id, _num, name)			\
{								\
	.type = IIO_PHASE,					\
	.output = true,						\
	.indexed = true,					\
	.channel = (_num),					\
	.address = (_id),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE) |	\
			      BIT(IIO_CHAN_INFO_LABEL),		\
	.label_name = name,					\
	.scan_index = (_id),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 6,					\
		.storagebits = 8,				\
		.shift = 0,					\
	},							\
}

#define ADAR300x_ATTEN_CH(_id, _num, name)			\
{								\
	.type = IIO_POWER,					\
	.indexed = true,					\
	.output = true,						\
	.channel = (_num),					\
	.address = (_id),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE) |	\
			      BIT(IIO_CHAN_INFO_LABEL),		\
	.label_name = name,					\
	.scan_index = (_id),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 6,					\
		.storagebits = 8,				\
		.shift = 0,					\
	},							\
}

#define ADAR300x_TEMP(_id, _num, name)				\
{								\
	.type = IIO_TEMP,					\
	.indexed = true,					\
	.channel = (_num),					\
	.address = (_id),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_OFFSET) |	\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_index = (_id),					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 8,					\
		.storagebits = 8,				\
		.shift = 0,					\
	},							\
}

enum adar300x_ids {
	ID_ADAR3000,
	ID_ADAR3001,
	ID_ADAR3002,
	ID_ADAR3003,
};

enum adar300x_iio_ram_ptrs {
	ADAR300x_PTR0_RAM_START,
	ADAR300x_PTR0_RAM_STOP,
	ADAR300x_PTR1_RAM_START,
	ADAR300x_PTR1_RAM_STOP,
	ADAR300x_PTR2_RAM_START,
	ADAR300x_PTR2_RAM_STOP,
	ADAR300x_PTR3_RAM_START,
	ADAR300x_PTR3_RAM_STOP,
};

enum adar300x_iio_ram_idxs {
	ADAR300x_RAM_INDEX0,
	ADAR300x_RAM_INDEX1,
	ADAR300x_RAM_INDEX2,
	ADAR300x_RAM_INDEX3,
};

enum adar300x_iio_fifo_attr {
	ADAR300x_FIFO_WR0,
	ADAR300x_FIFO_RD0,
	ADAR300x_FIFO_WR1,
	ADAR300x_FIFO_RD1,
	ADAR300x_FIFO_WR2,
	ADAR300x_FIFO_RD2,
	ADAR300x_FIFO_WR3,
	ADAR300x_FIFO_RD3,
};

enum adar300x_iio_amp_bias {
	ADAR300x_RESET_EL0V_AMP,
	ADAR300x_RESET_EL1V_AMP,
	ADAR300x_RESET_EL2V_AMP,
	ADAR300x_RESET_EL3V_AMP,
	ADAR300x_OPERATIONAL_EL0V_AMP,
	ADAR300x_OPERATIONAL_EL1V_AMP,
	ADAR300x_OPERATIONAL_EL2V_AMP,
	ADAR300x_OPERATIONAL_EL3V_AMP,
	ADAR300x_MUTE_EL0V_AMP,
	ADAR300x_MUTE_EL1V_AMP,
	ADAR300x_MUTE_EL2V_AMP,
	ADAR300x_MUTE_EL3V_AMP,
	ADAR300x_SLEEP_EL0V_AMP,
	ADAR300x_SLEEP_EL1V_AMP,
	ADAR300x_SLEEP_EL2V_AMP,
	ADAR300x_SLEEP_EL3V_AMP,
	ADAR300x_OPERATIONAL_EL0H_AMP,
	ADAR300x_OPERATIONAL_EL1H_AMP,
	ADAR300x_OPERATIONAL_EL2H_AMP,
	ADAR300x_OPERATIONAL_EL3H_AMP,
	ADAR300x_SLEEP_EL0H_AMP,
	ADAR300x_SLEEP_EL1H_AMP,
	ADAR300x_SLEEP_EL2H_AMP,
	ADAR300x_SLEEP_EL3H_AMP,
};

enum adar300x_beamstate_mode_ctrl {
	ADAR300x_DIRECT_CTRL,
	ADAR300x_MEMORY_CTRL,
	ADAR300x_FIFO_CTRL,
	ADAR300x_INST_DIRECT_CTRL,
	ADAR300x_RESET,
	ADAR300x_MUTE,
};

struct adar300x_chip_info {
	unsigned int			chip_id;
	unsigned int			num_channels;
	const struct iio_chan_spec	*channels;
	unsigned int			unpacked_beamst_len;
	unsigned int			packed_beamst_len;
	unsigned int			product_id;
};

struct adar300x_state {
	struct iio_dev				*indio_dev;
	struct spi_device			*spi;
	struct regmap				*regmap;
	const struct adar300x_chip_info		*chip_info;
	u16					dev_addr;
	u8					beam_index[ADAR300x_BEAMS_PER_DEVICE];
	u8			state_buf[ADAR300x_BEAMS_PER_DEVICE][ADAR300x_CHANNELS_PER_BEAM];
	enum adar300x_beamstate_mode_ctrl	beam_mode[ADAR300x_BEAMS_PER_DEVICE];
	enum adar300x_beamstate_mode_ctrl	beam_load_mode[ADAR300x_BEAMS_PER_DEVICE];
	struct mutex				lock;
	struct iio_dma_buffer_queue		queue;
	struct iio_buffer			*dma_buffer;
	struct gpio_desc			*gpio_reset;
};

ssize_t adar300x_ram_range_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len);

ssize_t adar300x_ram_range_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

ssize_t adar300x_ram_index_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len);

ssize_t adar300x_ram_index_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

ssize_t adar300x_fifo_ptr_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf);

ssize_t adar300x_show_update_intf_ctrl_available(struct device *dev,
						 struct device_attribute *attr,
						 char *buf);

ssize_t adar300x_update_intf_ctrl_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len);

ssize_t adar300x_update_intf_ctrl_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

ssize_t adar300x_update_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len);

ssize_t adar300x_update_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf);

ssize_t adar300x_load_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len);

ssize_t adar300x_load_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf);

ssize_t adar300x_show_mode_available(struct device *dev,
				     struct device_attribute *attr,
				     char *buf);

ssize_t adar300x_mode_store(struct device *dev,
			    struct device_attribute *attr,
			    const char *buf, size_t len);

ssize_t adar300x_mode_show(struct device *dev,
			   struct device_attribute *attr,
			   char *buf);

ssize_t adar300x_amp_bias_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len);

ssize_t adar300x_amp_bias_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf);

ssize_t adar300x_amp_en_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t len);

ssize_t adar300x_amp_en_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf);

int adar300x_probe(struct spi_device *spi,
		   const struct attribute_group *attr_group);

#endif
