// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * ADAR3000, ADAR3001, ADAR3002, ADAR3003 device driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/spi/spi.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include "adar300x.h"

#define ADAR3000_PRODUCT_ID 0x01

enum adar3000_iio_dev_attr {
	ADAR3000_BEAM0,
	ADAR3000_BEAM1,
	ADAR3000_BEAM2,
	ADAR3000_BEAM3,
	ADAR3000_BEAMS_NO,
};

static IIO_DEVICE_ATTR(beam0_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3000_BEAM0);

static IIO_DEVICE_ATTR(beam1_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3000_BEAM1);

static IIO_DEVICE_ATTR(beam2_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3000_BEAM2);

static IIO_DEVICE_ATTR(beam3_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3000_BEAM3);

static IIO_DEVICE_ATTR(update_intf_ctrl_available, 0444,
		       adar300x_show_update_intf_ctrl_available, NULL, 0);

static IIO_DEVICE_ATTR(update_intf_ctrl, 0644,
		       adar300x_update_intf_ctrl_show, adar300x_update_intf_ctrl_store, 0);

static IIO_DEVICE_ATTR(beam0_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3000_BEAM0);

static IIO_DEVICE_ATTR(beam1_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3000_BEAM1);

static IIO_DEVICE_ATTR(beam2_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3000_BEAM2);

static IIO_DEVICE_ATTR(beam3_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3000_BEAM3);

static IIO_DEVICE_ATTR(beam0_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam1_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam2_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam3_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam0_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3000_BEAM0);

static IIO_DEVICE_ATTR(beam1_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3000_BEAM1);

static IIO_DEVICE_ATTR(beam2_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3000_BEAM2);

static IIO_DEVICE_ATTR(beam3_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3000_BEAM3);

static IIO_DEVICE_ATTR(beam0_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam1_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam2_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam3_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(beam0_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR0_RAM_START);

static IIO_DEVICE_ATTR(beam1_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR1_RAM_START);

static IIO_DEVICE_ATTR(beam2_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR2_RAM_START);

static IIO_DEVICE_ATTR(beam3_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR3_RAM_START);

static IIO_DEVICE_ATTR(beam0_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR0_RAM_STOP);

static IIO_DEVICE_ATTR(beam1_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR1_RAM_STOP);

static IIO_DEVICE_ATTR(beam2_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR2_RAM_STOP);

static IIO_DEVICE_ATTR(beam3_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR3_RAM_STOP);

static IIO_DEVICE_ATTR(beam0_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX0);

static IIO_DEVICE_ATTR(beam1_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX1);

static IIO_DEVICE_ATTR(beam2_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX2);

static IIO_DEVICE_ATTR(beam3_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX3);

static IIO_DEVICE_ATTR(beam0_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD0);

static IIO_DEVICE_ATTR(beam1_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD1);

static IIO_DEVICE_ATTR(beam2_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD2);

static IIO_DEVICE_ATTR(beam3_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD3);

static IIO_DEVICE_ATTR(beam0_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR0);

static IIO_DEVICE_ATTR(beam1_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR1);

static IIO_DEVICE_ATTR(beam2_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR2);

static IIO_DEVICE_ATTR(beam3_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR3);

static IIO_DEVICE_ATTR(amp_bias_reset_EL0V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_RESET_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_bias_reset_EL1V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_RESET_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_bias_reset_EL2V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_RESET_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_bias_reset_EL3V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_RESET_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL0V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL1V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL2V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL3V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_bias_mute_EL0V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_MUTE_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_bias_mute_EL1V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_MUTE_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_bias_mute_EL2V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_MUTE_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_bias_mute_EL3V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_MUTE_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL0V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL1V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL2V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL3V, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL0H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL0H_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL1H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL1H_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL2H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL2H_AMP);

static IIO_DEVICE_ATTR(amp_bias_operational_EL3H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store,
		       ADAR300x_OPERATIONAL_EL3H_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL0H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL0H_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL1H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL1H_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL2H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL2H_AMP);

static IIO_DEVICE_ATTR(amp_bias_sleep_EL3H, 0644,
		       adar300x_amp_bias_show, adar300x_amp_bias_store, ADAR300x_SLEEP_EL3H_AMP);

static IIO_DEVICE_ATTR(amp_en_reset_EL0V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_RESET_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_en_reset_EL1V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_RESET_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_en_reset_EL2V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_RESET_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_en_reset_EL3V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_RESET_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL0V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL1V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL2V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL3V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_en_mute_EL0V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_MUTE_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_en_mute_EL1V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_MUTE_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_en_mute_EL2V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_MUTE_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_en_mute_EL3V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_MUTE_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL0V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL0V_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL1V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL1V_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL2V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL2V_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL3V, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL3V_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL0H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL0H_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL1H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL1H_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL2H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL2H_AMP);

static IIO_DEVICE_ATTR(amp_en_operational_EL3H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_OPERATIONAL_EL3H_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL0H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL0H_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL1H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL1H_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL2H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL2H_AMP);

static IIO_DEVICE_ATTR(amp_en_sleep_EL3H, 0644,
		       adar300x_amp_en_show, adar300x_amp_en_store, ADAR300x_SLEEP_EL3H_AMP);

static struct attribute *adar3000_attributes[] = {
	&iio_dev_attr_beam0_update.dev_attr.attr,
	&iio_dev_attr_beam1_update.dev_attr.attr,
	&iio_dev_attr_beam2_update.dev_attr.attr,
	&iio_dev_attr_beam3_update.dev_attr.attr,

	&iio_dev_attr_beam0_mode_available.dev_attr.attr,
	&iio_dev_attr_beam0_mode.dev_attr.attr,
	&iio_dev_attr_beam1_mode_available.dev_attr.attr,
	&iio_dev_attr_beam1_mode.dev_attr.attr,
	&iio_dev_attr_beam2_mode_available.dev_attr.attr,
	&iio_dev_attr_beam2_mode.dev_attr.attr,
	&iio_dev_attr_beam3_mode_available.dev_attr.attr,
	&iio_dev_attr_beam3_mode.dev_attr.attr,

	&iio_dev_attr_beam0_load_mode_available.dev_attr.attr,
	&iio_dev_attr_beam0_load_mode.dev_attr.attr,
	&iio_dev_attr_beam1_load_mode_available.dev_attr.attr,
	&iio_dev_attr_beam1_load_mode.dev_attr.attr,
	&iio_dev_attr_beam2_load_mode_available.dev_attr.attr,
	&iio_dev_attr_beam2_load_mode.dev_attr.attr,
	&iio_dev_attr_beam3_load_mode_available.dev_attr.attr,
	&iio_dev_attr_beam3_load_mode.dev_attr.attr,
	&iio_dev_attr_beam0_ram_start.dev_attr.attr,

	&iio_dev_attr_beam1_ram_start.dev_attr.attr,
	&iio_dev_attr_beam2_ram_start.dev_attr.attr,
	&iio_dev_attr_beam3_ram_start.dev_attr.attr,
	&iio_dev_attr_beam0_ram_stop.dev_attr.attr,
	&iio_dev_attr_beam1_ram_stop.dev_attr.attr,
	&iio_dev_attr_beam2_ram_stop.dev_attr.attr,
	&iio_dev_attr_beam3_ram_stop.dev_attr.attr,
	&iio_dev_attr_beam0_ram_index.dev_attr.attr,
	&iio_dev_attr_beam1_ram_index.dev_attr.attr,
	&iio_dev_attr_beam2_ram_index.dev_attr.attr,
	&iio_dev_attr_beam3_ram_index.dev_attr.attr,

	&iio_dev_attr_beam0_fifo_rd.dev_attr.attr,
	&iio_dev_attr_beam1_fifo_rd.dev_attr.attr,
	&iio_dev_attr_beam2_fifo_rd.dev_attr.attr,
	&iio_dev_attr_beam3_fifo_rd.dev_attr.attr,
	&iio_dev_attr_beam0_fifo_wr.dev_attr.attr,
	&iio_dev_attr_beam1_fifo_wr.dev_attr.attr,
	&iio_dev_attr_beam2_fifo_wr.dev_attr.attr,
	&iio_dev_attr_beam3_fifo_wr.dev_attr.attr,

	&iio_dev_attr_update_intf_ctrl.dev_attr.attr,
	&iio_dev_attr_update_intf_ctrl_available.dev_attr.attr,

	&iio_dev_attr_amp_bias_reset_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_bias_reset_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_bias_reset_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_bias_reset_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_bias_mute_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_bias_mute_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_bias_mute_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_bias_mute_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL0H.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL1H.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL2H.dev_attr.attr,
	&iio_dev_attr_amp_bias_operational_EL3H.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL0H.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL1H.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL2H.dev_attr.attr,
	&iio_dev_attr_amp_bias_sleep_EL3H.dev_attr.attr,

	&iio_dev_attr_amp_en_reset_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_en_reset_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_en_reset_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_en_reset_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_en_mute_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_en_mute_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_en_mute_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_en_mute_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL0V.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL1V.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL2V.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL3V.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL0H.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL1H.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL2H.dev_attr.attr,
	&iio_dev_attr_amp_en_operational_EL3H.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL0H.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL1H.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL2H.dev_attr.attr,
	&iio_dev_attr_amp_en_sleep_EL3H.dev_attr.attr,
	NULL,
};

static const struct attribute_group adar3000_attribute_group = {
	.attrs = adar3000_attributes,
};

#define DECLARE_ADAR3000_CHANNELS(name)					\
static const struct iio_chan_spec name[] = {				\
	ADAR300x_DELAY_CH(0, 0, "BEAM0_H_EL0_DELAY"),			\
	ADAR300x_ATTEN_CH(1, 0, "BEAM0_H_EL0_ATTENUATION"),		\
	ADAR300x_DELAY_CH(2, 1, "BEAM0_H_EL1_DELAY"),			\
	ADAR300x_ATTEN_CH(3, 1, "BEAM0_H_EL1_ATTENUATION"),		\
	ADAR300x_DELAY_CH(4, 2, "BEAM0_H_EL2_DELAY"),			\
	ADAR300x_ATTEN_CH(5, 2, "BEAM0_H_EL2_ATTENUATION"),		\
	ADAR300x_DELAY_CH(6, 3, "BEAM0_H_EL3_DELAY"),			\
	ADAR300x_ATTEN_CH(7, 3, "BEAM0_H_EL3_ATTENUATION"),		\
									\
	ADAR300x_DELAY_CH(8, 4, "BEAM0_V_EL0_DELAY"),			\
	ADAR300x_ATTEN_CH(9, 4, "BEAM0_V_EL0_ATTENUATION"),		\
	ADAR300x_DELAY_CH(10, 5, "BEAM0_V_EL1_DELAY"),			\
	ADAR300x_ATTEN_CH(11, 5, "BEAM0_V_EL1_ATTENUATION"),		\
	ADAR300x_DELAY_CH(12, 6, "BEAM0_V_EL2_DELAY"),			\
	ADAR300x_ATTEN_CH(13, 6, "BEAM0_V_EL2_ATTENUATION"),		\
	ADAR300x_DELAY_CH(14, 7, "BEAM0_V_EL3_DELAY"),			\
	ADAR300x_ATTEN_CH(15, 7, "BEAM0_V_EL3_ATTENUATION"),		\
									\
	ADAR300x_DELAY_CH(16, 8, "BEAM1_V_EL0_DELAY"),			\
	ADAR300x_ATTEN_CH(17, 8, "BEAM1_V_EL0_ATTENUATION"),		\
	ADAR300x_DELAY_CH(18, 9, "BEAM1_V_EL1_DELAY"),			\
	ADAR300x_ATTEN_CH(19, 9, "BEAM1_V_EL1_ATTENUATION"),		\
	ADAR300x_DELAY_CH(20, 10, "BEAM1_V_EL2_DELAY"),			\
	ADAR300x_ATTEN_CH(21, 10, "BEAM1_V_EL2_ATTENUATION"),		\
	ADAR300x_DELAY_CH(22, 11, "BEAM1_V_EL3_DELAY"),			\
	ADAR300x_ATTEN_CH(23, 11, "BEAM1_V_EL3_ATTENUATION"),		\
									\
	ADAR300x_DELAY_CH(24, 12, "BEAM1_H_EL0_DELAY"),			\
	ADAR300x_ATTEN_CH(25, 12, "BEAM1_H_EL0_ATTENUATION"),		\
	ADAR300x_DELAY_CH(26, 13, "BEAM1_H_EL1_DELAY"),			\
	ADAR300x_ATTEN_CH(27, 13, "BEAM1_H_EL1_ATTENUATION"),		\
	ADAR300x_DELAY_CH(28, 14, "BEAM1_H_EL2_DELAY"),			\
	ADAR300x_ATTEN_CH(29, 14, "BEAM1_H_EL2_ATTENUATION"),		\
	ADAR300x_DELAY_CH(30, 15, "BEAM1_H_EL3_DELAY"),			\
	ADAR300x_ATTEN_CH(31, 15, "BEAM1_H_EL3_ATTENUATION"),		\
									\
	ADAR300x_TEMP(32, 16, TEMP),					\
}

DECLARE_ADAR3000_CHANNELS(adar3000_channels);
DECLARE_ADAR3000_CHANNELS(adar3001_channels);
DECLARE_ADAR3000_CHANNELS(adar3002_channels);

static const struct adar300x_chip_info adar3000_chip_info_tbl[] = {
	[ID_ADAR3000] = {
		.chip_id = ID_ADAR3000,
		.channels = adar3000_channels,
		.num_channels = ARRAY_SIZE(adar3000_channels),
		.unpacked_beamst_len = 8,
		.packed_beamst_len = 6,
		.product_id = ADAR3000_PRODUCT_ID,
	},
	[ID_ADAR3001] = {
		.chip_id = ID_ADAR3001,
		.channels = adar3001_channels,
		.num_channels = ARRAY_SIZE(adar3002_channels),
		.unpacked_beamst_len = 8,
		.packed_beamst_len = 6,
		.product_id = ADAR3000_PRODUCT_ID,
	},
	[ID_ADAR3002] = {
		.chip_id = ID_ADAR3002,
		.channels = adar3002_channels,
		.num_channels = ARRAY_SIZE(adar3002_channels),
		.unpacked_beamst_len = 8,
		.packed_beamst_len = 6,
		.product_id = ADAR3000_PRODUCT_ID,
	},
};

int adar3000_probe(struct spi_device *spi)
{
	return adar300x_probe(spi, &adar3000_attribute_group);
}

static const struct of_device_id adar3000_of_match[] = {
	{ .compatible = "adi,adar3000",
		.data = &adar3000_chip_info_tbl[ID_ADAR3000], },
	{ .compatible = "adi,adar3001",
		.data = &adar3000_chip_info_tbl[ID_ADAR3001], },
	{ .compatible = "adi,adar3002",
		.data = &adar3000_chip_info_tbl[ID_ADAR3002], },
	{ }
};

MODULE_DEVICE_TABLE(of, adar3000_of_match);

static struct spi_driver adar3000_driver = {
	.driver = {
		.name	= "adar3000",
		.of_match_table = adar3000_of_match,
	},
	.probe = adar3000_probe,
};

module_spi_driver(adar3000_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAR3000 Beamformer");
MODULE_LICENSE("Dual BSD/GPL");
