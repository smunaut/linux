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

#define ADAR3003_PRODUCT_ID 0x00

enum adar3003_iio_dev_attr {
	ADAR3003_EL0VH,
	ADAR3003_EL1VH,
	ADAR3003_EL2VH,
	ADAR3003_EL3VH,
	ADAR3003_ELEM_NO,
};

static IIO_DEVICE_ATTR(el0vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL0VH);

static IIO_DEVICE_ATTR(el1vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL1VH);

static IIO_DEVICE_ATTR(el2vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL2VH);

static IIO_DEVICE_ATTR(el3vh_update, 0644,
		       adar300x_update_show, adar300x_update_store, ADAR3003_EL3VH);

static IIO_DEVICE_ATTR(update_intf_ctrl_available, 0444,
		       adar300x_show_update_intf_ctrl_available, NULL, 0);

static IIO_DEVICE_ATTR(update_intf_ctrl, 0644,
		       adar300x_update_intf_ctrl_show, adar300x_update_intf_ctrl_store, 0);

static IIO_DEVICE_ATTR(el0vh_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3003_EL0VH);

static IIO_DEVICE_ATTR(el1vh_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3003_EL1VH);

static IIO_DEVICE_ATTR(el2vh_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3003_EL2VH);

static IIO_DEVICE_ATTR(el3vh_mode, 0644,
		       adar300x_mode_show, adar300x_mode_store, ADAR3003_EL3VH);

static IIO_DEVICE_ATTR(el0vh_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el1vh_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el2vh_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el3vh_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el0vh_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3003_EL0VH);

static IIO_DEVICE_ATTR(el1vh_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3003_EL1VH);

static IIO_DEVICE_ATTR(el2vh_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3003_EL2VH);

static IIO_DEVICE_ATTR(el3vh_load_mode, 0644,
		       adar300x_load_mode_show, adar300x_load_mode_store, ADAR3003_EL3VH);

static IIO_DEVICE_ATTR(el0vh_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el1vh_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el2vh_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el3vh_load_mode_available, 0444, adar300x_show_mode_available, NULL, 0);

static IIO_DEVICE_ATTR(el0vh_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR0_RAM_START);

static IIO_DEVICE_ATTR(el1vh_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR1_RAM_START);

static IIO_DEVICE_ATTR(el2vh_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR2_RAM_START);

static IIO_DEVICE_ATTR(el3vh_ram_start, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR3_RAM_START);

static IIO_DEVICE_ATTR(el0vh_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR0_RAM_STOP);

static IIO_DEVICE_ATTR(el1vh_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR1_RAM_STOP);

static IIO_DEVICE_ATTR(el2vh_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR2_RAM_STOP);

static IIO_DEVICE_ATTR(el3vh_ram_stop, 0644,
		       adar300x_ram_range_show, adar300x_ram_range_store, ADAR300x_PTR3_RAM_STOP);

static IIO_DEVICE_ATTR(el0vh_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX0);

static IIO_DEVICE_ATTR(el1vh_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX1);

static IIO_DEVICE_ATTR(el2vh_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX2);

static IIO_DEVICE_ATTR(el3vh_ram_index, 0644,
		       adar300x_ram_index_show, adar300x_ram_index_store, ADAR300x_RAM_INDEX3);

static IIO_DEVICE_ATTR(el0vh_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD0);

static IIO_DEVICE_ATTR(el1vh_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD1);

static IIO_DEVICE_ATTR(el2vh_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD2);

static IIO_DEVICE_ATTR(el3vh_fifo_rd, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_RD3);

static IIO_DEVICE_ATTR(el0vh_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR0);

static IIO_DEVICE_ATTR(el1vh_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR1);

static IIO_DEVICE_ATTR(el2vh_fifo_wr, 0644,
		       adar300x_fifo_ptr_show, NULL, ADAR300x_FIFO_WR2);

static IIO_DEVICE_ATTR(el3vh_fifo_wr, 0644,
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

static struct attribute *adar3003_attributes[] = {
	&iio_dev_attr_el0vh_update.dev_attr.attr,
	&iio_dev_attr_el1vh_update.dev_attr.attr,
	&iio_dev_attr_el2vh_update.dev_attr.attr,
	&iio_dev_attr_el3vh_update.dev_attr.attr,

	&iio_dev_attr_update_intf_ctrl.dev_attr.attr,
	&iio_dev_attr_update_intf_ctrl_available.dev_attr.attr,

	&iio_dev_attr_el0vh_mode.dev_attr.attr,
	&iio_dev_attr_el1vh_mode.dev_attr.attr,
	&iio_dev_attr_el2vh_mode.dev_attr.attr,
	&iio_dev_attr_el3vh_mode.dev_attr.attr,

	&iio_dev_attr_el0vh_mode_available.dev_attr.attr,
	&iio_dev_attr_el1vh_mode_available.dev_attr.attr,
	&iio_dev_attr_el2vh_mode_available.dev_attr.attr,
	&iio_dev_attr_el3vh_mode_available.dev_attr.attr,

	&iio_dev_attr_el0vh_load_mode.dev_attr.attr,
	&iio_dev_attr_el1vh_load_mode.dev_attr.attr,
	&iio_dev_attr_el2vh_load_mode.dev_attr.attr,
	&iio_dev_attr_el3vh_load_mode.dev_attr.attr,

	&iio_dev_attr_el0vh_load_mode_available.dev_attr.attr,
	&iio_dev_attr_el1vh_load_mode_available.dev_attr.attr,
	&iio_dev_attr_el2vh_load_mode_available.dev_attr.attr,
	&iio_dev_attr_el3vh_load_mode_available.dev_attr.attr,

	&iio_dev_attr_el0vh_ram_start.dev_attr.attr,
	&iio_dev_attr_el1vh_ram_start.dev_attr.attr,
	&iio_dev_attr_el2vh_ram_start.dev_attr.attr,
	&iio_dev_attr_el3vh_ram_start.dev_attr.attr,

	&iio_dev_attr_el0vh_ram_stop.dev_attr.attr,
	&iio_dev_attr_el1vh_ram_stop.dev_attr.attr,
	&iio_dev_attr_el2vh_ram_stop.dev_attr.attr,
	&iio_dev_attr_el3vh_ram_stop.dev_attr.attr,

	&iio_dev_attr_el0vh_ram_index.dev_attr.attr,
	&iio_dev_attr_el1vh_ram_index.dev_attr.attr,
	&iio_dev_attr_el2vh_ram_index.dev_attr.attr,
	&iio_dev_attr_el3vh_ram_index.dev_attr.attr,

	&iio_dev_attr_el0vh_fifo_rd.dev_attr.attr,
	&iio_dev_attr_el1vh_fifo_rd.dev_attr.attr,
	&iio_dev_attr_el2vh_fifo_rd.dev_attr.attr,
	&iio_dev_attr_el3vh_fifo_rd.dev_attr.attr,

	&iio_dev_attr_el0vh_fifo_wr.dev_attr.attr,
	&iio_dev_attr_el1vh_fifo_wr.dev_attr.attr,
	&iio_dev_attr_el2vh_fifo_wr.dev_attr.attr,
	&iio_dev_attr_el3vh_fifo_wr.dev_attr.attr,

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

static const struct attribute_group adar3003_attribute_group = {
	.attrs = adar3003_attributes,
};

#define DECLARE_ADAR3003_CHANNELS(name)				\
static const struct iio_chan_spec name[] = {			\
	ADAR300x_DELAY_CH(0, 0, "EL0V_DELAY"),			\
	ADAR300x_ATTEN_CH(1, 0, "EL0V_ATTENUATION"),		\
	ADAR300x_DELAY_CH(2, 1, "EL0H_DELAY"),			\
	ADAR300x_ATTEN_CH(3, 1, "EL0H_ATTENUATION"),		\
								\
	ADAR300x_DELAY_CH(4, 2, "EL1V_DELAY"),			\
	ADAR300x_ATTEN_CH(5, 2, "EL1V_ATTENUATION"),		\
	ADAR300x_DELAY_CH(6, 3, "EL1H_DELAY"),			\
	ADAR300x_ATTEN_CH(7, 3, "EL1H_ATTENUATION"),		\
								\
	ADAR300x_DELAY_CH(8, 4, "EL2V_DELAY"),			\
	ADAR300x_ATTEN_CH(9, 4, "EL2V_ATTENUATION"),		\
	ADAR300x_DELAY_CH(10, 5, "EL2H_DELAY"),			\
	ADAR300x_ATTEN_CH(11, 5, "EL2H_ATTENUATION"),		\
								\
	ADAR300x_DELAY_CH(12, 6, "EL3V_DELAY"),			\
	ADAR300x_ATTEN_CH(13, 6, "EL3V_ATTENUATION"),		\
	ADAR300x_DELAY_CH(14, 7, "EL3H_DELAY"),			\
	ADAR300x_ATTEN_CH(15, 7, "EL3H_ATTENUATION"),		\
								\
	ADAR300x_TEMP(32, 16, TEMP),				\
}

DECLARE_ADAR3003_CHANNELS(adar3003_channels);

static const struct adar300x_chip_info adar3003_chip_info_tbl[] = {
	[ID_ADAR3003] = {
		.chip_id = ID_ADAR3003,
		.channels = adar3003_channels,
		.num_channels = ARRAY_SIZE(adar3003_channels),
		.unpacked_beamst_len = 4,
		.packed_beamst_len = 3,
		.product_id = ADAR3003_PRODUCT_ID,
	},
};

static const struct of_device_id adar3003_of_match[] = {
	{ .compatible = "adi,adar3003",
		.data = &adar3003_chip_info_tbl[ID_ADAR3003], },
	{ }
};

MODULE_DEVICE_TABLE(of, adar3000_of_match);

int adar3003_probe(struct spi_device *spi)
{
	return adar300x_probe(spi, &adar3003_attribute_group);
}

static struct spi_driver adar3003_driver = {
	.driver = {
		.name	= "adar3003",
		.of_match_table = adar3003_of_match,
	},
	.probe = adar3003_probe,
};

module_spi_driver(adar3003_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAR3000 Beamformer");
MODULE_LICENSE("Dual BSD/GPL");
