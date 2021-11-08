// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 */

#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

#include "adxl367.h"

struct adxl367_spi_state {
	struct spi_device	*spi;

	struct spi_message	reg_write_msg;
	struct spi_transfer	reg_write_xfer[1];

	struct spi_message	reg_read_msg;
	struct spi_transfer	reg_read_xfer[2];

	struct spi_message	fifo_msg;
	struct spi_transfer	fifo_xfer[2];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8			reg_write_buf[3]    ____cacheline_aligned;
	u8			reg_read_buf[3]     ____cacheline_aligned;
	u8			fifo_command_buf[1] ____cacheline_aligned;
};

static int adxl367_spi_read_fifo(void *context, __be16 *fifo_buf,
				 unsigned int fifo_entries)
{
	struct adxl367_spi_state *st = context;

	st->fifo_xfer[1].rx_buf = fifo_buf;
	st->fifo_xfer[1].len = fifo_entries * sizeof(*fifo_buf);

	return spi_sync(st->spi, &st->fifo_msg);
}

static int adxl367_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct adxl367_spi_state *st = context;
	int ret;

	st->reg_read_buf[1] = reg;

	ret = spi_sync(st->spi, &st->reg_read_msg);
	if (ret)
		return ret;

	*val = st->reg_read_buf[2];

	return 0;
}

static int adxl367_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct adxl367_spi_state *st = context;

	st->reg_write_buf[1] = reg;
	st->reg_write_buf[2] = val;

	return spi_sync(st->spi, &st->reg_write_msg);
}

static const struct regmap_config adxl367_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.reg_read = adxl367_reg_read,
	.reg_write = adxl367_reg_write,
};

static const struct adxl367_ops adxl367_spi_ops = {
	.read_fifo = adxl367_spi_read_fifo,
};

static int adxl367_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct adxl367_spi_state *st;
	struct regmap *regmap;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->spi = spi;

	st->reg_write_buf[0] = 0x0A;
	st->reg_write_xfer[0].tx_buf = &st->reg_write_buf[0];
	st->reg_write_xfer[0].len = 3;
	spi_message_init_with_transfers(&st->reg_write_msg, st->reg_write_xfer, 1);

	st->reg_read_buf[0] = 0x0B;
	st->reg_read_xfer[0].tx_buf = &st->reg_read_buf[0];
	st->reg_read_xfer[0].len = 2;
	st->reg_read_xfer[1].rx_buf = &st->reg_read_buf[2];
	st->reg_read_xfer[1].len = 1;
	spi_message_init_with_transfers(&st->reg_read_msg, st->reg_read_xfer, 2);

	st->fifo_command_buf[0] = 0x0D;
	st->fifo_xfer[0].tx_buf = &st->fifo_command_buf[0];
	st->fifo_xfer[0].len = 1;
	spi_message_init_with_transfers(&st->fifo_msg, st->fifo_xfer, 2);

	regmap = devm_regmap_init(&spi->dev, NULL, st,
				  &adxl367_spi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return adxl367_probe(&spi->dev, &adxl367_spi_ops, st, regmap, spi->irq,
			     id->name);
}

static const struct spi_device_id adxl367_spi_id[] = {
	{ "adxl367", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, adxl367_spi_id);

static const struct of_device_id adxl367_of_match[] = {
	{ .compatible = "adi,adxl367" },
	{ }
};
MODULE_DEVICE_TABLE(of, adxl367_of_match);

static struct spi_driver adxl367_spi_driver = {
	.driver = {
		.name = "adxl367_spi",
		.of_match_table = adxl367_of_match,
	},
	.probe = adxl367_spi_probe,
	.id_table = adxl367_spi_id,
};

module_spi_driver(adxl367_spi_driver);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXL367 3-axis accelerometer SPI driver");
MODULE_LICENSE("GPL");
