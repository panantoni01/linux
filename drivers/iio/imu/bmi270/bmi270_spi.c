// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>

#include "bmi270.h"

static int bmi270_spi_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	struct device *dev = &spi->dev;

	regmap = devm_regmap_init_spi(spi, &bmi270_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to init spi regmap");

	return bmi270_core_probe(dev, regmap);
}

static const struct spi_device_id bmi270_spi_id[] = {
	{ "bmi270", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, bmi270_spi_id);

static const struct of_device_id bmi270_of_match[] = {
	{ .compatible = "bosch,bmi270" },
	{ }
};
MODULE_DEVICE_TABLE(of, bmi270_of_match);

static struct spi_driver bmi270_spi_driver = {
	.driver = {
		.name = "bmi270_spi",
		.of_match_table = bmi270_of_match,
	},
	.probe = bmi270_spi_probe,
	.id_table = bmi270_spi_id,
};
module_spi_driver(bmi270_spi_driver);

MODULE_AUTHOR("Antoni Pokusinski <apokusinski01@gmail.com>");
MODULE_DESCRIPTION("BMI270 SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BMI270);
