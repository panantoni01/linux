// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Labs Si7210 Hall Effect sensor driver
 *
 * Copyright (c) 2024 Antoni Pokusinski <apokusinski@o2.pl>
 *
 * Datasheet:
 *  https://www.silabs.com/documents/public/data-sheets/si7210-datasheet.pdf
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/math64.h>
#include <linux/mutex.h>

#define SI7210_REG_DSPSIGM	0xC1
#define SI7210_REG_DSPSIGL	0xC2

#define SI7210_MASK_DSPSIGSEL	GENMASK(2, 0)
#define SI7210_REG_DSPSIGSEL	0xC3

#define SI7210_MASK_STOP	BIT(1)
#define SI7210_MASK_ONEBURST	BIT(2)
#define SI7210_REG_POWER_CTRL	0xC4

#define SI7210_MASK_ARAUTOINC	BIT(0)
#define SI7210_REG_ARAUTOINC	0xC5

#define SI7210_REG_OTP_ADDR	0xE1
#define SI7210_REG_OTP_DATA	0xE2

#define SI7210_MASK_OTP_READ_EN	BIT(1)
#define SI7210_REG_OTP_CTRL	0xE3

#define SI7210_OTPREG_TMP_OFF	0x1D
#define SI7210_OTPREG_TMP_GAIN	0x1E

static const struct regmap_range si7210_read_reg_ranges[] = {
	regmap_reg_range(SI7210_REG_DSPSIGM, SI7210_REG_ARAUTOINC),
	regmap_reg_range(SI7210_REG_OTP_ADDR, SI7210_REG_OTP_CTRL),
};

static const struct regmap_access_table si7210_readable_regs = {
	.yes_ranges = si7210_read_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(si7210_read_reg_ranges),
};

static const struct regmap_range si7210_write_reg_ranges[] = {
	regmap_reg_range(SI7210_REG_DSPSIGSEL, SI7210_REG_ARAUTOINC),
	regmap_reg_range(SI7210_REG_OTP_ADDR, SI7210_REG_OTP_CTRL),
};

static const struct regmap_access_table si7210_writeable_regs = {
	.yes_ranges = si7210_write_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(si7210_write_reg_ranges),
};

static const struct regmap_range si7210_volatile_reg_ranges[] = {
	regmap_reg_range(SI7210_REG_DSPSIGM, SI7210_REG_ARAUTOINC),
	regmap_reg_range(SI7210_REG_OTP_ADDR, SI7210_REG_OTP_CTRL),
};

static const struct regmap_access_table si7210_volatile_regs = {
	.yes_ranges = si7210_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(si7210_volatile_reg_ranges),
};

static const struct regmap_config si7210_regmap_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = SI7210_REG_OTP_CTRL,

	.rd_table = &si7210_readable_regs,
	.wr_table = &si7210_writeable_regs,
	.volatile_table = &si7210_volatile_regs,
};

struct si7210_data {
	struct i2c_client* client;
	struct regmap *regmap;
	u8 temp_offset;
	u8 temp_gain;
	struct mutex fetch_lock;
	struct mutex otp_lock;
};

static const struct iio_chan_spec si7210_channels[] = {
	{
		.type = IIO_MAGN,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET)
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)
	}
};

static int si7210_fetch_measurement(struct si7210_data* data,
				    struct iio_chan_spec const *chan,
				    void* buf)
{
	u8 dspsigsel = chan->type == IIO_MAGN ? 0 : 1;
	int ret;

	mutex_lock(&data->fetch_lock);

	ret = regmap_update_bits(data->regmap, SI7210_REG_DSPSIGSEL,
				SI7210_MASK_DSPSIGSEL, dspsigsel);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, SI7210_REG_ARAUTOINC,
				SI7210_MASK_ARAUTOINC, SI7210_MASK_ARAUTOINC);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, SI7210_REG_POWER_CTRL,
				SI7210_MASK_ONEBURST | SI7210_MASK_STOP,
				SI7210_MASK_ONEBURST & ~SI7210_MASK_STOP);
	if (ret < 0)
		return ret;

	/* Read the contents of two registers containing the result: DSPSIGM and DSPSIGL */
	ret = regmap_bulk_read(data->regmap, SI7210_REG_DSPSIGM, buf, 2);
	if (ret < 0)
		return ret;

	mutex_unlock(&data->fetch_lock);

	return 0;
}

static int si7210_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct si7210_data *data = iio_priv(indio_dev);
	long long tmp;
	u8 dspsig[2];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = si7210_fetch_measurement(data, chan, dspsig);
		if (ret < 0)
			return ret;

		*val = 256 * (dspsig[0] & 0x7F) + dspsig[1];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* TODO: for Si7210-B-05 and Si7210-B-15 the scale is 200mT */
		*val = 0;
		*val2 = 1250;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		*val = 16384;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		ret = si7210_fetch_measurement(data, chan, dspsig);
		if (ret < 0)
			return ret;

		/* value = 32 * dspsigm[6:0] + (dspsigl[7:0] >> 3) */
		tmp = 32 * (dspsig[0] & 0x7F) + (dspsig[1] >> 3);
		/* temp_raw = -3.83 * 1e-6 * value^2 + 0.16094 * value - 279.8 */
		tmp = ((-383 * tmp * tmp) / 100 + (160940 * tmp - 279800000));
		/* temperature = (1 + gain / 2048) * temp_raw + offset / 16 */
		tmp = (1 + data->temp_gain / 2048) * tmp + 62500 * data->temp_offset;
		/* temperature -= 0.222 * VDD, if VDD is unknown, then use VDD = 3.3 V" */
		tmp -= 732600;

		*val = div_s64_rem(tmp, 1000000, val2);

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int si7210_read_otpreg_val(struct si7210_data *data, unsigned int otpreg, u8* val)
{
	int ret;

	mutex_lock(&data->otp_lock);

	ret = regmap_write(data->regmap, SI7210_REG_OTP_ADDR, otpreg);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap,  SI7210_REG_OTP_CTRL, SI7210_MASK_OTP_READ_EN,
							SI7210_MASK_OTP_READ_EN);
	if (ret < 0)
		return ret;

	ret = regmap_read(data->regmap, SI7210_REG_OTP_DATA, (unsigned int *)val);
	if (ret < 0)
		return ret;

	mutex_unlock(&data->otp_lock);

	return 0;
}

static int si7210_device_wake(struct si7210_data *data)
{
	/* According to the datasheet, the primary method to wake up a device is
	to send an empty write. However this is not feasible using current API so we
	use the other method i.e. read a single byte. The device should respond with 0xFF */

	int ret = 0;

	ret = i2c_smbus_read_byte(data->client);
	if (ret < 0)
		return ret;

	if ((u8)ret != 0xFF)
		return -EIO;

	return 0;
}

static int si7210_device_init(struct si7210_data *data)
{
	int ret;

	ret = si7210_read_otpreg_val(data, SI7210_OTPREG_TMP_GAIN, &data->temp_gain);
	if (ret < 0)
		return 0;
	ret = si7210_read_otpreg_val(data, SI7210_OTPREG_TMP_OFF, &data->temp_offset);
	if (ret < 0)
		return 0;

	si7210_device_wake(data);

	msleep(1);

	return 0;
}

static const struct iio_info si7210_info = {
	.read_raw = si7210_read_raw,
};

static int si7210_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct si7210_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	mutex_init(&data->fetch_lock);
	mutex_init(&data->otp_lock);

	data->regmap = devm_regmap_init_i2c(client, &si7210_regmap_conf);
	if (IS_ERR(data->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(data->regmap),
				"failed to register regmap\n");

	indio_dev->name = dev_name(&client->dev);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &si7210_info;
	indio_dev->channels = si7210_channels;
	indio_dev->num_channels = ARRAY_SIZE(si7210_channels);

	ret = si7210_device_init(data);
	if (ret)
		return dev_err_probe(&client->dev, ret, "device initialization failed\n");

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id si7210_id[] = {
	{ "si7210", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si7210_id);

static const struct of_device_id si7210_dt_ids[] = {
	{ .compatible = "silabs,si7210" },
	{ }
};
MODULE_DEVICE_TABLE(of, si7210_dt_ids);

static struct i2c_driver si7210_driver = {
	.driver = {
		.name = "si7210",
		.of_match_table = si7210_dt_ids,
	},
	.probe		= si7210_probe,
	.id_table	= si7210_id,
};

module_i2c_driver(si7210_driver);
MODULE_AUTHOR("Antoni Pokusinski <apokusinski@o2.pl>");
MODULE_DESCRIPTION("Silicon Labs Si7210 Hall Effect sensor I2C driver");
MODULE_LICENSE("GPL v2");
