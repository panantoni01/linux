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
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/iio/events.h>
#include <linux/log2.h>

#define SI7210_REG_DSPSIGM	0xC1
#define SI7210_REG_DSPSIGL	0xC2
#define SI7210_MASK_DSPSIGSEL	GENMASK(2, 0)
#define SI7210_REG_DSPSIGSEL	0xC3
#define SI7210_MASK_STOP	BIT(1)
#define SI7210_MASK_ONEBURST	BIT(2)
#define SI7210_REG_POWER_CTRL	0xC4
#define SI7210_MASK_ARAUTOINC	BIT(0)
#define SI7210_REG_ARAUTOINC	0xC5
#define SI7210_MASK_LOW4FIELD	BIT(7)
#define SI7210_MASK_OP		GENMASK(6, 0)
#define SI7210_REG_THRESHOLD	0xC6
#define SI7210_MASK_FIELDPOLSEL	GENMASK(7, 6)
#define SI7210_MASK_HYST	GENMASK(5, 0)
#define SI7210_REG_HYSTERESIS	0xC7
#define SI7210_MASK_TAMPER	GENMASK(7, 2)
#define SI7210_REG_TAMPER	0xC9

#define SI7210_REG_OTP_ADDR	0xE1
#define SI7210_REG_OTP_DATA	0xE2
#define SI7210_MASK_OTP_READ_EN	BIT(1)
#define SI7210_REG_OTP_CTRL	0xE3

#define SI7210_OTPREG_TMP_OFF	0x1D
#define SI7210_OTPREG_TMP_GAIN	0x1E

static const struct regmap_range si7210_read_reg_ranges[] = {
	regmap_reg_range(SI7210_REG_DSPSIGM, SI7210_REG_HYSTERESIS),
	regmap_reg_range(SI7210_REG_TAMPER, SI7210_REG_TAMPER),
	regmap_reg_range(SI7210_REG_OTP_ADDR, SI7210_REG_OTP_CTRL),
};

static const struct regmap_access_table si7210_readable_regs = {
	.yes_ranges = si7210_read_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(si7210_read_reg_ranges),
};

static const struct regmap_range si7210_write_reg_ranges[] = {
	regmap_reg_range(SI7210_REG_DSPSIGSEL, SI7210_REG_HYSTERESIS),
	regmap_reg_range(SI7210_REG_TAMPER, SI7210_REG_TAMPER),
	regmap_reg_range(SI7210_REG_OTP_ADDR, SI7210_REG_OTP_CTRL),
};

static const struct regmap_access_table si7210_writeable_regs = {
	.yes_ranges = si7210_write_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(si7210_write_reg_ranges),
};

static const struct regmap_range si7210_volatile_reg_ranges[] = {
	/* TODO: check if dsisigsel, power_ctrl, arautoinc and otps are volatile */
	regmap_reg_range(SI7210_REG_DSPSIGM, SI7210_REG_DSPSIGL),
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

static const struct iio_event_spec si7210_magn_event[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
		BIT(IIO_EV_INFO_HYSTERESIS),
	},
};

static const struct iio_chan_spec si7210_channels[] = {
	{
		.type = IIO_MAGN,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
		.event_spec = si7210_magn_event,
		.num_event_specs = ARRAY_SIZE(si7210_magn_event)
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED)
	}
};

static inline int is_threshold_zero(int threshold) {
	/* From the datasheet: "threshold = 0, when sw_op = 127" */
	return ((threshold & SI7210_MASK_OP) == SI7210_MASK_OP);
}

static inline int is_hysteresis_zero(int hysteresis) {
	/* "When sw_hyst = 63, the hysteresis is set to zero" */
	return ((hysteresis & SI7210_MASK_HYST) == SI7210_MASK_HYST);
}

static int si7210_read_event(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int *val, int *val2)
{
	struct si7210_data *data = iio_priv(indio_dev);
	int ret, raw_thresh, raw_hyst, thresh, hyst;

	ret = regmap_read(data->regmap, SI7210_REG_THRESHOLD, &raw_thresh);
	if (ret < 0)
		return ret;

	if (info == IIO_EV_INFO_VALUE) {
		if (is_threshold_zero(raw_thresh))
			thresh = 0;
		else
			thresh = (16 + (raw_thresh & 0x0F)) *
				 (1 << ((raw_thresh & 0x70) >> 4)) * 5;

		*val = thresh / 1000;
		*val2 = 1000 * (thresh % 1000);
	} else {
		ret = regmap_read(data->regmap, SI7210_REG_HYSTERESIS, &raw_hyst);
		if (ret < 0)
			return ret;

		if (is_hysteresis_zero(raw_hyst))
			hyst = 0;
		else
			hyst = (8 + (raw_hyst & 0x7)) * (1 << ((raw_hyst & 0x38) >> 3)) * 5;

		/* "If sw_op = 127, (...) the hysteresis is multiplied by 2" */
		if (is_threshold_zero(raw_thresh))
			hyst *= 2;

		*val = hyst / 1000;
		*val2 = 1000 * (hyst % 1000);
	}

	return IIO_VAL_INT_PLUS_MICRO;
}

static int si7210_write_event(struct iio_dev *indio_dev,
		const struct iio_chan_spec *chan, enum iio_event_type type,
		enum iio_event_direction dir, enum iio_event_info info,
		int val, int val2)
{
	struct si7210_data *data = iio_priv(indio_dev);
	int ret, raw_thresh;

	if (val < 0)
		return -EINVAL;

	val = (val * 1000 + val2 / 1000) / 5;

	if (info == IIO_EV_INFO_VALUE) {
		if (val < 16)
			/* "threshold = 0, when sw_op = 127" */
			val = SI7210_MASK_OP;
		else {
			val = clamp_val(val, 16, 3840);
			val = (ilog2(val / 16) << 4) | (val / (1 << ilog2(val / 16)) - 16);
		}

		ret = regmap_update_bits(data->regmap, SI7210_REG_THRESHOLD,
					SI7210_MASK_OP, val);
		if (ret < 0)
			return ret;

	} else { /* IIO_EV_INFO_HYSTERESIS */
		ret = regmap_read(data->regmap, SI7210_REG_THRESHOLD, &raw_thresh);
		if (ret < 0)
			return ret;

		if (is_threshold_zero(raw_thresh))
			val /= 2;

		if (val < 8)
			/* "When sw_hyst = 63, the hysteresis is set to zero" */
			val = SI7210_MASK_HYST;
		else {
			val = clamp_val(val, 8, 1792);
			val = (ilog2(val / 8) << 3) | (val / (1 << ilog2(val / 8)) - 8);
		}

		ret = regmap_update_bits(data->regmap, SI7210_REG_HYSTERESIS,
					SI7210_MASK_HYST, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static irqreturn_t si7210_interrupt_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;

	iio_push_event(indio_dev,
			IIO_UNMOD_EVENT_CODE(IIO_MAGN, 0,
				IIO_EV_TYPE_THRESH,
				IIO_EV_DIR_RISING),
			iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static int si7210_fetch_measurement(struct si7210_data* data,
				    struct iio_chan_spec const *chan,
				    void* buf)
{
	u8 dspsigsel;
	int ret;

	if (chan->type == IIO_MAGN)
		dspsigsel = 0;
	else /* IIO_TEMP */
		dspsigsel = 1;

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
		/* TODO: the default scale should be deduced based on the part no. 
		In here we assume the usual scale 20mT */
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

	ret = si7210_device_wake(data);
	if (ret < 0)
		return ret;

	ret = si7210_read_otpreg_val(data, SI7210_OTPREG_TMP_GAIN, &data->temp_gain);
	if (ret < 0)
		return ret;
	ret = si7210_read_otpreg_val(data, SI7210_OTPREG_TMP_OFF, &data->temp_offset);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, SI7210_REG_THRESHOLD,
				SI7210_MASK_LOW4FIELD, ~SI7210_MASK_LOW4FIELD);
	if (ret < 0)
		return ret;

	/* "threshold = 0, when sw_op = 127" */
	ret = regmap_update_bits(data->regmap, SI7210_REG_THRESHOLD,
				SI7210_MASK_OP, SI7210_MASK_OP);
	if (ret < 0)
		return ret;

	/* "When sw_hyst = 63, the hysteresis is set to zero" */
	ret = regmap_update_bits(data->regmap, SI7210_REG_HYSTERESIS,
				SI7210_MASK_HYST, SI7210_MASK_HYST);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(data->regmap, SI7210_REG_HYSTERESIS,
				SI7210_MASK_FIELDPOLSEL, ~SI7210_MASK_FIELDPOLSEL);
	if (ret < 0)
		return ret;

	/* "The tamper feature is disabled if sw_tamper = 63" */
	ret = regmap_update_bits(data->regmap, SI7210_REG_TAMPER,
				SI7210_MASK_TAMPER, SI7210_MASK_TAMPER);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct iio_info si7210_info = {
	.read_raw = si7210_read_raw,
	.read_event_value = si7210_read_event,
	.write_event_value = si7210_write_event,
};

static int si7210_probe(struct i2c_client *client)
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

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, si7210_interrupt_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				indio_dev->name, indio_dev);
		if (ret) {
			dev_err_probe(&client->dev, ret, "irq request error\n");
			return ret;
		}
	}

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
