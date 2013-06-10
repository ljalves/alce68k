/*
 * Microchip MCP3208 - ADC driver
 *
 * Author: Luis Alves <ljalvs@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*#include <linux/delay.h>*/
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "asm/alce68k.h"

enum mcp3208_adc_id {
	ADC_CH0,
	ADC_CH1,
	ADC_CH2,
	ADC_CH3,
	ADC_CH4,
	ADC_CH5,
	ADC_CH6,
	ADC_CH7,
};


struct mcp3208_adc {
	/*struct lp8788 *lp;*/
	struct iio_map *map;
	struct mutex lock;
};


static int mcp3208_get_adc_result(struct mcp3208_adc *adc, enum mcp3208_adc_id id,
				int *val)
{
	unsigned int msb;
	unsigned int lsb;
	unsigned int result;
	/*u8 data;*/
	u8 rawdata[2];
	/*int size = ARRAY_SIZE(rawdata);
	int retry = 5;
	int ret;*/

	/* SPI EN + CLOCK DIVISOR */
	ADC_CTRL = SPICTRL_EN | 0x03;
	
	/* SPI CS */
	ADC_CTRL |= SPICTRL_CS;
	

	/* send STARTBIT + SINGLE ENDED + id_bit3 */
	ADC_DATA = 0x06 | (id >> 2);
	/* wait for not busy */
	while (ADC_CTRL & SPICTRL_BUSY);
	
	/* read first byte */
	ADC_DATA = (id << 6);
	/* wait for not busy */
	while (ADC_CTRL & SPICTRL_BUSY);
	rawdata[1] = ADC_DATA;
	
	/* read second byte */
	ADC_DATA = 0x00;
	/* wait for not busy */
	while (ADC_CTRL & SPICTRL_BUSY);
	rawdata[0] = ADC_DATA;

	
	ADC_CTRL = 0x00;

	/*data = (id << 1) | ADC_CONV_START;
	ret = mcp3208_write_byte(adc->lp, LP8788_ADC_CONF, data);
	if (ret)
		goto err_io;*/

	/* retry until adc conversion is done */
	/*data = 0;
	while (retry--) {
		usleep_range(100, 200);

		ret = mcp3208_read_byte(adc->lp, LP8788_ADC_DONE, &data);
		if (ret)
			goto err_io;

		if (data)
			break;
	}

	ret = mcp3208_read_multi_bytes(adc->lp, LP8788_ADC_RAW, rawdata, size);
	if (ret)
		goto err_io;
        */
	msb = (rawdata[1] << 8) & 0x00000f00;
	lsb = rawdata[0] & 0x000000ff;
	result = msb | lsb;
	*val = result;

	return 0;

/*err_io:
	return ret;*/
}


static int mcp3208_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct mcp3208_adc *adc = iio_priv(indio_dev);
	enum mcp3208_adc_id id = chan->channel;
	int ret;

	mutex_lock(&adc->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = mcp3208_get_adc_result(adc, id, val) ? -EIO : IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;/*mcp3208_scale[id] / 1000000;*/
		*val2 = 0;/*mcp3208_scale[id] % 1000000;*/
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&adc->lock);

	return ret;
}


static const struct iio_info mcp3208_adc_info = {
	.read_raw = &mcp3208_adc_read_raw,
	.driver_module = THIS_MODULE,
};


#define MCP3208_CHAN(_id, _type) {				\
		.type = _type,					\
		.indexed = 1,					\
		.channel = ADC_##_id,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
			BIT(IIO_CHAN_INFO_SCALE),	\
		.datasheet_name = #_id,				\
}


static const struct iio_chan_spec mcp3208_adc_channels[] = {
	[ADC_CH0] = MCP3208_CHAN(CH0, IIO_VOLTAGE),
	[ADC_CH1] = MCP3208_CHAN(CH1, IIO_VOLTAGE),
	[ADC_CH2] = MCP3208_CHAN(CH2, IIO_VOLTAGE),
	[ADC_CH3] = MCP3208_CHAN(CH3, IIO_VOLTAGE),
	[ADC_CH4] = MCP3208_CHAN(CH4, IIO_VOLTAGE),
	[ADC_CH5] = MCP3208_CHAN(CH5, IIO_VOLTAGE),
	[ADC_CH6] = MCP3208_CHAN(CH6, IIO_VOLTAGE),
	[ADC_CH7] = MCP3208_CHAN(CH7, IIO_VOLTAGE),
};

/* default maps used by iio consumer (lp8788-charger driver) */
static struct iio_map mcp3208_default_iio_maps[] = {
	/*{
		.consumer_dev_name = "lp8788-charger",
		.consumer_channel = "lp8788_vbatt_5p0",
		.adc_channel_label = "ADC_CH0",
	},
	{
		.consumer_dev_name = "lp8788-charger",
		.consumer_channel = "lp8788_adc1",
		.adc_channel_label = "ADC_CH1",
	},*/
	{ }
};



static int mcp3208_iio_map_register(struct iio_dev *indio_dev,
				struct alce_platform_adc *pdata,
				struct mcp3208_adc *adc)
{
	struct iio_map *map;
	int ret;

	/*map = (!pdata || !pdata->adc_pdata) ?
		mcp3208_default_iio_maps : pdata->adc_pdata;*/
	
	map = mcp3208_default_iio_maps; /*pdata->adc_pdata;*/

	ret = iio_map_array_register(indio_dev, map);
	if (ret) {
		/*dev_err(pdata->dev, "iio map err: %d\n", ret);*/
		return ret;
	}

	adc->map = map;
	return 0;
}

static inline void mcp3208_iio_map_unregister(struct iio_dev *indio_dev,
				struct mcp3208_adc *adc)
{
	iio_map_array_unregister(indio_dev);
}


static int mcp3208_adc_probe(struct platform_device *pdev)
{
	/*struct mcp3208 *lp = dev_get_drvdata(pdev->dev.parent);*/
	/*struct alce_platform_adc *platp = pdev->dev.platform_data;*/
	struct iio_dev *indio_dev;
	struct mcp3208_adc *adc;
	int ret;

	indio_dev = iio_device_alloc(sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	/*adc->lp = lp;*/
	platform_set_drvdata(pdev, indio_dev);

	/*ret = mcp3208_iio_map_register(indio_dev, platp, adc);
	if (ret)
		goto err_iio_map;*/

	mutex_init(&adc->lock);

	/*indio_dev->dev.parent = lp->dev;*/
	indio_dev->dev.parent = &pdev->dev;
	
	indio_dev->name = pdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mcp3208_adc_info;
	indio_dev->channels = mcp3208_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(mcp3208_adc_channels);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "iio dev register err: %d\n", ret);
		goto err_iio_device;
	}

	return 0;

err_iio_device:
	mcp3208_iio_map_unregister(indio_dev, adc);
err_iio_map:
	iio_device_free(indio_dev);
	return ret;
}


static int mcp3208_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	/*struct adc3208_adc *adc = iio_priv(indio_dev);*/

	iio_device_unregister(indio_dev);
	
	/*mcp3208_iio_map_unregister(indio_dev, adc);*/
	iio_device_free(indio_dev);

	return 0;
}


static struct platform_driver mcp3208_adc_driver = {
	.probe = mcp3208_adc_probe,
	.remove = mcp3208_adc_remove,
	.driver = {
		.name = "mcp3208-adc",
		.owner = THIS_MODULE,
	},
};
module_platform_driver(mcp3208_adc_driver);

MODULE_DESCRIPTION("Microchip MCP3208 ADC Driver");
MODULE_AUTHOR("Luis Alves");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mcp3208-adc");
