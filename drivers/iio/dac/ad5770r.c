// SPDX-License-Identifier: GPL-2.0+
/*
 * AD5770R Digital to analog converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/fs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/property.h>
#include <linux/gpio/consumer.h>

/*SPI configuration registers*/
#define AD5770R_INTERFACE_CONFIG_A	0x00
#define AD5770R_INTERFACE_CONFIG_B	0x01
#define AD5770R_CHIP_TYPE		0x03
#define AD5770R_PRODUCT_ID_L		0x04
#define AD5770R_PRODUCT_ID_H		0x05
#define AD5770R_CHIP_GRADE		0x06
#define AD5770R_SCRATCH_PAD		0x0A
#define AD5770R_SPI_REVISION		0x0B
#define AD5770R_VENDOR_L		0x0C
#define AD5770R_VENDOR_H		0x0D
#define AD5770R_STREAM_MODE		0x0E
#define AD5770R_INTERFACE_CONFIG_C	0x10
#define AD5770R_INTERFACE_STATUS_A	0x11

/*AD5770R configuration registers*/
#define AD5770R_CHANNEL_CONFIG		0x14
#define AD5770R_OUTPUT_RANGE_CH0	0x15
#define AD5770R_OUTPUT_RANGE_CH1	0x16
#define AD5770R_OUTPUT_RANGE_CH2	0x17
#define AD5770R_OUTPUT_RANGE_CH3	0x18
#define AD5770R_OUTPUT_RANGE_CH4	0x19
#define AD5770R_OUTPUT_RANGE_CH5	0x1A
#define AD5770R_REFERENCE		0x1B
#define AD5770R_ALARM_CONFIG		0x1C
#define AD5770R_OUTPUT_FILTER_CH0	0x1D
#define AD5770R_OUTPUT_FILTER_CH1	0x1E
#define AD5770R_OUTPUT_FILTER_CH2	0x1F
#define AD5770R_OUTPUT_FILTER_CH3	0x20
#define AD5770R_OUTPUT_FILTER_CH4	0x21
#define AD5770R_OUTPUT_FILTER_CH5	0x22
#define AD5770R_MONITOR_SETUP		0x23
#define AD5770R_STATUS			0x24
#define AD5770R_HW_LDAC			0x25
#define AD5770R_CH0_DAC_LSB		0x26
#define AD5770R_CH0_DAC_MSB		0x27
#define AD5770R_CH1_DAC_LSB		0x28
#define AD5770R_CH1_DAC_MSB		0x29
#define AD5770R_CH2_DAC_LSB		0x2A
#define AD5770R_CH2_DAC_MSB		0x2B
#define AD5770R_CH3_DAC_LSB		0x2C
#define AD5770R_CH3_DAC_MSB		0x2D
#define AD5770R_CH4_DAC_LSB		0x2E
#define AD5770R_CH4_DAC_MSB		0x2F
#define AD5770R_CH5_DAC_LSB		0x30
#define AD5770R_CH5_DAC_MSB		0x31
#define AD5770R_DAC_PAGE_MASK_LSB	0x32
#define AD5770R_DAC_PAGE_MASK_MSB	0x33
#define AD5770R_CH_SELECT		0x34
#define AD5770R_INPUT_PAGE_MASK_LSB	0x35
#define AD5770R_INPUT_PAGE_MASK_MSB	0x36
#define AD5770R_SW_LDAC			0x37
#define AD5770R_CH0_INPUT_LSB		0x38
#define AD5770R_CH0_INPUT_MSB		0x39
#define AD5770R_CH1_INPUT_LSB		0x3A
#define AD5770R_CH1_INPUT_MSB		0x3B
#define AD5770R_CH2_INPUT_LSB		0x3C
#define AD5770R_CH2_INPUT_MSB		0x3D
#define AD5770R_CH3_INPUT_LSB		0x3E
#define AD5770R_CH3_INPUT_MSB		0x3F
#define AD5770R_CH4_INPUT_LSB		0x40
#define AD5770R_CH4_INPUT_MSB		0x41
#define AD5770R_CH5_INPUT_LSB		0x42
#define AD5770R_CH5_INPUT_MSB		0x43
#define AD5770R_CH_ENABLE		0x44

/* AD5770R_INTERFACE_CONFIG_A */
#define AD5770R_INTERFACE_CONFIG_A_SW_RESET_MSK			BIT(7) | BIT(0)
#define AD5770R_INTERFACE_CONFIG_A_SW_RESET(x)			(((x) & 0x1) | 0x80)
#define AD5770R_INTERFACE_CONFIG_A_ADDR_ASCENSION_MSB_MSK	BIT(5)
#define AD5770R_INTERFACE_CONFIG_A_ADDR_ASCENSION_MSB(x)	(((x) & 0x1) << 5)
#define AD5770R_INTERFACE_CONFIG_A_SDO_ACTIVE_MSK		BIT(4) | BIT(3)

/* AD5770R_INTERFACE_CONFIG_B */
#define AD5770R_INTERFACE_CONFIG_B_SINGLE_INST_MSK		BIT(7)
#define AD5770R_INTERFACE_CONFIG_B_SINGLE_INST(x)		(((x) & 0x1) << 7)
#define AD5770R_INTERFACE_CONFIG_B_SHORT_INST_MSK		BIT(3)
#define AD5770R_INTERFACE_CONFIG_B_SHORT_INST(x)		(((x) & 0x1) << 3)

/* AD5770R_INTERFACE_CONFIG_C */
#define AD5770R_INTERFACE_CONFIG_C_STRUCT_REGISTER_ACCESS_MSK	BIT(5)
#define AD5770R_INTERFACE_CONFIG_C_STRUCT_REGISTER_ACCESS(x)	(((x) & 0x1) << 5)

/* AD5770R_CHANNEL_CONFIG */
#define AD5770R_CHANNEL_CONFIG_CH0_SINK_EN_MSK			BIT(7)
#define AD5770R_CHANNEL_CONFIG_CH0_SINK_EN(x)			(((x) & 0x1) << 7)
#define AD5770R_CHANNEL_CONFIG_CH5_SHUTDOWN_B_MSK		BIT(5)
#define AD5770R_CHANNEL_CONFIG_CH5_SHUTDOWN_B(x)		(((x) & 0x1) << 5)
#define AD5770R_CHANNEL_CONFIG_CH4_SHUTDOWN_B_MSK		BIT(4)
#define AD5770R_CHANNEL_CONFIG_CH4_SHUTDOWN_B(x)		(((x) & 0x1) << 4)
#define AD5770R_CHANNEL_CONFIG_CH3_SHUTDOWN_B_MSK		BIT(3)
#define AD5770R_CHANNEL_CONFIG_CH3_SHUTDOWN_B(x)		(((x) & 0x1) << 3)
#define AD5770R_CHANNEL_CONFIG_CH2_SHUTDOWN_B_MSK		BIT(2)
#define AD5770R_CHANNEL_CONFIG_CH2_SHUTDOWN_B(x)		(((x) & 0x1) << 2)
#define AD5770R_CHANNEL_CONFIG_CH1_SHUTDOWN_B_MSK		BIT(1)
#define AD5770R_CHANNEL_CONFIG_CH1_SHUTDOWN_B(x)		(((x) & 0x1) << 1)
#define AD5770R_CHANNEL_CONFIG_CH0_SHUTDOWN_B_MSK		BIT(0)
#define AD5770R_CHANNEL_CONFIG_CH0_SHUTDOWN_B(x)		(((x) & 0x1) << 0)

/* AD5770R_OUTPUT_RANGE */
#define AD5770R_OUTPUT_RANGE_OUTPUT_SCALING_MSK			GENMASK(7, 2)
#define AD5770R_OUTPUT_RANGE_OUTPUT_SCALING(x)			(((x) & 0x3F) << 2)
#define AD5770R_OUTPUT_RANGE_MODE_MSK				GENMASK(1, 0)
#define AD5770R_OUTPUT_RANGE_MODE(x)				((x) & 0x03)

/* AD5770R_REFERENCE */
#define AD5770R_REFERENCE_RESISTOR_SEL_MSK			BIT(2)
#define AD5770R_REFERENCE_RESISTOR_SEL(x)			(((x) & 0x1) << 2)
#define AD5770R_REFERENCE_VOLTATE_SEL_MSK			GENMASK(1, 0)
#define AD5770R_REFERENCE_VOLTATE_SEL(x)			(((x) & 0x3) << 0)

/* AD5770R_ALARM_CONFIG */
#define AD5770R_ALARM_CONFIG_BACKGROUND_CRC_ALARM_MASK(x)	(((x) & 0x1) << 7)
#define AD5770R_ALARM_CONFIG_IREF_FAULT_ALARM_MASK(x)		(((x) & 0x1) << 6)
#define AD5770R_ALARM_CONFIG_NEGATIVE_CHANNEL0_ALARM_MASK(x)	(((x) & 0x1) << 5)
#define AD5770R_ALARM_CONFIG_OVER_TEMP_ALARM_MASK(x)		(((x) & 0x1) << 4)
#define AD5770R_ALARM_CONFIG_TEMP_WARNING_ALARM_MASK(x)		(((x) & 0x1) << 3)
#define AD5770R_ALARM_CONFIG_BACKGROUND_CRC_EN(x)		(((x) & 0x1) << 2)
#define AD5770R_ALARM_CONFIG_THERMAL_SHUTDOWN_EN(x)		(((x) & 0x1) << 1)
#define AD5770R_ALARM_CONFIG_OPEN_DRAIN_EN(x)			(((x) & 0x1) << 0)

/* AD5770R_OUTPUT_FILTER_CH */
#define AD5770R_OUTPUT_FILTER_CH_MSK			GENMASK(3, 0)
#define AD5770R_OUTPUT_FILTER_CH(x)			(((x) & 0xF) << 0)

/* AD5770R_MONITOR_SETUP */
#define AD5770R_MONITOR_SETUP_MON_FUNCTION_MSK		GENMASK(7, 6)
#define AD5770R_MONITOR_SETUP_MON_FUNCTION(x)		(((x) & 0x3) << 6)
#define AD5770R_MONITOR_SETUP_MUX_BUFFER_MSK		BIT(5)
#define AD5770R_MONITOR_SETUP_MUX_BUFFER(x)		(((x) & 0x1) << 5)
#define AD5770R_MONITOR_SETUP_IB_EXT_EN_MSK		BIT(4)
#define AD5770R_MONITOR_SETUP_IB_EXT_EN(x)		(((x) & 0x1) << 4)
#define AD5770R_MONITOR_SETUP_MON_CH_MSK		GENMASK(3, 0)
#define AD5770R_MONITOR_SETUP_MON_CH(x)			(((x) & 0x7) << 0)

/* AD5770R_STATUS */
#define AD5770R_STATUS_BACKGROUND_CRC_STATUS_MSK	BIT(7)
#define AD5770R_STATUS_IREF_FAULT_MSK			BIT(3)
#define AD5770R_STATUS_NEGATIVE_CHANNEL0_MSK		BIT(2)
#define AD5770R_STATUS_OVER_TEMP_MSK			BIT(1)
#define AD5770R_STATUS_TEMP_WARNING_MSK			BIT(0)

/* AD5770R_HW_LDAC */
#define AD5770R_HW_LDAC_MASK_CH(x, channel)		(((x) & 0x1) << (channel))

/* AD5770R_CH_DAC */
#define AD5770R_CH_DAC_DATA_LSB(x)			(((x) & 0x3F) << 2)
#define AD5770R_CH_DAC_DATA_MSB(x)			(((x) & 0xFF) << 0)

/* AD5770R_CH_SELECT */
#define AD5770R_CH_SELECT_SEL_CH(x, channel)		(((x) & 0x1) << (channel))

/* AD5770R_CH_INPUT */
#define AD5770R_CH_DAC_INPUT_DATA_LSB(x)		(((x) & 0x3F) << 2)
#define AD5770R_CH_DAC_INPUT_DATA_MSB(x)		(((x) & 0xFF) << 0)

/* AD5770R_SW_LDAC */
#define AD5770R_SW_LDAC_CH(x, channel)			(((x) & 0x1) << (channel))

/* AD5770R_CH_ENABLE */
#define AD5770R_CH_ENABLE_SET(x, channel)		(((x) & 0x1) << (channel))

#define AD5770R_REG_READ(x)				(((x) & 0x7F) | 0x80)
#define AD5770R_REG_WRITE(x)				((x) & 0x7F)

#define AD5770R_MAX_CHANNELS	6

enum ad5770r_output_filter_resistor {
	AD5770R_OUTPUT_FILTER_RESISTOR_60_OHM = 0x0,
	AD5770R_OUTPUT_FILTER_RESISTOR_5_6_KOHM = 0x5,
	AD5770R_OUTPUT_FILTER_RESISTOR_11_2_KOHM,
	AD5770R_OUTPUT_FILTER_RESISTOR_22_2_KOHM,
	AD5770R_OUTPUT_FILTER_RESISTOR_44_4_KOHM,
	AD5770R_OUTPUT_FILTER_RESISTOR_104_KOHM,
};

enum ad5770r_monitor_function {
	AD5770R_DISABLE = 0,
	AD5770R_VOLTAGE_MONITORING,
	AD5770R_CURRENT_MONITORING,
	AD5770R_TEMPERATURE_MONITORING
};

enum ad5770r_ch {
	AD5770R_CH0 = 0,
	AD5770R_CH1,
	AD5770R_CH2,
	AD5770R_CH3,
	AD5770R_CH4,
	AD5770R_CH5
};

enum ad5770r_ref_v {
	AD5770R_EXT_REF_2_5_V = 0,
	AD5770R_INT_REF_1_25_V_OUT_ON,
	AD5770R_EXT_REF_1_25_V,
	AD5770R_INT_REF_1_25_V_OUT_OFF
};

struct ad5770r_monitor_setup {
	enum ad5770r_monitor_function	monitor_function;
	bool				mux_buffer;
	bool				ib_ext_en;
	enum ad5770r_ch			monitor_channel;
};

struct ad5770r_dac_page_mask {
	u16	dac_data_page_mask;
	u16	input_page_mask;
};

struct ad5770r_output_range {
	u8	output_scale;
	u8	output_range_mode;
};

struct ad5770r_dev_spi_setting {
	bool	addr_ascension;
	bool	single_instruction; /* for multibyte read/write */
	u8	stream_mode_length;
};

struct ad5770r_channel_switches {
	bool en0, en1, en2, en3, en4, en5, sink0;
};

struct ad5770r_alarm_cfg {
	bool open_drain_en;
	bool thermal_shutdown_en;
	bool background_crc_en;
	bool temp_warning_msk;
	bool over_temp_msk;
	bool neg_ch0_msk;
	bool iref_fault_msk;
	bool background_crc_msk;
};

/**
 * struct ad5770R_state - driver instance specific data
 * @spi:	spi_device
 */

struct ad5770r_state {
	struct spi_device *spi;
	struct regmap	*regmap;
	struct gpio_desc *gpio_reset;
	struct ad5770r_output_range	output_mode[AD5770R_MAX_CHANNELS];
	struct ad5770r_alarm_cfg	alarm_config;
	struct ad5770r_monitor_setup	mon_setup;
	bool				ext_ref;
	enum ad5770r_ref_v		ref_sel;
	struct ad5770r_channel_switches	ch_config;
	struct ad5770r_channel_switches	ch_enable;
};

static const struct regmap_config ad5770r_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
	.reg_format_endian = REGMAP_ENDIAN_LITTLE,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

struct ad5770r_range {
	int min;
	int max;
};

static struct ad5770r_range ad5770r_min_max_table[] = {
{ 0, 300 },
{ 0, 140 },
{ 0, 55 },
{ 0, 45 },
{ 0, 45 },
{ 0, 45 },
};

#define AD5770R_IDAC_CHANNEL(index, reg) {			\
	.type = IIO_CURRENT,					\
	.address = reg,						\
	.indexed = 1,						\
	.channel = index,					\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec ad5770r_channels[] = {
	AD5770R_IDAC_CHANNEL(0, AD5770R_CH0_DAC_MSB),
	AD5770R_IDAC_CHANNEL(1, AD5770R_CH1_DAC_MSB),
	AD5770R_IDAC_CHANNEL(2, AD5770R_CH2_DAC_MSB),
	AD5770R_IDAC_CHANNEL(3, AD5770R_CH3_DAC_MSB),
	AD5770R_IDAC_CHANNEL(4, AD5770R_CH4_DAC_MSB),
	AD5770R_IDAC_CHANNEL(5, AD5770R_CH5_DAC_MSB),
};

static int ad5770r_set_channel_en(struct ad5770r_state *st,
		const struct ad5770r_channel_switches *channel_enable)
{
	if (!st || !channel_enable)
		return ENOMEM;

	return regmap_write(st->regmap,
		AD5770R_CH_ENABLE,
		AD5770R_CH_ENABLE_SET(channel_enable->en0, AD5770R_CH0) |
		AD5770R_CH_ENABLE_SET(channel_enable->en1, AD5770R_CH1) |
		AD5770R_CH_ENABLE_SET(channel_enable->en2, AD5770R_CH2) |
		AD5770R_CH_ENABLE_SET(channel_enable->en3, AD5770R_CH3) |
		AD5770R_CH_ENABLE_SET(channel_enable->en4, AD5770R_CH4) |
		AD5770R_CH_ENABLE_SET(channel_enable->en5, AD5770R_CH5));
}

static int ad5770r_channel_config(struct ad5770r_state *st,
		const struct ad5770r_channel_switches *channel_config)
{
	if (!st || !channel_config)
		return ENOMEM;

	return regmap_write(st->regmap,
	AD5770R_CHANNEL_CONFIG,
	AD5770R_CHANNEL_CONFIG_CH0_SHUTDOWN_B(channel_config->en0) |
	AD5770R_CHANNEL_CONFIG_CH1_SHUTDOWN_B(channel_config->en1) |
	AD5770R_CHANNEL_CONFIG_CH2_SHUTDOWN_B(channel_config->en2) |
	AD5770R_CHANNEL_CONFIG_CH3_SHUTDOWN_B(channel_config->en3) |
	AD5770R_CHANNEL_CONFIG_CH4_SHUTDOWN_B(channel_config->en4) |
	AD5770R_CHANNEL_CONFIG_CH5_SHUTDOWN_B(channel_config->en5) |
	AD5770R_CHANNEL_CONFIG_CH0_SINK_EN(channel_config->sink0));
}

static int ad5770r_set_output_mode(struct ad5770r_state *st,
				const struct ad5770r_output_range *output_mode,
				enum ad5770r_ch channel)
{
	if (!st || !output_mode)
		return ENOMEM;

	return regmap_write(st->regmap,
	AD5770R_OUTPUT_RANGE_CH0 + channel,
	AD5770R_OUTPUT_RANGE_OUTPUT_SCALING(output_mode->output_scale) |
	AD5770R_OUTPUT_RANGE_MODE(output_mode->output_range_mode));
}

static int ad5770r_set_alarm(struct ad5770r_state *st,
			  const struct ad5770r_alarm_cfg *const alarm_config)
{
	if (!st || !alarm_config)
		return ENOMEM;

	return regmap_write(st->regmap, AD5770R_ALARM_CONFIG,
AD5770R_ALARM_CONFIG_OPEN_DRAIN_EN(alarm_config->open_drain_en) |
AD5770R_ALARM_CONFIG_THERMAL_SHUTDOWN_EN(alarm_config->thermal_shutdown_en) |
AD5770R_ALARM_CONFIG_BACKGROUND_CRC_EN(alarm_config->background_crc_en) |
AD5770R_ALARM_CONFIG_TEMP_WARNING_ALARM_MASK(alarm_config->temp_warning_msk) |
AD5770R_ALARM_CONFIG_OVER_TEMP_ALARM_MASK(alarm_config->over_temp_msk) |
AD5770R_ALARM_CONFIG_NEGATIVE_CHANNEL0_ALARM_MASK(alarm_config->neg_ch0_msk) |
AD5770R_ALARM_CONFIG_IREF_FAULT_ALARM_MASK(alarm_config->iref_fault_msk) |
AD5770R_ALARM_CONFIG_BACKGROUND_CRC_ALARM_MASK(alarm_config->background_crc_msk));
}

static int ad5770r_set_monitor_setup(struct ad5770r_state *st,
				  const struct ad5770r_monitor_setup *mon_setup)
{
	if (!st || !mon_setup)
		return ENOMEM;

	return regmap_write(st->regmap,
	AD5770R_MONITOR_SETUP,
	AD5770R_MONITOR_SETUP_MON_CH(mon_setup->monitor_channel) |
	AD5770R_MONITOR_SETUP_IB_EXT_EN(mon_setup->ib_ext_en) |
	AD5770R_MONITOR_SETUP_MUX_BUFFER(mon_setup->mux_buffer) |
	AD5770R_MONITOR_SETUP_MON_FUNCTION(mon_setup->monitor_function));
}

static int ad5770r_set_reference(struct ad5770r_state *st,
		bool external_reference,
		enum ad5770r_ref_v reference_selector)
{
	if (!st)
		return ENOMEM;

	return regmap_write(st->regmap,
		AD5770R_REFERENCE,
		AD5770R_REFERENCE_RESISTOR_SEL(external_reference) |
		AD5770R_REFERENCE_VOLTATE_SEL(reference_selector));

}

static int ad5770r_soft_reset(struct ad5770r_state *st)
{
	if (!st)
		return ENOMEM;

	return regmap_write(st->regmap,
		AD5770R_INTERFACE_CONFIG_A,
		AD5770R_INTERFACE_CONFIG_A_SW_RESET(1));
}

static int ad5770r_reset(struct ad5770r_state *st)
{
	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 0);
		udelay(100);
		gpiod_set_value(st->gpio_reset, 1);
	} else {
		/* Perform a software reset */
		return ad5770r_soft_reset(st);
	}

	mdelay(1);

	return 0;
}

static int ad5770r_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int max, min, ret;
	u8 buf[2];

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read(st->regmap,
			AD5770R_REG_READ(chan->address),
			buf, 2);
		*val = ((u16)buf[0] << 6) + (buf[1] >> 2);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		min = ad5770r_min_max_table[chan->channel].min;
		max = ad5770r_min_max_table[chan->channel].max;
		*val = max - min;
		*val2 = 14;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad5770r_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int ret;
	u8 buf[2];

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		buf[0] = ((u16)val >> 6) & 0xFF;
		buf[1] = (val & 0x3F) << 2;
		ret = regmap_bulk_write(st->regmap, chan->address,
			 buf, 2);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ad5770r_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int ret;

	if (readval == NULL)
		ret = regmap_write(st->regmap, reg, writeval);
	else
		ret = regmap_read(st->regmap, AD5770R_REG_READ(reg), readval);

	return ret;
}

static const struct iio_info ad5770r_info = {
	.read_raw = ad5770r_read_raw,
	.write_raw = ad5770r_write_raw,
	.debugfs_reg_access = &ad5770r_reg_access,
};

static void ad5770r_update_range(struct ad5770r_state *st)
{
	int i;

	if (st->output_mode[AD5770R_CH0].output_range_mode == 0x00) {
		ad5770r_min_max_table[AD5770R_CH0].min = 0;
		ad5770r_min_max_table[AD5770R_CH0].max = 300;
	} else if (st->output_mode[AD5770R_CH0].output_range_mode == 0x01) {
		ad5770r_min_max_table[AD5770R_CH0].min = -60;
		ad5770r_min_max_table[AD5770R_CH0].max = 0;
	} else if (st->output_mode[AD5770R_CH0].output_range_mode == 0x02) {
		ad5770r_min_max_table[AD5770R_CH0].min = -60;
		ad5770r_min_max_table[AD5770R_CH0].max = 300;
	}

	/* Low headroom */
	if (st->output_mode[AD5770R_CH1].output_range_mode == 0x01) {
		ad5770r_min_max_table[AD5770R_CH1].min = 0;
		ad5770r_min_max_table[AD5770R_CH1].max = 140;
	/* Low noise and PSRR */
	} else if (st->output_mode[AD5770R_CH1].output_range_mode == 0x02) {
		ad5770r_min_max_table[AD5770R_CH1].min = 0;
		ad5770r_min_max_table[AD5770R_CH1].max = 140;
	} else if (st->output_mode[AD5770R_CH1].output_range_mode == 0x03) {
		ad5770r_min_max_table[AD5770R_CH1].min = 0;
		ad5770r_min_max_table[AD5770R_CH1].max = 250;
	}

	if (st->output_mode[AD5770R_CH2].output_range_mode == 0x00) {
		ad5770r_min_max_table[AD5770R_CH2].min = 0;
		ad5770r_min_max_table[AD5770R_CH2].max = 55;
	} else if (st->output_mode[AD5770R_CH2].output_range_mode == 0x01) {
		ad5770r_min_max_table[AD5770R_CH2].min = 0;
		ad5770r_min_max_table[AD5770R_CH2].max = 150;
	}

	for (i = AD5770R_CH2; i < AD5770R_MAX_CHANNELS; i++)
		if (st->output_mode[i].output_range_mode == 0x00) {
			ad5770r_min_max_table[i].min = 0;
			ad5770r_min_max_table[i].max = 45;
		} else if (st->output_mode[i].output_range_mode == 0x01) {
			ad5770r_min_max_table[i].min = 0;
			ad5770r_min_max_table[i].max = 100;
		}
};

static void ad5770r_parse_dt(struct ad5770r_state *st)
{
	unsigned int i;
	u8 tmparray[8], tmp;

	/* Default range for CH1 */
	st->output_mode[AD5770R_CH1].output_range_mode = 2;
	if (!device_property_read_u8_array(&st->spi->dev, "adi,output",
					    tmparray, 6))
		for (i = 0; i < AD5770R_MAX_CHANNELS; i++)
			st->output_mode[i].output_range_mode = tmparray[i];

	ad5770r_update_range(st);

	if (!device_property_read_u8_array(&st->spi->dev, "adi,alarm",
					    tmparray, 8)) {
		st->alarm_config.open_drain_en = tmparray[0];
		st->alarm_config.thermal_shutdown_en = tmparray[1];
		st->alarm_config.background_crc_en = tmparray[2];
		st->alarm_config.temp_warning_msk = tmparray[3];
		st->alarm_config.over_temp_msk = tmparray[4];
		st->alarm_config.neg_ch0_msk = tmparray[5];
		st->alarm_config.iref_fault_msk = tmparray[6];
		st->alarm_config.background_crc_msk = tmparray[7];
	}

	if (!device_property_read_u8_array(&st->spi->dev, "adi,monitor",
					    tmparray, 4)) {
		st->mon_setup.monitor_function = tmparray[0];
		st->mon_setup.mux_buffer = tmparray[1];
		st->mon_setup.ib_ext_en = tmparray[2];
		st->mon_setup.monitor_channel = tmparray[3];
	}

	if (!device_property_read_u8(&st->spi->dev, "adi,ref_voltage",
					 &tmp))
		st->ref_sel = tmp;

	if (!device_property_read_u8(&st->spi->dev, "adi,external_ref",
					&tmp))
		st->ext_ref = tmp;

	if (!device_property_read_u8_array(&st->spi->dev,
		"adi,channel_cfg", tmparray, 7)) {
		st->ch_config.en0 = tmparray[0];
		st->ch_config.en1 = tmparray[1];
		st->ch_config.en2 = tmparray[2];
		st->ch_config.en3 = tmparray[3];
		st->ch_config.en4 = tmparray[4];
		st->ch_config.en5 = tmparray[5];
		st->ch_config.sink0 = tmparray[6];
	}

	if (!device_property_read_u8_array(&st->spi->dev,
		"adi,channel_enable", tmparray, 6)) {
		st->ch_config.en0 = tmparray[0];
		st->ch_config.en1 = tmparray[1];
		st->ch_config.en2 = tmparray[2];
		st->ch_config.en3 = tmparray[3];
		st->ch_config.en4 = tmparray[4];
		st->ch_config.en5 = tmparray[5];
	}
}

static int ad5770r_init(struct ad5770r_state *st)
{
	int ret, i;

	ad5770r_parse_dt(st);

	st->gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
						 GPIOD_OUT_HIGH);

	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	/* Perform a reset */
	ret = ad5770r_reset(st);
	if (ret < 0)
		return ret;

	/* Set output range */
	for (i = 0; i < AD5770R_MAX_CHANNELS; i++)
		ret = ad5770r_set_output_mode(st,
			&st->output_mode[i], i);

	ret |= ad5770r_set_alarm(st, &st->alarm_config);
	ret |= ad5770r_set_monitor_setup(st, &st->mon_setup);
	ret |= ad5770r_set_reference(st, st->ext_ref, st->ref_sel);

	ret |= ad5770r_channel_config(st, &st->ch_config);
	ret |= ad5770r_set_channel_en(st, &st->ch_enable);

	return ret;
}

static int ad5770r_probe(struct spi_device *spi)
{
	struct ad5770r_state *st;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	regmap = devm_regmap_init_spi(spi, &ad5770r_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	st->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5770r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5770r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5770r_channels);

	ret = ad5770r_init(st);

	if (ret < 0) {
		dev_err(&spi->dev, "AD5770R init failed\n");
		return ret;
	}

	ret = iio_device_register(indio_dev);

	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device\n");
		return ret;
	}

	return 0;
}

static const struct spi_device_id ad5770r_id[] = {
	{ "ad5770r", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5770r_id);

static struct spi_driver ad5770r_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad5770r_probe,
	.id_table = ad5770r_id,
};

module_spi_driver(ad5770r_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5770 IDAC");
MODULE_LICENSE("GPL v2");
