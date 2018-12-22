/*
 * @file lsm303dlhc_mag.c
 * driver for the LSM303DLHC 3D Magnetometer module connected via I2C
 *
 * Write by Sarlor, 01/19/2017
 *  */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* register addresses: M: mag, T: temp */
#define ADDR_CRA_REG_M			0x00
#define ADDR_CRB_REG_M			0x01
#define ADDR_MR_REG_M			0x02

#define ADDR_OUT_X_H_M			0x03
#define ADDR_OUT_X_L_M			0x04
#define ADDR_OUT_Z_H_M			0x05
#define ADDR_OUT_Z_L_M			0x06
#define ADDR_OUT_Y_H_M			0x07
#define ADDR_OUT_Y_L_M			0x08

#define ADDR_SR_REG_M			0x09
#define ADDR_IRA_REG_M			0x0a
#define ADDR_IRB_REG_M			0x0b
#define ADDR_IRC_REG_M			0x0c

#define ADDR_TEMP_OUT_H_M		0x31
#define ADDR_TEMP_OUT_L_M		0x32

#define REG_CRA_TEMP_BITS		(1<<7)
#define REG_CRA_TEMP_EN			(1<<7)
#define REG_CRA_RATE_BITS		((1<<4) | (1<<3) | (1<<2))
#define REG_CRA_RATE_0_75HZ		((0<<4) | (0<<3) | (0<<2))
#define REG_CRA_RATE_1_5HZ		((0<<4) | (0<<3) | (1<<2))
#define REG_CRA_RATE_3HZ		((0<<4) | (1<<3) | (0<<2))
#define REG_CRA_RATE_7_5HZ		((0<<4) | (1<<3) | (1<<2))
#define REG_CRA_RATE_15HZ		((1<<4) | (0<<3) | (0<<2))
#define REG_CRA_RATE_30HZ		((1<<4) | (0<<3) | (1<<2))
#define REG_CRA_RATE_75HZ		((1<<4) | (1<<3) | (0<<2))
#define REG_CRA_RATE_220HZ		((1<<4) | (1<<3) | (1<<2))

#define REG_GAIN_BITS			((1<<7) | (1<<6) | (1<<5))
#define REG_GAIN_1_3_G			((0<<7) | (0<<6) | (1<<5))
#define REG_GAIN_1_9G			((0<<7) | (1<<6) | (0<<5))
#define REG_GAIN_2_5G			((0<<7) | (1<<6) | (1<<5))
#define REG_GAIN_4G			((1<<7) | (0<<6) | (0<<5))
#define REG_GAIN_4_7G			((1<<7) | (0<<6) | (1<<5))
#define REG_GAIN_5_6G			((1<<7) | (1<<6) | (0<<5))
#define REG_GAIN_8_1G			((1<<7) | (1<<6) | (1<<5))

#define REG_MR_MODE_BITS		((1<<1) | (1<<0))
#define REG_MR_MODE_CONTIN		((0<<1) | (0<<0))
#define REG_MR_MODE_SINGLE		((0<<1) | (1<<0))
#define REG_MR_MODE_SLEEP		((1<<1) | (0<<0))

#define REG_SR_LOCK_BITS		(1<<1)
#define REG_SR_DRDY_BITS		(1<<0)

/* Device info */
#define SENSOR_NAME			"lsm303dlhc_mag"
#define ABSMIN				-32
#define ABSMAX				31
#define FUZZ				1
#define LSG				21
#define MAX_DELAY			50

#define MODE_STANDBY			0x00		/* Power-down mode */
#define MODE_ACTIVE			0x01		/* Normal mode	   */

/* Magnetometer */
struct lsm303dlhc_mag {
	s16 x;
	s16 y;
	s16 z;
};

/* lsm303 deviece */
struct lsm303dlhc_data {
	struct i2c_client *lsm303dlhc_client;
	struct input_dev *input;
	struct mutex enable_mutex;
	struct delayed_work work;
	struct lsm303dlhc_mag offset;
	atomic_t delay;
	atomic_t enable;
	atomic_t position;
	atomic_t calibrated;
	atomic_t fuzz;
};

/*
 * converts the complement code to the original code (word).
 * @number: complement will to converted.
 *
 * Returning the value of the primitive to be converted.
 *  */
static int lsm303dlhc_complement_to_original(unsigned int number)
{
	s16 complement;

	if ((int)number < 0)
		complement = (s16)((~(number-0x0001))|0x8000);
	else
		complement = (s16)number;

	return complement;
}

/*
 * the adjusted data value.
 * @number: adjustment data.
 *
 * Since the valid data range of the magnetometer is 0xF800 to 0X07FF, it is
 * necessary to process the acquired data. Returning the adjusted data value.
 *  */
static int lsm303dlhc_data_adjustment(unsigned int number)
{
	if ((int)number < 0)
		return -(number & 0x07ff);
	else
		return (number & 0x07ff);
}

/*
 * lsm303dlhc read byte.
 * @client: Handle to slave device.
 * @reg: Byte interpreted by slave.
 * @data: Byte being stored.
 *
 * From the specified register 'reg' to read a byte of data, stored in
 * the '*data'. Returning negative errno else zero on success.
 *  */
static int lsm303dlhc_smbus_read_byte(struct i2c_client *client,
			unsigned char reg, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_read_byte_data(client, reg);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

/*
 * lsm303dlhc write byte.
 * @client: Handle to slave device.
 * @reg: Byte interpreted by slave.
 * @data: Byte being written.
 *
 * Tells the specified contents '*data' to be written to the specified
 * register 'reg'. Returning negative errno else zero on success.
 *  */
static int lsm303dlhc_smbus_write_byte(struct i2c_client *client,
		unsigned char reg, unsigned char *data)
{
	if (i2c_smbus_write_byte_data(client, reg, *data) < 0)
		return -EAGAIN;
	return 0;
}

/*
 * modify the value of the register.
 * @client: Handle to slave device.
 * @reg: modified register.
 * @clearbits: mask value.
 * @setbits: set value.
 *
 * Modify the value of the register 'reg' to 'setbits'. Returning negative errno
 * else zero on success.
 *  */
static int modify_reg(struct i2c_client *client, unsigned char reg,
		uint8_t clearbits, uint8_t setbits)
{
	unsigned char val;

	if (lsm303dlhc_smbus_read_byte(client, reg, &val) < 0)
		return -EAGAIN;

	val &= ~clearbits;
	val |= setbits;

	if (lsm303dlhc_smbus_write_byte(client, reg, &val) < 0)
		return -EAGAIN;
	return 0;
}

/*
 * reads the value of several bits in succession.
 * @client: Handle to slave device.
 * @reg: read register.
 * @data: stores the value read.
 * @start: the index to start reading.
 * @len: the length of the read.
 *
 * From the specified register 'reg', read the 'len' bit data from the index
 * 'start' and store it in '*data'. Returning negative errno else zero on
 * success.
 *  */
static int lsm303dlhc_smbus_read_bits(struct i2c_client *client,
		unsigned char reg, unsigned char *data,
		unsigned char start, unsigned char len)
{
	unsigned char tmp;

	if(start >= 8 || len > 8)
		return -EINVAL;
	if (lsm303dlhc_smbus_read_byte(client, reg, &tmp) < 0)
		return -EAGAIN;

	*data = (tmp >> start) & (0xff >> (8-len));
	return 0;
}

/*
 * lsm303dlhc magnetometer initialization.
 * @client: Handle to slave device.
 *  */
#define REG_DEF_NUM 3
static int lsm303dlhc_hw_init(struct i2c_client *client)
{
	unsigned char i;
	u8 reg_addr[REG_DEF_NUM] =
	{
		ADDR_MR_REG_M,
		ADDR_CRA_REG_M,
		ADDR_CRB_REG_M
	};
	u8 reg_data[REG_DEF_NUM] =
	{
		REG_MR_MODE_CONTIN,
		REG_CRA_TEMP_EN | REG_CRA_RATE_75HZ,
		REG_GAIN_1_9G
	};

	for(i = 0; i < REG_DEF_NUM; i++)
		lsm303dlhc_smbus_write_byte(client, reg_addr[i], reg_data+i);

	return 0;
}

/*
 * lsm303dlhc read x/y/z axis magnetic field data.
 * @client: Handle to slave device.
 * @mag: to saving magnetometer data.
 *
 * The value is expressed as two's complement. So, you need to convert it
 * to the original code.
 *  */
static int lsm303dlhc_read_mag_data(struct i2c_client *client,
		struct lsm303dlhc_mag *mag)
{
	unsigned char tmp_data[7], i;
	u16	temp[3];

	for (i=0; i<7; i++)
		lsm303dlhc_smbus_read_byte(client,ADDR_OUT_X_H_M+i, tmp_data+i);

	temp[0] = ((tmp_data[0] << 8) & 0xff00) | tmp_data[1];
	temp[1] = ((tmp_data[2] << 8) & 0xff00) | tmp_data[3];
	temp[2] = ((tmp_data[4] << 8) & 0xff00) | tmp_data[5];

	temp[0] = lsm303dlhc_complement_to_original(temp[0]);
	temp[1] = lsm303dlhc_complement_to_original(temp[1]);
	temp[2] = lsm303dlhc_complement_to_original(temp[2]);

	mag->x = lsm303dlhc_data_adjustment(temp[0]);
	mag->z = lsm303dlhc_data_adjustment(temp[1]);
	mag->y = lsm303dlhc_data_adjustment(temp[2]);

	return 0;
}

/*
 * temperature sensor enable.
 * @data: ON or OFF.
 *
 * The temperature sensor switch. 0 is temperature sensor disabled, 1 is
 * enabled.
 *   */
static int set_temperature(struct lsm303dlhc_data *lsm303dlhc,
		unsigned char data)
{
	unsigned char clearbits, setbits;

	clearbits = REG_CRA_TEMP_BITS;
	setbits = (data & 0x01) << 7;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CRA_REG_M,
				clearbits, setbits) < 0)
		return -1;
	return 0;
}

static ssize_t lsm303dlhc_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char i, data;
	size_t count = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	for (i=ADDR_CRA_REG_M; i<=ADDR_IRC_REG_M; i++) {
		if (lsm303dlhc_smbus_read_byte(lsm303dlhc->lsm303dlhc_client,
					i, &data) >= 0)
			count += sprintf(buf+count, "0x%02x:0x%02x ", i, data);
	}

	for (i=ADDR_TEMP_OUT_H_M; i<=ADDR_TEMP_OUT_L_M; i++) {
		if (lsm303dlhc_smbus_read_byte(lsm303dlhc->lsm303dlhc_client,
					i, &data) >= 0)
			count += sprintf(buf+count, "0x%02x:0x%02x ", i, data);
	}

	buf[count-1] = '\0';
	return count;
}

static ssize_t lsm303dlhc_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int addr, value;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	sscanf(buf, "0x%x:0x%x", &addr, &value);
	if (lsm303dlhc_smbus_write_byte(lsm303dlhc->lsm303dlhc_client,
			(unsigned char)addr, (unsigned char *)&value) < 0)
		return -EINVAL;

	return count;
}

static ssize_t lsm303dlhc_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	return sprintf(buf, "%d", atomic_read(&lsm303dlhc->delay));
}

static ssize_t lsm303dlhc_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	atomic_set(&lsm303dlhc->delay, (unsigned int)data);
	return count;
}

static ssize_t lsm303dlhc_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	return sprintf(buf, "%d", atomic_read(&lsm303dlhc->position));
}

static ssize_t lsm303dlhc_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtol(buf, 10, &data);
	if (err)
		return err;

	atomic_set(&lsm303dlhc->position, (int)data);
	return count;
}

static ssize_t lsm303dlhc_fuzz_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	return sprintf(buf, "%d", atomic_read(&lsm303dlhc->fuzz));
}

static ssize_t lsm303dlhc_fuzz_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	atomic_set(&lsm303dlhc->fuzz, (int)data);
	if(lsm303dlhc->input != NULL) {
		lsm303dlhc->input->absinfo[ABS_X].fuzz = data;
		lsm303dlhc->input->absinfo[ABS_Y].fuzz = data;
		lsm303dlhc->input->absinfo[ABS_Z].fuzz = data;
	}

	return count;
}

static ssize_t lsm303dlhc_temp_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CRA_REG_M, &data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_temp_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (set_temperature(lsm303dlhc, data) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CRA_REG_M, &data, 2, 3) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = REG_CRA_RATE_BITS;
	setbits = (data & 0x07) << 2;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CRA_REG_M,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CRB_REG_M, &data, 5, 3) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_range_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (data < 1 || data > 7)
		return -EINVAL;

	clearbits = REG_GAIN_BITS;
	setbits = (data & 0x07) << 5;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CRB_REG_M,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_MR_REG_M, &data, 0, 2) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	clearbits = REG_MR_MODE_BITS;
	setbits = data & 0x03;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_MR_REG_M,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_mag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);
	struct lsm303dlhc_mag mag = {0};

	lsm303dlhc_read_mag_data(lsm303dlhc->lsm303dlhc_client, &mag);
	return sprintf(buf, "%d %d %d", mag.x, mag.y, mag.z);
}

static ssize_t lsm303dlhc_lock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_SR_REG_M, &data, 1, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_drdy_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_SR_REG_M, &data, 0, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data[2], i;
	s16 tmp;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	for (i=0; i<2; i++) {
		if (lsm303dlhc_smbus_read_byte(lsm303dlhc->lsm303dlhc_client,
					ADDR_TEMP_OUT_H_M+i, data+i) < 0)
			return sprintf(buf, "%d", -EAGAIN);
	}

	tmp = ((data[0] << 8) | data[1]) >> 4;
	return sprintf(buf, "%d", lsm303dlhc_complement_to_original(tmp));
}

static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_reg_show, lsm303dlhc_reg_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_delay_show, lsm303dlhc_delay_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_position_show, lsm303dlhc_position_store);
static DEVICE_ATTR(fuzz, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_fuzz_show, lsm303dlhc_fuzz_store);
static DEVICE_ATTR(temp_en, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_temp_en_show, lsm303dlhc_temp_en_store);
static DEVICE_ATTR(sample, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_sample_show, lsm303dlhc_sample_store);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_range_show, lsm303dlhc_range_store);
static DEVICE_ATTR(mode, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_mode_show, lsm303dlhc_mode_store);
static DEVICE_ATTR(mag, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_mag_show, NULL);
static DEVICE_ATTR(lock, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_lock_show, NULL);
static DEVICE_ATTR(drdy, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_drdy_show, NULL);
static DEVICE_ATTR(temperature, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_temperature_show, NULL);

static struct attribute *lsm303dlhc_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_fuzz.attr,
	&dev_attr_temp_en.attr,
	&dev_attr_sample.attr,
	&dev_attr_range.attr,
	&dev_attr_mode.attr,
	&dev_attr_mag.attr,
	&dev_attr_lock.attr,
	&dev_attr_drdy.attr,
	&dev_attr_temperature.attr,
	NULL
};

static struct attribute_group lsm303dlhc_attribute_group = {
	.attrs = lsm303dlhc_attributes
};

/* periodically read the magnetometer data and reported. */
static void lsm303dlhc_work_func(struct work_struct *work)
{
	int result;
	struct lsm303dlhc_data *lsm303dlhc = container_of((struct delayed_work *)work,
				struct lsm303dlhc_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&lsm303dlhc->delay));
	struct lsm303dlhc_mag mag;

	result = lsm303dlhc_read_mag_data(lsm303dlhc->lsm303dlhc_client, &mag);
	if (result == 0) {
		input_report_abs(lsm303dlhc->input, ABS_X, mag.x);
		input_report_abs(lsm303dlhc->input, ABS_Y, mag.y);
		input_report_abs(lsm303dlhc->input, ABS_Z, mag.z);
		input_sync(lsm303dlhc->input);
	}

	schedule_delayed_work(&lsm303dlhc->work, delay);
}

static int lsm303dlhc_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err, cfg_position;
	int cfg_calibration[3];
	struct lsm303dlhc_data *data;
	struct input_dev *dev;

	/* Check whether the client's adapter supports the I2C interface */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		err = -EINVAL;
		goto exit;
	}

	data = kzalloc(sizeof(struct lsm303dlhc_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	data->lsm303dlhc_client = client;
	mutex_init(&data->enable_mutex);

	INIT_DELAYED_WORK(&data->work, lsm303dlhc_work_func);
	atomic_set(&data->delay, MAX_DELAY);
	atomic_set(&data->enable, 0);

	cfg_position = -3;
	atomic_set(&data->position, cfg_position);
	atomic_set(&data->calibrated, 0);
	atomic_set(&data->fuzz, FUZZ);

	err = lsm303dlhc_hw_init(data->lsm303dlhc_client);
	if (err < 0) {
		printk(KERN_ERR "lsm303dlhc magnetometer hardware init fail\n");
		goto kfree_exit;
	}

	dev = input_allocate_device();
	if (!dev) {
		printk(KERN_ERR "lsm303dlhc magnetometer input_allocate_device fail\n");
		err = -ENOMEM;
		goto kfree_exit;
	}
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, FUZZ, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, FUZZ, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, FUZZ, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		printk(KERN_INFO "lsm303dlhc magnetometer input_register_device fail\n");
		input_free_device(dev);
		goto kfree_exit;
	}

	data->input = dev;
	err = sysfs_create_group(&data->input->dev.kobj,
			&lsm303dlhc_attribute_group);
	if (err < 0)
		goto error_sysfs;

	memset(cfg_calibration, 0, sizeof(cfg_calibration));

	data->offset.x = (signed short)cfg_calibration[0];
	data->offset.y = (signed short)cfg_calibration[1];
	data->offset.z = (signed short)cfg_calibration[2];

	dev_info(&data->lsm303dlhc_client->dev,
			"Successfully initialized lsm303dlhc magnetometer!\n");
	return 0;

error_sysfs:
	input_unregister_device(data->input);
kfree_exit:
	kfree(data);
exit:
	return err;
}

static int __devexit lsm303dlhc_remove(struct i2c_client *client)
{
	struct lsm303dlhc_data *data = i2c_get_clientdata(client);

	set_temperature(data, 0);
	sysfs_remove_group(&data->input->dev.kobj, &lsm303dlhc_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static int lsm303dlhc_suspend(struct i2c_client *client, pm_message_t state)
{
	struct lsm303dlhc_data *data = i2c_get_clientdata(client);

	set_temperature(data, 0);

	return 0;
}

static int lsm303dlhc_resume(struct i2c_client *client)
{
	struct lsm303dlhc_data *data = i2c_get_clientdata(client);

	lsm303dlhc_hw_init(data->lsm303dlhc_client);
	set_temperature(data, 1);

	return 0;
}

static const struct i2c_device_id lsm303dlhc_mag_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lsm303dlhc_mag_id);

static struct i2c_driver lsm303dlhc_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= SENSOR_NAME,
	},
	.class			= I2C_CLASS_HWMON,
	.id_table		= lsm303dlhc_mag_id,
	.probe			= lsm303dlhc_probe,
	.remove			= __devexit_p(lsm303dlhc_remove),
	.suspend 		= lsm303dlhc_suspend,
	.resume  		= lsm303dlhc_resume,
};

static int __init lsm303dlhc_mag_init(void)
{
	return i2c_add_driver(&lsm303dlhc_driver);
}

static void __exit lsm303dlhc_mag_exit(void)
{
	i2c_del_driver(&lsm303dlhc_driver);
}

MODULE_AUTHOR("Sarlor <kinsleer@outlook.com>");
MODULE_DESCRIPTION("LSM303DLHC Magnetometer Driver");
MODULE_LICENSE("GPL");

module_init(lsm303dlhc_mag_init);
module_exit(lsm303dlhc_mag_exit);
