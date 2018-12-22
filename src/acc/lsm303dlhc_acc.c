/*
 * @file lsm303dlhc_acc.c
 * driver for the LSM303DLHC 3D accelerometer module connected via I2C
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

/* register addresses: A: accel, T: temp */
#define ADDR_CTRL_REG1_A		0x20
#define ADDR_CTRL_REG2_A		0x21
#define ADDR_CTRL_REG3_A		0x22
#define ADDR_CTRL_REG4_A		0x23
#define ADDR_CTRL_REG5_A		0x24
#define ADDR_CTRL_REG6_A		0x25

#define ADDR_REFERENCE_A		0x26
#define ADDR_STATUS_REG_A		0x27

#define ADDR_OUT_X_L_A			0x28
#define ADDR_OUT_X_H_A			0x29
#define ADDR_OUT_Y_L_A			0x2a
#define ADDR_OUT_Y_H_A			0x2b
#define ADDR_OUT_Z_L_A			0x2c
#define ADDR_OUT_Z_H_A			0x2d

#define ADDR_FIFO_CTRL_REG_A		0x2e
#define ADDR_FIFO_SRC_REG_A		0x2f

#define ADDR_INT1_CFG_A			0x30
#define ADDR_INT1_SOURCE_A		0x31
#define ADDR_INT1_THS_A			0x32
#define ADDR_INT1_DURATION_A		0x33
#define ADDR_INT2_CFG_A			0x34
#define ADDR_INT2_SOURCE_A		0x35
#define ADDR_INT2_THS_A			0x36
#define ADDR_INT2_DURATION_A		0x37

#define ADDR_CLICK_CFG_A		0x38
#define ADDR_CLICK_SRC_A		0x39
#define ADDR_CLICK_THS_A		0x3a

#define ADDR_TIME_LIMIT_A		0x3b
#define ADDR_TIME_LATENCY_A		0x3c
#define ADDR_TIME_WINDOW_A		0x3d

#define REG1_RATE_BITS			((1<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_PD			((0<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_1HZ			((0<<7) | (0<<6) | (0<<5) | (1<<4))
#define REG1_RATE_10HZ			((0<<7) | (0<<6) | (1<<5) | (0<<4))
#define REG1_RATE_25HZ			((0<<7) | (0<<6) | (1<<5) | (1<<4))
#define REG1_RATE_50HZ			((0<<7) | (1<<6) | (0<<5) | (0<<4))
#define REG1_RATE_100HZ			((0<<7) | (1<<6) | (0<<5) | (1<<4))
#define REG1_RATE_200HZ			((0<<7) | (1<<6) | (1<<5) | (0<<4))
#define REG1_RATE_400HZ			((0<<7) | (1<<6) | (1<<5) | (1<<4))
#define REG1_RATE_1620HZ		((1<<7) | (0<<6) | (0<<5) | (0<<4))
#define REG1_RATE_5376HZ		((1<<7) | (0<<6) | (0<<5) | (1<<4))

#define REG1_LOWPWR_BITS		(1<<3)
#define REG1_AXIS_BITS			((1<<2) | (1<<1) | (1<<0))
#define REG1_Z_EN			(1<<2)
#define REG1_Y_EN			(1<<1)
#define REG1_X_EN			(1<<0)

#define REG2_HIGHPASS_BITS		((1<<7) | (1<<6))
#define REG2_HIGHPASS_RESET_NORML	((0<<7) | (0<<6))
#define REG2_HIGHPASS_REF		((0<<7) | (1<<6))
#define REG2_HIGHPASS_NORML		((1<<7) | (0<<6))
#define REG2_HIGHPASS_AUTO_IRQ		((1<<7) | (1<<6))

#define REG2_HIGHPASS_CUTOFF_BITS	((1<<5) | (1<<4))
#define REG2_FILTER_BITS		(1<<3)
#define REG2_HPCLICK_BITS		(1<<2)
#define REG2_HPIS2_BITS			(1<<1)
#define REG2_HPIS1_BITS			(1<<0)

#define REG3_I1_CLICK_EN		(1<<7)
#define REG3_I1_AOI1_EN			(1<<6)
#define REG3_I1_AOI2_EN			(1<<5)
#define REG3_I1_DRDY1_EN		(1<<4)
#define REG3_I1_DRDY2_EN		(1<<3)
#define REG3_I1_WTM_EN			(1<<2)
#define REG3_I1_OVERRUN_EN		(1<<1)

#define REG4_BDU_BITS			(1<<7)
#define REG4_BDU_NOT_UPDATE		(1<<7)
#define REG4_BLE_BITS			(1<<6)
#define REG4_BLE_MSB			(1<<6)
#define REG4_BLE_LSB			(0<<6)

#define REG4_FULLSCALE_BITS		((1<<5) | (1<<4))
#define REG4_FULLSCALE_2G		((0<<5) | (0<<4))
#define REG4_FULLSCALE_4G		((0<<5) | (1<<4))
#define REG4_FULLSCALE_8G		((1<<5) | (0<<4))
#define REG4_FULLSCALE_16G		((1<<5) | (1<<4))

#define REG4_HIGH_RESOLUTION_BITS	(1<<3)
#define REG4_HIGH_RESOLUTION_EN		(1<<3)
#define REG4_SPI_3_WIRE			(1<<0)
#define REG4_SPI_4_WIRE			(0<<0)

#define REG5_BOOT_BITS			(1<<7)
#define REG5_BOOT_EN			(1<<7)
#define REG5_FIFO_EN_BITS		(1<<6)
#define REG5_FIFO_EN			(1<<6)
#define REG5_LIR_INT1			(1<<3)
#define REG5_D4D_INT1			(1<<2)
#define REG5_LIR_INT2			(1<<1)
#define REG5_D4D_INT2			(1<<0)

#define REG6_I2_CLICK_EN		(1<<7)
#define REG6_I2_INT1_EN			(1<<6)
#define REG6_I2_INT2_EN			(1<<5)
#define REG6_BOOT_I1_EN			(1<<4)
#define REG6_P2_ACT_EN			(1<<3)
#define REG6_H_LACTIVE_EN		(1<<1)

#define REG_STATUS_ZYXOR_BITS		(1<<7)
#define REG_STATUS_ZOR_BITS		(1<<6)
#define REG_STATUS_YOR_BITS		(1<<5)
#define REG_STATUS_XOR_BITS		(1<<4)
#define REG_STATUS_ZYXDA_BITS		(1<<3)
#define REG_STATUS_ZDA_BITS		(1<<2)
#define REG_STATUS_YDA_BITS		(1<<1)
#define REG_STATUS_XDA_BITS		(1<<0)

#define REG_FIFO_MODE_BITS		((1<<7) | (1<<6))
#define REG_FIFO_MODE_BYPASS		((0<<7) | (0<<6))
#define REG_FIFO_MODE_FIFO		((0<<7) | (1<<6))
#define REG_FIFO_MODE_STREAM		((1<<7) | (0<<6))
#define REG_FIFO_MODE_TRIGGER		((1<<7) | (1<<6))

#define REG_FIFO_TRIGGER_INT1		(0<<5)
#define REG_FIFO_TRIGGER_INT2		(1<<5)

/* Device info */
#define SENSOR_NAME			"lsm303dlhc_acc"
#define ABSMIN				-32
#define ABSMAX				31
#define FUZZ				1
#define LSG				21
#define MAX_DELAY			50

#define MODE_STANDBY			0x00		/* Power-down mode */
#define MODE_ACTIVE			0x01		/* Normal mode     */

/* Accelerome */
struct lsm303dlhc_acc {
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
	struct lsm303dlhc_acc offset;
	atomic_t sample;
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
 * lsm303dlhc power-down mode enable.
 * @client: Handle to slave device.
 * @mode: power-mode mode.
 *
 * Sets the power mode of the device. 0 is power-down mode, 1 is normal mode or
 * sleep mode. Returning negative errno else zero on success.
 *  */
static int lsm303dlhc_set_mode(struct i2c_client *client, unsigned char mode)
{
	unsigned char clearbits = REG1_RATE_BITS;
	unsigned char setbits;
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (mode == MODE_ACTIVE)
		mode = atomic_read(&lsm303dlhc->sample);

	setbits = (mode & 0x0f) << 4;
	if (modify_reg(client, ADDR_CTRL_REG1_A, clearbits, setbits) < 0)
		return -EAGAIN;
	return 0;
}

/*
 * sets the sampling frequency of the acceleration.
 * @client: Handle to slave device.
 * @rate: the set sampling frequency.
 *
 * The sampling frequency is different in two different modes. In normal mode,
 * the sampling frequency is 1.344kHz. Returning negative errno else zero
 * on success.
 *  */
static int set_sample(struct i2c_client *client, unsigned long rate)
{
	unsigned char clearbits, setbits;
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (rate == 0)
		return -1;

	if (rate == 1)
		setbits = 1;
	else if (rate == 10)
		setbits = 2;
	else if (rate == 25)
		setbits = 3;
	else if (rate == 50)
		setbits = 4;
	else if (rate == 100)
		setbits = 5;
	else if (rate == 200)
		setbits = 6;
	else if (rate == 400)
		setbits = 7;
	else if (rate == 1620)
		setbits = 8;
	else
		setbits = 9;

	atomic_set(&lsm303dlhc->sample, setbits);

	clearbits = REG1_RATE_BITS;
	setbits = (setbits & 0x0f) << 4;

	if (modify_reg(client, ADDR_CTRL_REG1_A, clearbits, setbits) < 0)
		return -EAGAIN;
	return 0;
}

static void lsm303dlhc_set_enable(struct device *, int);

/*
 * When the new settings and the old settings are not equal, use the new
 * settings.
 *    */
static void update_ctrl_reg1(struct device *dev, unsigned char data)
{
	unsigned char enable = !!((data & REG1_RATE_BITS) >> 4);
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (atomic_read(&lsm303dlhc->enable) != enable)
		lsm303dlhc_set_enable(dev, enable);

	if (enable)
		atomic_set(&lsm303dlhc->sample, (data & 0xf0) >> 4);
}

/* 
 * updates the configuration values.
 * @dev: device.
 * @addr: the updated register address.
 * @value: the updated value.
 * 
 * Returning negative errno else zero on success.
 *   */
static int reg_update(struct device *dev, unsigned char addr,
		unsigned char value)
{
	switch (addr) {
		case ADDR_CTRL_REG1_A:
			update_ctrl_reg1(dev, value);
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int check_value(unsigned char addr, unsigned char value)
{
	switch (addr) {
		case ADDR_CTRL_REG1_A:
			if (value < 0x00 || value > 0x9F)
				return -EINVAL;
			break;
		default:
			return 0;
	}

	return 0;
}

/*
 * lsm303dlhc accelerometer initialization.
 * @client: Handle to slave device.
 *  */
#define REG_DEF_NUM 3
static int lsm303dlhc_hw_init(struct i2c_client *client)
{
	unsigned char i;
	u8 reg_addr[REG_DEF_NUM] =
	{
		ADDR_CTRL_REG1_A,
		ADDR_CTRL_REG4_A,
		ADDR_CTRL_REG3_A
	};
	u8 reg_data[REG_DEF_NUM] =
	{
		REG1_Z_EN | REG1_Y_EN | REG1_X_EN,
		REG4_BDU_NOT_UPDATE | REG4_FULLSCALE_8G | REG4_HIGH_RESOLUTION_EN,
		REG3_I1_DRDY1_EN
	};

	for(i = 0; i < REG_DEF_NUM; i++)
		lsm303dlhc_smbus_write_byte(client, reg_addr[i], reg_data+i);

	set_sample(client, 400);

	return 0;
}

/*
 * lsm303dlhc read x/y/z axis acceleration data.
 * @client: Handle to slave device.
 * @acc: to saving acceleration data.
 *
 * The value is expressed as two's complement. So, you need to convert it
 * to the original code.
 *  */
static int lsm303dlhc_read_acc_data(struct i2c_client *client,
		struct lsm303dlhc_acc *acc)
{
	unsigned char tmp_data[7], i;
	u16	temp[3];

	for (i=0; i<7; i++)
		lsm303dlhc_smbus_read_byte(client,ADDR_OUT_X_L_A+i, tmp_data+i);

	temp[0] = ((tmp_data[1] << 8) & 0xff00) | tmp_data[0];
	temp[1] = ((tmp_data[3] << 8) & 0xff00) | tmp_data[2];
	temp[2] = ((tmp_data[5] << 8) & 0xff00) | tmp_data[4];

	acc->x = lsm303dlhc_complement_to_original(temp[0]);
	acc->y = lsm303dlhc_complement_to_original(temp[1]);
	acc->z = lsm303dlhc_complement_to_original(temp[2]);

	return 0;
}

static ssize_t lsm303dlhc_reg_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char i, data;
	size_t count = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	for (i=ADDR_CTRL_REG1_A; i<=ADDR_TIME_WINDOW_A; i++) {
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
	if (check_value((unsigned char)addr, (unsigned char)value) < 0)
		return -EINVAL;

	if (lsm303dlhc_smbus_write_byte(lsm303dlhc->lsm303dlhc_client,
			(unsigned char)addr, (unsigned char *)&value) < 0)
		return -EINVAL;

	reg_update(dev, (unsigned char)addr, (unsigned char)value);
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

/*
 * the device is switched off. Enable is 1 to start the device, 0 is to
 * turn off the device. When the device is activated periodically read
 * the accelerometer data value.
 *  */
static void lsm303dlhc_do_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (enable) {
		lsm303dlhc_set_mode(lsm303dlhc->lsm303dlhc_client, MODE_ACTIVE);
		schedule_delayed_work(&lsm303dlhc->work,
			msecs_to_jiffies(atomic_read(&lsm303dlhc->delay)));
	} else {
		lsm303dlhc_set_mode(lsm303dlhc->lsm303dlhc_client, MODE_STANDBY);
		cancel_delayed_work_sync(&lsm303dlhc->work);
	}
}

/*
 * set the device on or off.
 * @dev: device.
 * @enable: ON(1) or OFF(0).
 *
 * When the new settings and the old settings are not equal, use the new
 * settings.
 *  */
static void lsm303dlhc_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);
	int pre_enable = atomic_read(&lsm303dlhc->enable);

	mutex_lock(&lsm303dlhc->enable_mutex);
	if (enable != pre_enable) {
		lsm303dlhc_do_enable(dev, enable);
		atomic_set(&lsm303dlhc->enable, enable);
	}
	mutex_unlock(&lsm303dlhc->enable_mutex);
}

static ssize_t lsm303dlhc_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	return sprintf(buf, "%d", atomic_read(&lsm303dlhc->enable));
}

static ssize_t lsm303dlhc_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if ((data == 0) || (data == 1))
		lsm303dlhc_set_enable(dev, data);
	return count;
}

static int get_sample(struct i2c_client *client)
{
	unsigned char data, mode;

	if (lsm303dlhc_smbus_read_bits(client, ADDR_CTRL_REG1_A, &data,
				4, 4) < 0)
		return -EAGAIN;

	if (lsm303dlhc_smbus_read_bits(client, ADDR_CTRL_REG1_A, &mode,
				3, 1) < 0)
		return -EAGAIN;

	if (data == 0)
		return -1;
	if (mode == 1) {
		if (data == 1)
			return 1;
		if (data == 2)
			return 10;
		if (data == 3)
			return 25;
		if (data == 4)
			return 50;
		if (data == 5)
			return 100;
		if (data == 6)
			return 200;
		if (data == 7)
			return 400;
		if (data == 8)
			return 1620;
		else
			return 5376;
	} else
		return 1344;
}

static ssize_t lsm303dlhc_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	return sprintf(buf, "%d", get_sample(lsm303dlhc->lsm303dlhc_client));
}

static ssize_t lsm303dlhc_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long rate;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &rate);
	if (err)
		return err;

	if (set_sample(lsm303dlhc->lsm303dlhc_client, rate) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_lowpower_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG1_A, &mode, 3, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", mode);
}

static int set_lowpower(struct i2c_client *client, unsigned long data)
{
	unsigned char setbits;

	setbits = (!!data) << 3;
	if (modify_reg(client, ADDR_CTRL_REG1_A, REG1_LOWPWR_BITS, setbits) < 0)
		return -1;

	setbits = ((data & 0x01) ? 0 : 1) << 3;
	if (modify_reg(client, ADDR_CTRL_REG4_A, REG4_HIGH_RESOLUTION_BITS,
				setbits) < 0)
		return -1;
	return 0;
}

static ssize_t lsm303dlhc_lowpower_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (set_lowpower(lsm303dlhc->lsm303dlhc_client, data) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_axis_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG1_A, &data, 0, 3) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_axis_store(struct device *dev,
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

	clearbits = REG1_AXIS_BITS;
	setbits = data & 0x07;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG1_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_highpass_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG2_A, &mode, 6, 2) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", mode);
}

static ssize_t lsm303dlhc_highpass_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned char clearbits, setbits;
	int err;
	unsigned long rate;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &rate);
	if (err)
		return err;

	clearbits = REG2_HIGHPASS_BITS;
	setbits = (rate & 0x03) << 6;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG2_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_highpass_freq_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG2_A, &data, 4, 2) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_highpass_freq_store(struct device *dev,
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

	clearbits = REG2_HIGHPASS_CUTOFF_BITS;
	setbits = (data & 0x03) << 4;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG2_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_filter_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG2_A, &data, 3, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_filter_store(struct device *dev,
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

	clearbits = REG2_FILTER_BITS;
	setbits = (data & 0x01) << 3;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG2_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_highpass_click_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG2_A, &data, 2, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_highpass_click_store(struct device *dev,
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

	clearbits = REG2_HPCLICK_BITS;
	setbits = (data & 0x01) << 2;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG2_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_highpass_irq2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG2_A, &data, 1, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_highpass_irq2_store(struct device *dev,
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

	clearbits = REG2_HPIS2_BITS;
	setbits = (data & 0x01) << 1;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG2_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_highpass_irq1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG2_A, &data, 0, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_highpass_irq1_store(struct device *dev,
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

	clearbits = REG2_HPIS1_BITS;
	setbits = data & 0x01;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG2_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_bdu_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG4_A, &data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_bdu_store(struct device *dev,
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

	clearbits = REG4_BDU_BITS;
	setbits = (data & 0x01) << 7;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG4_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_ble_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG4_A, &data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_ble_store(struct device *dev,
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

	clearbits = REG4_BLE_BITS;
	setbits = (data & 0x01) << 6;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG4_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static int get_range(struct i2c_client *client)
{
	unsigned char data;

	if (lsm303dlhc_smbus_read_bits(client, ADDR_CTRL_REG4_A,
				&data, 4, 2) < 0)
		return -EAGAIN;

	if (data == 0)
		return 2;
	if (data == 1)
		return 4;
	if (data == 2)
		return 8;

	return 16;
}

static ssize_t lsm303dlhc_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	return sprintf(buf, "%d", get_range(lsm303dlhc->lsm303dlhc_client));
}

static int set_range(struct i2c_client *client, unsigned long data)
{
	unsigned char clearbits, setbits;

	if (data == 2)
		data = 0;
	else if (data == 4)
		data = 1;
	else if (data == 8)
		data = 2;
	else if (data == 16)
		data = 3;
	else
		return -1;

	clearbits = REG4_FULLSCALE_BITS;
	setbits = (data & 0x03) << 4;

	if (modify_reg(client, ADDR_CTRL_REG4_A, clearbits, setbits) < 0)
		return -1;
	return 0;
}

static ssize_t lsm303dlhc_range_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	unsigned long data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	err = strict_strtoul(buf, 10, &data);
	if (err)
		return err;

	if (set_range(lsm303dlhc->lsm303dlhc_client, data) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_hr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG4_A, &data, 3, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_boot_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG5_A, &data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_boot_store(struct device *dev,
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

	clearbits = REG5_BOOT_BITS;
	setbits = (data & 0x01) << 7;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG5_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_fifo_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_CTRL_REG5_A, &data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_fifo_en_store(struct device *dev,
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

	clearbits = REG5_FIFO_EN_BITS;
	setbits = (data & 0x01) << 6;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_CTRL_REG5_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static ssize_t lsm303dlhc_zyxor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 7, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_zor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 6, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_yor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 5, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_xor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 4, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_zyxda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 3, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_zda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 2, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_yda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 1, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_xda_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_STATUS_REG_A, &data, 0, 1) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_acc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);
	static struct lsm303dlhc_acc acc;

	lsm303dlhc_read_acc_data(lsm303dlhc->lsm303dlhc_client, &acc);
	return sprintf(buf, "%d %d %d", acc.x, acc.y, acc.z);
}

static ssize_t lsm303dlhc_fifo_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data;
	struct i2c_client *client = to_i2c_client(dev);
	struct lsm303dlhc_data *lsm303dlhc = i2c_get_clientdata(client);

	if (lsm303dlhc_smbus_read_bits(lsm303dlhc->lsm303dlhc_client,
				ADDR_FIFO_CTRL_REG_A, &data, 6, 2) < 0)
		return sprintf(buf, "%d", -EAGAIN);
	return sprintf(buf, "%d", data);
}

static ssize_t lsm303dlhc_fifo_mode_store(struct device *dev,
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

	clearbits = REG_FIFO_MODE_BITS;
	setbits = (data & 0x03) << 6;

	if (modify_reg(lsm303dlhc->lsm303dlhc_client, ADDR_FIFO_CTRL_REG_A,
				clearbits, setbits) < 0)
		return -EAGAIN;
	return count;
}

static DEVICE_ATTR(reg, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_reg_show, lsm303dlhc_reg_store);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_delay_show, lsm303dlhc_delay_store);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_position_show, lsm303dlhc_position_store);
static DEVICE_ATTR(fuzz, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_fuzz_show, lsm303dlhc_fuzz_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_enable_show, lsm303dlhc_enable_store);
static DEVICE_ATTR(sample, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_sample_show, lsm303dlhc_sample_store);
static DEVICE_ATTR(lowpower, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_lowpower_show, lsm303dlhc_lowpower_store);
static DEVICE_ATTR(axis, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_axis_show, lsm303dlhc_axis_store);
static DEVICE_ATTR(highpass_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_highpass_mode_show, lsm303dlhc_highpass_mode_store);
static DEVICE_ATTR(highpass_freq, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_highpass_freq_show, lsm303dlhc_highpass_freq_store);
static DEVICE_ATTR(filter, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_filter_show, lsm303dlhc_filter_store);
static DEVICE_ATTR(highpass_click, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_highpass_click_show,lsm303dlhc_highpass_click_store);
static DEVICE_ATTR(highpass_irq2, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_highpass_irq2_show, lsm303dlhc_highpass_irq2_store);
static DEVICE_ATTR(highpass_irq1, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_highpass_irq1_show, lsm303dlhc_highpass_irq1_store);
static DEVICE_ATTR(bdu, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_bdu_show, lsm303dlhc_bdu_store);
static DEVICE_ATTR(ble, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_ble_show, lsm303dlhc_ble_store);
static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_range_show, lsm303dlhc_range_store);
static DEVICE_ATTR(hr, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_hr_show, NULL);
static DEVICE_ATTR(boot, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_boot_show, lsm303dlhc_boot_store);
static DEVICE_ATTR(fifo_en, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_fifo_en_show, lsm303dlhc_fifo_en_store);
static DEVICE_ATTR(zyxor, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_zyxor_show, NULL);
static DEVICE_ATTR(zor, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_zor_show, NULL);
static DEVICE_ATTR(yor, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_yor_show, NULL);
static DEVICE_ATTR(xor, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_xor_show, NULL);
static DEVICE_ATTR(zyxda, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_zyxda_show, NULL);
static DEVICE_ATTR(zda, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_zda_show, NULL);
static DEVICE_ATTR(yda, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_yda_show, NULL);
static DEVICE_ATTR(xda, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_xda_show, NULL);
static DEVICE_ATTR(acc, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_acc_show, NULL);
static DEVICE_ATTR(fifo_mode, S_IRUGO|S_IWUSR|S_IWGRP,
		lsm303dlhc_fifo_mode_show, lsm303dlhc_fifo_mode_store);

static struct attribute *lsm303dlhc_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_delay.attr,
	&dev_attr_position.attr,
	&dev_attr_fuzz.attr,
	&dev_attr_enable.attr,
	&dev_attr_sample.attr,
	&dev_attr_lowpower.attr,
	&dev_attr_axis.attr,
	&dev_attr_highpass_mode.attr,
	&dev_attr_highpass_freq.attr,
	&dev_attr_filter.attr,
	&dev_attr_highpass_click.attr,
	&dev_attr_highpass_irq2.attr,
	&dev_attr_highpass_irq1.attr,
	&dev_attr_bdu.attr,
	&dev_attr_ble.attr,
	&dev_attr_range.attr,
	&dev_attr_hr.attr,
	&dev_attr_boot.attr,
	&dev_attr_fifo_en.attr,
	&dev_attr_zyxor.attr,
	&dev_attr_zor.attr,
	&dev_attr_yor.attr,
	&dev_attr_xor.attr,
	&dev_attr_zyxda.attr,
	&dev_attr_zda.attr,
	&dev_attr_yda.attr,
	&dev_attr_xda.attr,
	&dev_attr_acc.attr,
	&dev_attr_fifo_mode.attr,
	NULL
};

static struct attribute_group lsm303dlhc_attribute_group = {
	.attrs = lsm303dlhc_attributes
};

/* periodically read the accelerometer data and reported. */
static void lsm303dlhc_work_func(struct work_struct *work)
{
	int result;
	struct lsm303dlhc_data *lsm303dlhc = container_of((struct delayed_work *)work,
				struct lsm303dlhc_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&lsm303dlhc->delay));
	struct lsm303dlhc_acc acc;

	result = lsm303dlhc_read_acc_data(lsm303dlhc->lsm303dlhc_client, &acc);
	if (result == 0) {
		input_report_abs(lsm303dlhc->input, ABS_X, acc.x);
		input_report_abs(lsm303dlhc->input, ABS_Y, acc.y);
		input_report_abs(lsm303dlhc->input, ABS_Z, acc.z);
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
		printk(KERN_ERR "lsm303dlhc accelerometer hardware init fail\n");
		goto kfree_exit;
	}

	dev = input_allocate_device();
	if (!dev) {
		printk(KERN_ERR "lsm303dlhc accelerometer input_allocate_device fail\n");
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
		printk(KERN_INFO "lsm303dlhc accelerometer input_register_device fail\n");
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

	lsm303dlhc_set_enable(&client->dev, 1);
	dev_info(&data->lsm303dlhc_client->dev,
			"Successfully initialized lsm303dlhc accelerometer!\n");
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

	lsm303dlhc_set_enable(&client->dev, 0);
	sysfs_remove_group(&data->input->dev.kobj, &lsm303dlhc_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static int lsm303dlhc_suspend(struct i2c_client *client, pm_message_t state)
{
	lsm303dlhc_do_enable(&client->dev, 0);

	return 0;
}

static int lsm303dlhc_resume(struct i2c_client *client)
{
	struct lsm303dlhc_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	lsm303dlhc_hw_init(data->lsm303dlhc_client);
	lsm303dlhc_do_enable(dev, atomic_read(&data->enable));

	return 0;
}

static const struct i2c_device_id lsm303dlhc_acc_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lsm303dlhc_acc_id);

static struct i2c_driver lsm303dlhc_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= SENSOR_NAME,
	},
	.class			= I2C_CLASS_HWMON,
	.id_table		= lsm303dlhc_acc_id,
	.probe			= lsm303dlhc_probe,
	.remove			= __devexit_p(lsm303dlhc_remove),
	.suspend 		= lsm303dlhc_suspend,
	.resume  		= lsm303dlhc_resume,
};

static int __init lsm303dlhc_acc_init(void)
{
	return i2c_add_driver(&lsm303dlhc_driver);
}

static void __exit lsm303dlhc_acc_exit(void)
{
	i2c_del_driver(&lsm303dlhc_driver);
}

MODULE_AUTHOR("Sarlor <kinsleer@outlook.com>");
MODULE_DESCRIPTION("LSM303DLHC Accessible Driver");
MODULE_LICENSE("GPL");

module_init(lsm303dlhc_acc_init);
module_exit(lsm303dlhc_acc_exit);
