#ifndef _LSM303DLHC_H_
#define _LSM303DLHC_H_

/* LSM303DLHC 3D accelerometer and 3D magnetometer module
 *
 * Attributes of accelerometer module
 * interface		RW status	Description		I/O format
 * reg			RW		Register interface	"0x%02x:0x%02x"
 * delay		RW		Delay interface		"%d"
 * position		RW		Don't known		"%d"
 * fuzz			RW		Don't known		"%d"
 * enable		RW		Power mode enable	"%d"
 * sample		RW		Output date rate	"%d"
 * lowpower		RW		low-power mode enable 	"%d"
 * axis			RW		Z/Y/X axis enable	"%d"
 * highpass_mode	RW		High-pass mode		"%d"
 * highpass_freq	RW		High-pass cutoff freq	"%d"
 * filter		RW		filtered data selection "%d"
 * highpass_click	RW		highpass filter enabled for CLICK "%d"
 * highpass_irq2	RW					Highpass filter enabled for IRQ2	"%d"
 * highpass_irq1	RW					Highpass filter enabled for IRQ1	"%d"
 * bdu				RW					Block data update					"%d"
 * ble				RW					Big/little endian data selection 	"%d"
 * range			RW					Full scale selection				"%d"
 * hr				RW					High resolution output mode			"%d"
 * boot				RW					Reboot memory content				"%d"
 * fifo_en			RW					FIFO enable							"%d"
 * zyxor			Read Only			X/Y/Z axis data overrun				"%d"
 * zor				Read Only			Z axis data overrun					"%d"
 * yor				Read Only			Y axis data overrun					"%d"
 * xor				Read Only			X axis data overrun					"%d"
 * zyxda			Read Only			X/Y/Z axis has new data				"%d"
 * zda				Read Only			Z axis has new data					"%d"
 * yda				Read Only			Y axis has new data					"%d"
 * xda				Read Only			X axis has new data					"%d"
 * acc 				Read Only			accelerometer data 					"%d %d %d"
 * fifo_mode		RW 					FIFO mode selection					"%d"
 *  */
/* Attributes of magnetometer module
 * interface		RW status	Description		I/O format
 * reg			RW		Register interface	"0x%02x:0x%02x"
 * delay			RW					Delay interface						"%d"
 * position			RW					Don't known							"%d"
 * fuzz				RW					Don't known							"%d"
 * temp_en			RW					Temperature sensor enable			"%d"
 * sample			RW					Data output rate					"%d"
 * range			RW					Full scale selection				"%d"
 * mode				RW					Mode select							"%d"
 * mag				Read Only			magnetometer data					"%d %d %d"
 * lock				Read Only			Data output register lock 			"%d"
 * drdy				Read Only			Data ready 							"%d"
 * temperature		Read Only			Temperature data					"%d"
 *   */
#define LSM303DLHC_ACC_PATHBASE	"/sys/devices/virtual/input/input2"
#define LSM303DLHC_MAG_PATHBASE	"/sys/devices/virtual/input/input3"

/* Acceleration config */
/* Power-down and data rate selection in low-power mode */
#define LSM303DLHC_ACC_POWERDOWN				0
#define LSM303DLHC_ACC_RATE_1HZ					1
#define LSM303DLHC_ACC_RATE_10HZ				10
#define LSM303DLHC_ACC_RATE_25HZ				25
#define LSM303DLHC_ACC_RATE_50HZ				50
#define LSM303DLHC_ACC_RATE_100HZ				100
#define LSM303DLHC_ACC_RATE_200HZ				200
#define LSM303DLHC_ACC_RATE_400HZ				400
#define LSM303DLHC_ACC_RATE_1620HZ				1620
#define LSM303DLHC_ACC_RATE_5376HZ				5376
/* Data rate selection in normal mode */
#define LSM303DLHC_ACC_RATE_1344HZ				1344

/* Low-power mode enable */
#define LSM303DLHC_ACC_LOWPOWER_DIS				0
#define LSM303DLHC_ACC_LOWPOWER_EN				1

/* Z/Y/Z axis enable */
#define LSM303DLHC_ACC_ZYZ_DIS					0
#define LSM303DLHC_ACC_ZYZ_EN					1

/* High pass filter mode selection */
#define LSM303DLHC_ACC_HIGHPASS_MODE_NOR_REST		0 /* Normal mode (reset) */
#define LSM303DLHC_ACC_HIGHPASS_MODE_REF		1 /* Reference signal */
#define LSM303DLHC_ACC_HIGHPASS_MODE_NOR		2 /* Normal mode */
#define LSM303DLHC_ACC_HIGHPASS_MODE_AUTO		3 /* Autoreset on irq event */

/* Filtered data selection */
#define LSM303DLHC_ACC_FILTER_BYPASS			0 /* Internal filter bypass */
#define LSM303DLHC_ACC_FILTER_OUTPUT			1 /* Data from intel filter */

/* High pass filter enabled for CLICK function */
#define LSM303DLHC_ACC_HIGHPASS_CLICK_BYPASS		0
#define LSM303DLHC_ACC_HIGHPASS_CLICK_EN		1

/* High pass filter enabled for AOI function on interrupt 2 */
#define LSM303DLHC_ACC_HIGHPASS_IRQ2_BYPASS		0
#define LSM303DLHC_ACC_HIGHPASS_IRQ2_EN			1

/* High pass filter enabled for AOI function on interrupt 1 */
#define LSM303DLHC_ACC_HIGHPASS_IRQ1_BYPASS		0
#define LSM303DLHC_ACC_HIGHPASS_IRQ1_EN			1

/* Block data update */
#define LSM303DLHC_ACC_BDU_UPDATE			0 /* Continuos update */
#define LSM303DLHC_ACC_BDU_NOT_UPDATE			1 /* Output registers not update until MSB and LSB reading */

/* Big/little endian data selection */
#define LSM303DLHC_ACC_BLE_LSB					0
#define LSM303DLHC_ACC_BLE_MSB					1

/* Full-scale selection */
#define LSM303DLHC_ACC_RANGE_2G					2
#define LSM303DLHC_ACC_RANGE_4G					4
#define LSM303DLHC_ACC_RANGE_8G					8
#define LSM303DLHC_ACC_RANGE_16G				16

/* High resolution output mode */
#define LSM303DLHC_ACC_HR_DIS					0
#define LSM303DLHC_ACC_HR_EN					1

/* Reboot memory content */
#define LSM303DLHC_ACC_REBOOT_NOR				0
#define LSM303DLHC_ACC_REBOOT_EN				1

/* FIFO enable */
#define LSM303DLHC_ACC_FIFO_DIS					0
#define LSM303DLHC_ACC_FIFO_EN					1

/* X/Y/Z axis data overrun */
#define LSM303DLHC_ACC_NO_XYZDATA_OVERRUN			0
#define LSM303DLHC_ACC_HAVE_XYZDATA_OVERRUN			1

/* X/Y/Z axis new data available */
#define LSM303DLHC_ACC_NO_ZYXDATA_AVAILABLE			0
#define LSM303DLHC_ACC_HAVA_ZYXDATA_AVAILABLE			1

/* FIFO mode selection */
#define LSM303DLHC_ACC_FIFO_BYPASS_MODE				0
#define LSM303DLHC_ACC_FIFO_FIFO_MODE				1
#define LSM303DLHC_ACC_FIFO_STREAM_MODE				2
#define LSM303DLHC_ACC_FIFO_TRIGGER_MODE			3

/* Temperature sensor enable */
#define LSM303DLHC_MAG_TEMP_DIS					0
#define LSM303DLHC_MAG_TEMP_EN					1

/* Data output rate bits */
#define LSM303DLHC_MAG_RATE_0_75HZ				0
#define LSM303DLHC_MAG_RATE_1_5HZ				1
#define LSM303DLHC_MAG_RATE_3HZ					2
#define LSM303DLHC_MAG_RATE_7_5HZ				3
#define LSM303DLHC_MAG_RATE_15HZ				4
#define LSM303DLHC_MAG_RATE_30HZ				5
#define LSM303DLHC_MAG_RATE_75HZ				6
#define LSM303DLHC_MAG_RATE_220HZ				7

/* Gain configuration bits. The gain configuration is common for all channels */
#define LSM303DLHC_MAG_GRANGE_1_3G				1
#define LSM303DLHC_MAG_GRANGE_1_9G				2
#define LSM303DLHC_MAG_GRANGE_2_5G				3
#define LSM303DLHC_MAG_GRANGE_4G				4
#define LSM303DLHC_MAG_GRANGE_4_7G				5
#define LSM303DLHC_MAG_GRANGE_5_6G				6
#define LSM303DLHC_MAG_GRANGE_8_1G				7

/* Mode select bits. These bits select the operation mode of this device */
#define LSM303DLHC_MAG_MODE_CON					0
#define LSM303DLHC_MAG_MODE_SINGLE				1
#define LSM303DLHC_MAG_MODE_SLEEP				2

/* Data output register lock */
#define LSM303DLHC_MAG_NOTLOCK					0
#define LSM303DLHC_MAG_LOCK					1

/* Data ready bit. This bit is when a new set of measurements are available */
#define LSM303DLHC_MAG_NOTDRDY					0
#define LSM303DLHC_MAG_DRDY					1

#define BUF_MAX							128

/* Accelerome Magnetometer data */
struct axis_data {
	double x;
	double y;
	double z;
};

struct lsm303dlhc {
	int fd_data;
	int range;
	int delay;
	int sample;
	double lsb;
	struct axis_data data;
	struct axis_data offset;
	struct axis_data standData;
};

#endif /* _LSM303DLHC_H_ */
