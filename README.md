### LSM303DLHC 3D Accelerometer and 3D Magnetometer Driver

LSM303DLHC Ultra Compact High Perfomance E-compass 3D Accelerometer and 3D Magnetometer module driver.

#### Building the source

##### Developed LSM303DLHC accelerometer driver
1. Clone the code
```bash
git clone https://github.com/Sarlor/lsm303dlhc-driver.git
cd lsm303dlhc-driver/src/acc
```

2. Edit the 'Makefile' file, set the correct kernel directory 'KERN_DIR'
```bash
vim Makefile
```

3. Copy the driver files to the 'hwmon' directory
```bash
make cp
```

4. Add board-level support code to the  appropriate place
```bash
vim arch/arm/plat-s5p4418/realarm/device.c
```

```c
#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || defined(CONFIG_SENSORS_LSM303DLHC_ACC_MODULE)
#include <linux/i2c.h>

/* CODEC */
static struct i2c_board_info __initdata lsm303dlhc_acc_i2c_bdi = {
	.type   = "lsm303dlhc_acc",
	.addr   = 0x19
 };

 #endif

#if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || defined(CONFIG_SENSORS_LSM303DLHC_ACC_MODULE)
	printk("plat: add accelerometer lsm303dlhc\n");
	i2c_register_board_info(1, &lsm303dlhc_acc_i2c_bdi, 1);
#endif
```

5. Modify the kernel configuration file for lsm303dlhc accelerometer
```bash
vim drivers/hwmon/Kconfig
```

```
config SENSORS_LSM303DLHC_ACC
	tristate "3D accelerometer module"
	depends on I2C
	default n
	help
	Say Y here to enable the Accelerometer module.
	
	If unsure, say N.
```

```bash
vim drivers/hwmon/Makefile
```

```
obj-$(CONFIG_SENSORS_LSM303DLHC_ACC)    += lsm303dlhc_acc.o
```

#### Developed LSM303DLHC magnetometer driver

1. Enter the src directory, and edit Makefile file, set the correct kernel directory 'KERN_DIR'
```bash
cd src/mag
vim Makefile
```

2. Copy the driver files to the 'hwmon' directory
```bash
make cp
```

3. Add board-level support code to the appropriate place
```bash
vim arch/arm/plat-s5p4418/realarm/device.c
```

```c
#if defined(CONFIG_SENSORS_LSM303DLHC_MAG) || defined(CONFIG_SENSORS_LSM303DLHC_MAG_MODULE)
#include <linux/i2c.h>

/* CODEC */
static struct i2c_board_info __initdata lsm303dlhc_mag_i2c_bdi = {
	.type   = "lsm303dlhc_mag",
	.addr   = 0x1E
};

#endif

#if defined(CONFIG_SENSORS_LSM303DLHC_MAG) || defined(CONFIG_SENSORS_LSM303DLHC_MAG_MODULE)
	printk("plat: add magnetometer lsm303dlhc\n");
	i2c_register_board_info(1, &lsm303dlhc_mag_i2c_bdi, 1);
#endif
```

4. Modify the kernel configuration file for lsm303dlhc accelerometer
```bash
vim drivers/hwmon/Kconfig
```

```
config SENSORS_LSM303DLHC_MAG
	tristate "3D magnetometer module"
	depends on I2C
	default n
	help
	Say Y here to enable the Magnetometer module.

	If unsure, say N.
```

```bash
vim drivers/hwmon/Makefile
```

```
obj-$(CONFIG_SENSORS_LSM303DLHC_MAG)    += lsm303dlhc_mag.o
```

#### Configure  kernel
```bash
make menuconfig
```

```
Device Drivers -> Hardware Monitoring support -> 3D accelerometer module && 3D magnetometer module
```
