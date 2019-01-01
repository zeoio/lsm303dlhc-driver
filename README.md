### LSM303DLHC 3D Accelerometer and 3D Magnetometer Driver

LSM303DLHC Ultra Compact High Perfomance E-compass 3D Accelerometer and 3D Magnetometer module driver.

1. First developed LSM303DLHC Accelerometer driver.

1. Frist enter the 'src' directory.
        ===> $: cd src/acc
2. Edit the 'Makefile' file, set the correct kernel directory 'KERN_DIR'.
        ===> $: vim Makefile
3. Copy the driver files to the 'hwmon' directory.
        ===> $: make cp
4. Add board-level support code.
        ===> $: vim arch/arm/plat-s5p4418/realarm/device.c
5. Add the code in the appropriate place.
        CODE 1:

                #if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || defined(CONFIG_SENSORS_LSM303DLHC_ACC_MODULE)
                #include <linux/i2c.h>

                /* CODEC */
                static struct i2c_board_info __initdata lsm303dlhc_acc_i2c_bdi = {
                        .type   = "lsm303dlhc_acc",
                        .addr   = 0x19
                };

                #endif

        CODE 2:

                #if defined(CONFIG_SENSORS_LSM303DLHC_ACC) || defined(CONFIG_SENSORS_LSM303DLHC_ACC_MODULE)
                        printk("plat: add accelerometer lsm303dlhc\n");
                        i2c_register_board_info(1, &lsm303dlhc_acc_i2c_bdi, 1);
                #endif
      
6. Modify the kernel configuration file for lsm303dlhc accelerometer.
        ===> $: vim drivers/hwmon/Kconfig
        ADD CODE:

                config SENSORS_LSM303DLHC_ACC
                        tristate "3D accelerometer module"
                        depends on I2C
                        default n
                        help
                                Say Y here to enable the Accelerometer module.

                                If unsure, say N.

        ===> $: vim drivers/hwmon/Makefile
        ADD CODE:

                obj-$(CONFIG_SENSORS_LSM303DLHC_ACC)    += lsm303dlhc_acc.o

2. Second developed LSM303DLHC magnetometer driver.

(1). Frist enter the 'src' directory.
        ===> $: cd src/mag
(2). Edit the 'Makefile' file, set the correct kernel directory 'KERN_DIR'.
        ===> $: vim Makefile
(3). Copy the driver files to the 'hwmon' directory.
        ===> $: make cp
(4). Add board-level support code.
        ===> $: vim arch/arm/plat-s5p4418/realarm/device.c
(5). Add the code in the appropriate place.
        CODE 1:
       #if defined(CONFIG_SENSORS_LSM303DLHC_MAG) || defined(CONFIG_SENSORS_LSM303DLHC_MAG_MODULE)
                #include <linux/i2c.h>

                /* CODEC */
                static struct i2c_board_info __initdata lsm303dlhc_mag_i2c_bdi = {
                        .type   = "lsm303dlhc_mag",
                        .addr   = 0x1E
                };

                #endif

        CODE 2:

                #if defined(CONFIG_SENSORS_LSM303DLHC_MAG) || defined(CONFIG_SENSORS_LSM303DLHC_MAG_MODULE)
                        printk("plat: add magnetometer lsm303dlhc\n");
                        i2c_register_board_info(1, &lsm303dlhc_mag_i2c_bdi, 1);
                #endif

(6). Modify the kernel configuration file for lsm303dlhc accelerometer.
        ===> $: vim drivers/hwmon/Kconfig
        ADD CODE:

        config SENSORS_LSM303DLHC_MAG
                tristate "3D magnetometer module"
                depends on I2C
                default n
                help
                        Say Y here to enable the Magnetometer module.

                        If unsure, say N.

        ===> $: vim drivers/hwmon/Makefile
        ADD CODE:

                obj-$(CONFIG_SENSORS_LSM303DLHC_MAG)    += lsm303dlhc_mag.o

3. Configure the kernel to use this driver.
        ===> $: make menuconfig
                Device Drivers -> Hardware Monitoring support -> 3D accelerometer module && 3D magnetometer module

        Check, save.
