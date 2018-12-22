简介:
	LSM303DLHC驱动由两部分组成，分别是acceleration和magnetometer.
	Acceleration的接口基路径是：LSM303DLHC_ACC_PATH_BASE， 其值为"/sys/devices/virtual/input/input2/"
	Magnetometer的接口基路径是：#define LSM303DLHC_MAG_PATH_BASE, 其值为"/sys/devices/virtual/input/input3/"
	
默认配置:
	Acceleration:
		1. 电源模式：正常
		2. Z/Y/X轴启动
		3. 开启数据收发准备状态在INT1
		4. BDU为1 (输出数据不更新, 直到MSB和LSB被读)
		5. 设置Rate为1344HZ(在低电模式下为400HZ)
		6. 设置Range为8G
		7. HR启用(高分辨率输出模式启用)
	Magnetometer:
		1. 温度传感器启动
		2. 设置Rate为75HZ
		3. Range为1.9G
		4. 传感器模式为Continuous-conversion模式
	
配置宏:
	包含 #include "lsm303dlhc.h"

	#define LSM303DLHC_ACC_PATH_BASE				Acceleration基路径	
	#define LSM303DLHC_MAG_PATH_BASE				Magnetometer基路径

	注意：
		Acceleration的数据输出频率分为普通模式和低电模式， 普通模式的频率固定位1344HZ
		低电模式则为下面的(1HZ, 10HZ, 25HZ, ..., 5376HZ).
		低电模式的设置方法为：lowpower为启动状态，HR为禁止状态
		普通模式的设置方法为：lowpower为禁用状态，HR为启用状态
		低电模式只需要设置lowpower, 其中会自动设置HR的状态，无需手动设置
		
	/****************************** Acceleration ****************************************/
	#define LSM303DLHC_ACC_POWERDOWN				启用掉电模式
	#define LSM303DLHC_ACC_RATE_1HZ					掉电模式下的频率为1HZ
	#define LSM303DLHC_ACC_RATE_10HZ				掉电模式下的频率为10HZ
	#define LSM303DLHC_ACC_RATE_25HZ				掉电模式下的频率为25HZ	
	#define LSM303DLHC_ACC_RATE_50HZ				掉电模式下的频率为50HZ
	#define LSM303DLHC_ACC_RATE_100HZ				掉电模式下的频率为100HZ
	#define LSM303DLHC_ACC_RATE_200HZ				掉电模式下的频率为200HZ
	#define LSM303DLHC_ACC_RATE_400HZ				掉电模式下的频率为400HZ
	#define LSM303DLHC_ACC_RATE_1620HZ				掉电模式下的频率为1620HZ
	#define LSM303DLHC_ACC_RATE_5376HZ				掉电模式下的频率为5376HZ

	#define LSM303DLHC_ACC_RATE_1344HZ				普通模式下的频率为1344HZ

	#define LSM303DLHC_ACC_LOWPOWER_DIS				禁用低电模式
	#define LSM303DLHC_ACC_LOWPOWER_EN				启用低电模式

	#define LSM303DLHC_ACC_ZYZ_DIS					Z/Y/X轴禁用
	#define LSM303DLHC_ACC_ZYZ_EN					Z/Y/X轴启动

	#define LSM303DLHC_ACC_HIGHPASS_MODE_NOR_REST	高通滤波模式为正常模式(重置)
	#define LSM303DLHC_ACC_HIGHPASS_MODE_REF		高通滤波模式为参考信号滤波模式
	#define LSM303DLHC_ACC_HIGHPASS_MODE_NOR		高通滤波模式为正常模式
	#define LSM303DLHC_ACC_HIGHPASS_MODE_AUTO		高通滤波模式为在中断时间时自动重置

	#define LSM303DLHC_ACC_FILTER_BYPASS			过滤数据选择为内部过滤器旁路
	#define LSM303DLHC_ACC_FILTER_OUTPUT			过滤数据选择为数据从内部滤波器发送到输出寄存器和FIFO

	#define LSM303DLHC_ACC_HIGHPASS_CLICK_BYPASS	高通滤波启用为了CLICK的过滤器旁路
	#define LSM303DLHC_ACC_HIGHPASS_CLICK_EN		高通滤波启用为了CLICK的过滤功能

	#define LSM303DLHC_ACC_HIGHPASS_IRQ2_BYPASS		对IRQ2的AOI功能启用高通滤波器的旁路功能
	#define LSM303DLHC_ACC_HIGHPASS_IRQ2_EN			对IRQ2的AOI功能启用高通滤波

	#define LSM303DLHC_ACC_HIGHPASS_IRQ1_BYPASS		对IRQ1的AOI功能启用高通滤波器的旁路功能
	#define LSM303DLHC_ACC_HIGHPASS_IRQ1_EN			对IRQ1的AOI功能启用高通滤波

	#define LSM303DLHC_ACC_BDU_UPDATE				数据块连续更新
	#define LSM303DLHC_ACC_BDU_NOT_UPDATE			数据块输出到寄存器不更新， 直到MSB和LSM被读

	#define LSM303DLHC_ACC_BLE_LSB					LSB模式
	#define LSM303DLHC_ACC_BLE_MSB					MSB模式

	#define LSM303DLHC_ACC_RANGE_2G					满量程选择2G
	#define LSM303DLHC_ACC_RANGE_4G					满量程选择4G
	#define LSM303DLHC_ACC_RANGE_8G					满量程选择8G
	#define LSM303DLHC_ACC_RANGE_16G				满量程选择16G
	
	#define LSM303DLHC_ACC_HR_DIS					高分辨率输出模式禁用
	#define LSM303DLHC_ACC_HR_EN					高分辨率输出模式启用

	#define LSM303DLHC_ACC_REBOOT_NOR				内存内容为正常模式
	#define LSM303DLHC_ACC_REBOOT_EN				重新启动内存内容

	#define LSM303DLHC_ACC_FIFO_DIS					FIFO禁止
	#define LSM303DLHC_ACC_FIFO_EN					FIFO启用

	#define LSM303DLHC_ACC_NO_XYZDATA_OVERRUN		X/Y/Z轴数据没有溢出
	#define LSM303DLHC_ACC_HAVE_XYZDATA_OVERRUN		X/Y/Z轴数据溢出

	#define LSM303DLHC_ACC_NO_ZYXDATA_AVAILABLE		X/Y/Z轴没有新的可用数据
	#define LSM303DLHC_ACC_HAVA_ZYXDATA_AVAILABLE	X/Y/Z轴有新的可用的数据

	#define LSM303DLHC_ACC_FIFO_BYPASS_MODE			FIFO模式为旁路模式
	#define LSM303DLHC_ACC_FIFO_FIFO_MODE			FIFO模式为FIFO模式
	#define LSM303DLHC_ACC_FIFO_STREAM_MODE			FIFO模式为流模式
	#define LSM303DLHC_ACC_FIFO_TRIGGER_MODE		FIFO模式为触发模式
	
操作接口:
	reg					寄存器接口, 可查看所有寄存器的值和修改特定寄存器的值
						输入输出格式: "0x%02x:0x%02x" 				(例如: "0x02:0xef")
						read(L3G_GET_PATH(reg), buf, 100);			读取全部寄存器的值
																	各个值之间以空格分割(注意: buf不能太小)
						write(L3G_GET_PATH(reg), "0x02:0xef", 10);	修改寄存器0x02值为0xef 
	
	delay				延迟接口				
						输入输出格式: "%d"							(例如: "50")
			
	position			三轴加速度偏移值
						输入输出格式: "%d", 默认值为-3
	
	fuzz				未知
	
	enable				三轴加速度电源管理
						输入输出格式: "%d", 输入输出值为(0000 - 1001)
						0000: 断电, 其他值: 启用
	
	sample				获取/设置输出数据速率
						输入输出格式: "%d", 输入输出值为(95HZ, 190HZ, 380HZ, 760HZ)	
		
	lowpower			获取/设置低电模式
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 普通模式, 1: 低电模式
						
	axis				启动/禁用X/Y/Z轴
						输入输出格式： "%d", 输入输出值为(0-7)
						0: X/Y/Z全部禁止, 1: X轴使能, 2: Y轴使能, 4: Z轴使能 (可以使用或运算)
	
	highpass_mode		获取/设置高通滤波模式选择
						输入输出格式: "%d",  输入输出值为(0-3) 
						0: 一般模式(重置), 1: 参考滤波信号, 2: 普通模式, 3: 中断事件自动重启
						
	highpass_freq		获取/设置高通滤波频率
	
	filter				获取/设置过滤数据选择
						输入输出格式："%d", 输入输出值为(0-1)
						0: 内部过滤器旁路, 1: 数据从内部滤波器发送到输出寄存器和FIFO
	
	highpass_click		获取/设置启用了CLICK功能的高通滤波器
						输入输出格式： "%d", 输入输出值为(0-1)
						0: 过滤器旁路, 1: 过滤器启用
	
	highpass_irq2		获取/设置对中断2的AOI功能启用高通滤波器
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 过滤器旁路, 1: 过滤器启用
	
	highpass_irq1		获取/设置对中断1的AOI功能启用高通滤波器
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 过滤器旁路, 1: 过滤器启用
	
	bdu					获取/设置快数据更新
						输入输出格式: "%d",  输入输出值为(0-1)
						0： 连续更新, 1: 输出寄存器不更新
	
	ble					获取/设置大小端数据选择
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 小端, 		1: 大端
	
	range				获取/设置量程选择
						输入输出格式： "%d", 输入输出值为(2G, 4G, 8G, 16G)
	
	hr					获取/设置高分辨率输出模式
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 禁止,		1: 启用
	
	boot				获取/设置重启内存内容状态
						输入输出格式: "%d", 输入输出值为(0-1)
						0: 正常模式, 	1: 重启内存内容
	
	fifo_en				获得/设置FIFO使能
						输入输出格式: "%d",  输入输出值为(0-1)
						0: 禁止, 1: 使能
	
	zyxor				获取X/Y/Z轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
	
	zor					获取Z轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
	
	yor					获取Y轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
						
	xor					获取X轴数据是否溢出
						输出格式: "%d",  输出值为(0-1)
						0: 没有数据溢出, 1: 溢出
						
	zyxda				获取X/Y/Z轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	zda					获取Z轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	yda					获取Y轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
						
	xda					获取X轴数据是否有新数据可用
						输出格式: "%d",  输出值为(0-1)
						0: 没有可用数据, 1: 有
	
	acc					获取三轴加速度的数据值
						输出格式: "%d %d %d"
		
	fifo_mode			获取/设置FIFO模式
						输入输出格式: "%d", 输入输出值为(0-3)
						0: Bypass mode, 1: FIFO mode, 2: Stream mode 4: Trigger mode						
						
	/**************************** Magnetometer ************************************/
	#define LSM303DLHC_MAG_TEMP_DIS					温度传感器禁用
	#define LSM303DLHC_MAG_TEMP_EN					温度传感器启用

	#define LSM303DLHC_MAG_RATE_0_75HZ				频率为0.75HZ
	#define LSM303DLHC_MAG_RATE_1_5HZ				频率为1.5HZ
	#define LSM303DLHC_MAG_RATE_3HZ					频率为3HZ
	#define LSM303DLHC_MAG_RATE_7_5HZ				频率为7.5HZ
	#define LSM303DLHC_MAG_RATE_15HZ				频率为15HZ
	#define LSM303DLHC_MAG_RATE_30HZ				频率为30HZ
	#define LSM303DLHC_MAG_RATE_75HZ				频率为75HZ
	#define LSM303DLHC_MAG_RATE_220HZ				频率为220HZ

	#define LSM303DLHC_MAG_GRANGE_1_3G				Range的范围为1.3G
	#define LSM303DLHC_MAG_GRANGE_1_9G				Range的范围为1.9G
	#define LSM303DLHC_MAG_GRANGE_2_5G				Range的范围为2.5G
	#define LSM303DLHC_MAG_GRANGE_4G				Range的范围为4G
	#define LSM303DLHC_MAG_GRANGE_4_7G				Range的范围为4.7G
	#define LSM303DLHC_MAG_GRANGE_5_6G				Range的范围为5.6G
	#define LSM303DLHC_MAG_GRANGE_8_1G				Range的范围为8.1G

	#define LSM303DLHC_MAG_MODE_CON					模式为连续转换模式
	#define LSM303DLHC_MAG_MODE_SINGLE				模式为单转欢模式
	#define LSM303DLHC_MAG_MODE_SLEEP				模式为休眠模式 (值为2)
	#define LSM303DLHC_MAG_MODE_SLEEP				模式为休眠模式 (值为3)

	#define LSM303DLHC_MAG_NOTLOCK					数据输出寄存器未上锁
	#define LSM303DLHC_MAG_LOCK						数据输出寄存器已上锁

	#define LSM303DLHC_MAG_NOTDRDY					没有可用的数据
	#define LSM303DLHC_MAG_DRDY						有可用的新数据
	
操作接口:
	reg					寄存器接口, 可查看所有寄存器的值和修改特定寄存器的值
						输入输出格式: "0x%02x:0x%02x" 				(例如: "0x02:0xef")
						read(L3G_GET_PATH(reg), buf, 100);			读取全部寄存器的值
																	各个值之间以空格分割(注意: buf不能太小)
						write(L3G_GET_PATH(reg), "0x02:0xef", 10);	修改寄存器0x02值为0xef 
	
	delay				延迟接口				
						输入输出格式: "%d"							(例如: "50")
			
	position			三轴加速度偏移值
						输入输出格式: "%d", 默认值为-3
	
	fuzz				未知
	
	temp_en				获取/设置温度传感器的状态
						输入输出格式: "%d", 输入输出值为(0-1)
	
	sample				获取/设置输出数据速率
						输入输出格式: "%d", 输入输出值为(0-7)
						0: 0.75HZ, 1: 1.5HZ, 2: 3HZ,  3: 7.5HZ
						4: 15HZ,   5: 30HZ,  6: 75HZ, 7: 220HZ

	range				获取/设置量程选择
						输入输出格式: "%d", 输入输出值为(1-7)
						1: 1.3G,	2: 1.9G,	3: 2.5G,	4: 4G
						5: 4.7G,	6: 5.6G,	7: 8.1G
	
	mode				获取/设置磁力传感器模式状态
						输入输出格式: "%d", 输入输出值为(1-3)
						0: 连续转换模式		1: 单转换模式
						2: 休眠模式			3: 休眠模式
	
	mag					获取磁力传感器的数据值
						输出格式: "%d %d %d"
	
	lock				获取数据输出寄存器锁定的状态
						输出格式: "%d", 输出值为(0-1)
						0: 未锁,			1: 已锁
	
	drdy				获取数据就绪状态
						输出格式: "%d", 输出值为(0-1)
						0: 无有效数据,			1: 有数据
	
	temperature			获取温度传感器数据值
						输出格式: "%d"