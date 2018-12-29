obj-m = lsm303dlhc_acc.o
KERN_DIR = ~/Documents/Sensor/kernel-3.4.39
PWD = $(shell pwd)

all:
	make -C $(KERN_DIR) SUBDIRS=$(PWD) modules

.PHONY: clean cp
clean:
	rm *.o *.ko *.symvers *.order *.mod.c
cp:
	cp lsm303dlhc_acc.c $(KERN_DIR)/drivers/hwmon
