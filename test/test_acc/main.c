#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include "ops.h"
#include "lsm303dlhc.h"

int dev_init(struct lsm303dlhc *);
void get_lsb(const int, double *);
void get_offset(struct lsm303dlhc *);
int read_acc(const char *, const char *, struct axis_data *);
int test_data(struct lsm303dlhc *);
int dev_close(struct lsm303dlhc *);

int main(void)
{
	unsigned i;
	struct lsm303dlhc lsm303dlhc;

	dev_init(&lsm303dlhc);

	for (i=0; i<10000; i++) {
		test_data(&lsm303dlhc);
		show_data(&lsm303dlhc);
		usleep(2800);
	}

	dev_close(&lsm303dlhc);
	return 0;
}

int dev_init(struct lsm303dlhc *lsm303dlhc)
{
	set_ops(LSM303DLHC_ACC_PATHBASE, "sample", 400);
	set_ops(LSM303DLHC_ACC_PATHBASE, "enable", 1);

	memset(lsm303dlhc, 0, sizeof(*lsm303dlhc));

	lsm303dlhc->standData.x = 0;
	lsm303dlhc->standData.y = 0;
	lsm303dlhc->standData.z = 1;

	get_ops(LSM303DLHC_ACC_PATHBASE, "range", &lsm303dlhc->range);
	get_ops(LSM303DLHC_ACC_PATHBASE, "delay", &lsm303dlhc->delay);
	get_ops(LSM303DLHC_ACC_PATHBASE, "sample", &lsm303dlhc->sample);

	get_lsb(lsm303dlhc->range, &lsm303dlhc->lsb);
	get_offset(lsm303dlhc);
}

int dev_close(struct lsm303dlhc *lsm303dlhc)
{
	set_ops(LSM303DLHC_ACC_PATHBASE, "enable", 0);
}

int show_data(struct lsm303dlhc *lsm303dlhc)
{
	unsigned long tm;
	struct timeval time;

        gettimeofday(&time, NULL);
	tm = time.tv_sec*1000 + time.tv_usec/1000;

	printf("%10.5f\t%10.5f\t%10.5f\t%d\n", lsm303dlhc->data.x,
			lsm303dlhc->data.y, lsm303dlhc->data.z, tm);
}

void get_lsb(const int range, double *lsb)
{
	switch(range) {
	case LSM303DLHC_ACC_RANGE_2G:
		*lsb = 16384.00;
		break;
	case LSM303DLHC_ACC_RANGE_4G:
		*lsb = 8192.00;
		break;
	case LSM303DLHC_ACC_RANGE_8G:
		*lsb = 4096.00;
		break;
	case LSM303DLHC_ACC_RANGE_16G:
		*lsb = 2048.00;
		break;
	default:
		break;
	}
}

void get_offset(struct lsm303dlhc *lsm303dlhc)
{
	unsigned char i;
	struct axis_data tmp = {0};

	for (i=0; i<20; i++) {
		while (1) {
			read_acc(LSM303DLHC_ACC_PATHBASE, "acc", &tmp);

			if (fabs(tmp.x/lsm303dlhc->lsb) < (lsm303dlhc->standData.x + lsm303dlhc->range*0.05)
					&& fabs(tmp.y/lsm303dlhc->lsb) < (lsm303dlhc->standData.y + lsm303dlhc->range*0.05)
					&& fabs(tmp.z/lsm303dlhc->lsb) < (lsm303dlhc->standData.z + lsm303dlhc->range*0.05))
				break;
		}

		lsm303dlhc->offset.x += tmp.x;
		lsm303dlhc->offset.y += tmp.y;
		lsm303dlhc->offset.z += tmp.z;

		usleep(1000);
	}

	lsm303dlhc->offset.x = lsm303dlhc->offset.x/lsm303dlhc->lsb/20 - lsm303dlhc->standData.x;
	lsm303dlhc->offset.y = lsm303dlhc->offset.y/lsm303dlhc->lsb/20 - lsm303dlhc->standData.y;
	lsm303dlhc->offset.z = lsm303dlhc->offset.z/lsm303dlhc->lsb/20 - lsm303dlhc->standData.z;
}

int read_acc(const char *base, const char *name, struct axis_data *data)
{
	char buf[BUF_MAX] = "", *end;
	int fd;
	char *pathname = get_path(base, name);

	if (pathname == NULL) {
		free(pathname);
		return -1;
	}

	fd = open(pathname, O_RDWR);
	if (fd < 0) {
		printf("ERROR: Open %s fail!\n", pathname);
		free(pathname);
		return -1;
	}

	if (read(fd, buf, sizeof(buf)) < 0) {
		printf("ERROR: Read acc data!\n");
		close(fd);
		free(pathname);
		return -1;
	}

	data->x = strtod(buf, &end);
	data->y = strtod(end, &end);
	data->z = strtod(end, NULL);

	free(pathname);
	close(fd);
	return 0;
}

int test_data(struct lsm303dlhc *lsm303dlhc)
{
	struct axis_data tmp = {0};

	if (read_acc(LSM303DLHC_ACC_PATHBASE, "acc", &tmp) < 0)
		return -1;

	lsm303dlhc->data.x = tmp.x/lsm303dlhc->lsb - lsm303dlhc->offset.x;
	lsm303dlhc->data.y = tmp.y/lsm303dlhc->lsb - lsm303dlhc->offset.y;
	lsm303dlhc->data.z = tmp.z/lsm303dlhc->lsb - lsm303dlhc->offset.z;

	return 0;
}
