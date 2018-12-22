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
int get_sample(struct lsm303dlhc *);
int get_range(struct lsm303dlhc *);
int show_data(struct lsm303dlhc *);
void get_lsb(struct lsm303dlhc *);
void get_offset(struct lsm303dlhc *);
int read_mag(const char *, const char *, struct axis_data *);
int test_data(struct lsm303dlhc *);

int main(void)
{
	unsigned int i;
	struct lsm303dlhc lsm303dlhc;

	dev_init(&lsm303dlhc);

	for (i=0; i<10000; i++) {
		test_data(&lsm303dlhc);
		show_data(&lsm303dlhc);
		usleep(2800);
	}

	return 0;
}

int dev_init(struct lsm303dlhc *lsm303dlhc)
{
	set_ops(LSM303DLHC_MAG_PATHBASE, "sample", LSM303DLHC_MAG_RATE_75HZ);

	memset(lsm303dlhc, 0, sizeof(*lsm303dlhc));

	lsm303dlhc->standData.x = 0;
	lsm303dlhc->standData.y = 0;
	lsm303dlhc->standData.z = 0;

	get_range(lsm303dlhc);
	get_sample(lsm303dlhc);
	get_lsb(lsm303dlhc);
	get_ops(LSM303DLHC_MAG_PATHBASE, "delay", &lsm303dlhc->delay);

	printf("range: %.2f delay: %d sample: %.2lf lsb: %.2lf zlsb: %.2lf\n",
			lsm303dlhc->range, lsm303dlhc->delay, lsm303dlhc->sample,
			lsm303dlhc->lsb, lsm303dlhc->zlsb);

	get_offset(lsm303dlhc);
	printf("offset: %lf %lf %lf\n", lsm303dlhc->offset.x,
			lsm303dlhc->offset.y, lsm303dlhc->offset.z);
}

int get_sample(struct lsm303dlhc *lsm303dlhc)
{
	int sample;
	get_ops(LSM303DLHC_MAG_PATHBASE, "sample", &sample);

	switch (sample) {
	case LSM303DLHC_MAG_RATE_0_75HZ:
		lsm303dlhc->sample = 0.75;
		break;
	case LSM303DLHC_MAG_RATE_1_5HZ:
		lsm303dlhc->sample = 1.5;
		break;
	case LSM303DLHC_MAG_RATE_3HZ:
		lsm303dlhc->sample = 3.0;
		break;
	case LSM303DLHC_MAG_RATE_7_5HZ:
		lsm303dlhc->sample = 7.5;
		break;
	case LSM303DLHC_MAG_RATE_15HZ:
		lsm303dlhc->sample = 15.0;
		break;
	case LSM303DLHC_MAG_RATE_30HZ:
		lsm303dlhc->sample = 30.0;
		break;
	case LSM303DLHC_MAG_RATE_75HZ:
		lsm303dlhc->sample = 75.0;
		break;
	case LSM303DLHC_MAG_RATE_220HZ:
		lsm303dlhc->sample = 220.0;
		break;
	default:
		break;
	}
}

int get_range(struct lsm303dlhc *lsm303dlhc)
{
	int range;
	get_ops(LSM303DLHC_MAG_PATHBASE, "range", &range);

	switch (range) {
	case LSM303DLHC_MAG_GRANGE_1_3G:
		lsm303dlhc->range = 1.3;
		break;
	case LSM303DLHC_MAG_GRANGE_1_9G:
		lsm303dlhc->range = 1.9;
		break;
	case LSM303DLHC_MAG_GRANGE_2_5G:
		lsm303dlhc->range = 2.5;
		break;
	case LSM303DLHC_MAG_GRANGE_4G:
		lsm303dlhc->range = 4.0;
		break;
	case LSM303DLHC_MAG_GRANGE_4_7G:
		lsm303dlhc->range = 4.7;
		break;
	case LSM303DLHC_MAG_GRANGE_5_6G:
		lsm303dlhc->range = 5.6;
		break;
	case LSM303DLHC_MAG_GRANGE_8_1G:
		lsm303dlhc->range = 8.1;
		break;
	default:
		break;
	}
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

void get_lsb(struct lsm303dlhc *lsm303dlhc)
{
	int range;
	get_ops(LSM303DLHC_MAG_PATHBASE, "range", &range);

	switch(range) {
	case LSM303DLHC_MAG_GRANGE_1_3G:
		lsm303dlhc->lsb = 1100.00;
		lsm303dlhc->zlsb = 980.00;
		break;
	case LSM303DLHC_MAG_GRANGE_1_9G:
		lsm303dlhc->lsb = 855.00;
		lsm303dlhc->zlsb = 760.00;
		break;
	case LSM303DLHC_MAG_GRANGE_2_5G:
		lsm303dlhc->lsb = 670.00;
		lsm303dlhc->zlsb = 600.00;
		break;
	case LSM303DLHC_MAG_GRANGE_4G:
		lsm303dlhc->lsb = 450.00;
		lsm303dlhc->zlsb = 400.00;
		break;
	case LSM303DLHC_MAG_GRANGE_4_7G:
		lsm303dlhc->lsb = 400.00;
		lsm303dlhc->zlsb = 355.00;
		break;
	case LSM303DLHC_MAG_GRANGE_5_6G:
		lsm303dlhc->lsb = 330.00;
		lsm303dlhc->zlsb = 295.00;
		break;
	case LSM303DLHC_MAG_GRANGE_8_1G:
		lsm303dlhc->lsb = 230.00;
		lsm303dlhc->zlsb = 205.00;
		break;
	default:
		break;
	};
}

void get_offset(struct lsm303dlhc *lsm303dlhc)
{
	unsigned int i;
	struct axis_data tmp = {0}, max_data = {0}, min_data = {0};

	for (i=0; i<10000; i++) {
		read_mag(LSM303DLHC_MAG_PATHBASE, "mag", &tmp);

		if (tmp.x > max_data.x)
			max_data.x = tmp.x*0.2 + max_data.x*0.8;
		if (tmp.x < min_data.x)
			min_data.x = tmp.x*0.2 + min_data.x*0.8;

		if (tmp.y > max_data.y)
			max_data.y = tmp.y*0.2 + max_data.y*0.8;
		if (tmp.y < min_data.y)
			min_data.y = tmp.y*0.2 + min_data.y*0.8;

		if (tmp.z > max_data.z)
			max_data.z = tmp.z*0.2 + max_data.z*0.8;
		if (tmp.z < min_data.z)
			min_data.z = tmp.z*0.2 + min_data.z*0.8;

		usleep(1000);
	}

	lsm303dlhc->offset.x = (max_data.x+min_data.x) / 2.0;
	lsm303dlhc->offset.y = (max_data.y+min_data.y) / 2.0;
	lsm303dlhc->offset.z = (max_data.z+min_data.z) / 2.0;

/*         unsigned char i;
 *         struct axis_data tmp = {0};
 *
 *         for (i=0; i<20; i++) {
 *                 while (1) {
 *                         read_mag(LSM303DLHC_MAG_PATHBASE, "mag", &tmp);
 *                         printf("%lf %lf %lf\n", tmp.x, tmp.y, tmp.z);
 *
 *                         if (fabs(tmp.x/lsm303dlhc->lsb) < (lsm303dlhc->standData.x + lsm303dlhc->range*0.05)
 *                                         && fabs(tmp.y/lsm303dlhc->lsb) < (lsm303dlhc->standData.y + lsm303dlhc->range*0.05)
 *                                         && fabs(tmp.z/lsm303dlhc->zlsb) < (lsm303dlhc->standData.z + lsm303dlhc->range*0.05))
 *                                 break;
 *                 }
 *
 *                 lsm303dlhc->offset.x += tmp.x;
 *                 lsm303dlhc->offset.y += tmp.y;
 *                 lsm303dlhc->offset.z += tmp.z;
 *
 *                 usleep(1000);
 *         }
 *
 *         lsm303dlhc->offset.x = lsm303dlhc->offset.x/lsm303dlhc->lsb/20 - lsm303dlhc->standData.x;
 *         lsm303dlhc->offset.y = lsm303dlhc->offset.y/lsm303dlhc->lsb/20 - lsm303dlhc->standData.y;
 *         lsm303dlhc->offset.z = lsm303dlhc->offset.z/lsm303dlhc->zlsb/20 - lsm303dlhc->standData.z; */
}

int read_mag(const char *base, const char *name, struct axis_data *data)
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
		printf("ERROR: Read mag data!\n");
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

	if (read_mag(LSM303DLHC_MAG_PATHBASE, "mag", &tmp) < 0)
		return -1;

	lsm303dlhc->data.x = (tmp.x - lsm303dlhc->offset.x) / lsm303dlhc->lsb;
	lsm303dlhc->data.y = (tmp.y - lsm303dlhc->offset.y) / lsm303dlhc->lsb;
	lsm303dlhc->data.z = (tmp.z - lsm303dlhc->offset.z) / lsm303dlhc->zlsb;

	return 0;
}
