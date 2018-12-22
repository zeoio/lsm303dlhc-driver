#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#define BUF_MAX		128

char *get_path(const char *base, const char *name)
{
	if (name == NULL)
		return NULL;

	char *pathname = malloc(sizeof(char) * (strlen(base)+strlen(name)+1));
	if (pathname == NULL) {
		printf("ERROR: Get pathname!\n");
		return NULL;
	}

	sprintf(pathname, "%s/%s", base, name);
	return pathname;
}

int write_ops(const char *pathname, int data)
{
	char buf[BUF_MAX] = "";
	int fd;

	fd = open(pathname, O_RDWR);
	if (fd < 0) {
		printf("ERROR: Open %s fail!\n", pathname);
		return -1;
	}

	sprintf(buf, "%d", data);

	if (write(fd, buf, strlen(buf)+1) < 0) {
		printf("ERROR: Write %s fail!\n", buf);
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}

int read_ops(const char *pathname, int *data)
{
	char buf[BUF_MAX] = "";
	int fd;

	fd = open(pathname, O_RDWR);
	if (fd < 0) {
		printf("ERROR: Open %s fail!\n", pathname);
		return -1;
	}

	if (read(fd, buf, sizeof(buf)) < 0) {
		printf("ERROR: Read %s fail!\n", pathname);
		close(fd);
		return -1;
	}

	*data = (int)strtol(buf, NULL, 10);
	close(fd);
	return 0;
}

int set_ops(const char *base, const char *name, int data)
{
	char *pathname = get_path(base, name);

	if (pathname) {
		if (write_ops(pathname, data) < 0)
			return -1;
	}

	free(pathname);
	return 0;
}

int get_ops(const char *base, const char *name, int *data)
{
	char *pathname = get_path(base, name);

	if (pathname) {
		if (read_ops(pathname, data) < 0)
			return -1;
	}

	free(pathname);
	return 0;
}
