#ifndef _OPS_H_
#define _OPS_H_

char *get_path(const char *, const char *);
int write_ops(const char *, int);
int read_ops(const char *, long *);
int set_ops(const char *, const char *, int);
int get_ops(const char *, const char *, int *);

#endif /* _OPS_H_ */
