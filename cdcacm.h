#ifndef CDCACM_H
#define CDCACM_H

#include <stdint.h>

#define CDCACM_BUFSIZE 1024

int cdcacm_init(void);

int cdcacm_main(void);

uint16_t cdcacm_len_read(void);
uint16_t cdcacm_len_line(void);
uint16_t cdcacm_read(char **buf, uint16_t max);
int cdcacm_write(char *buf, uint16_t len);
void cdcacm_write_hex(char val);

#endif
