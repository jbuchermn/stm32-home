#ifndef CDCACM_H
#define CDCACM_H

#define NOTIF_PACKET_SIZE 16
#define CDCACM_PACKET_SIZE 64

#include <stdbool.h>
void assertm(bool condition, const char *msg);
#define assert(c) assertm(c, #c)

void cdcacm_init(void);
int cdcacm_get_configuration(void);
const char *cdcacm_get_serialno(void);
void cdcacm_write_now(const char *buf, int len);

#endif
