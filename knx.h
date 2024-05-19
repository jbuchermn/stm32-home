#ifndef KNX_H
#define KNX_H

#include <stdbool.h>
#include <stdint.h>

struct knx_telegram {
    bool repeated;
    char priority;

    uint16_t source_address;
    uint16_t target_address;
    bool target_is_group_address;
    char routing_counter;

    int sequence;
    uint16_t apci;
    char payload[14];
    char payload_len;
};

int knx_init(void);
int knx_main(void);

int knx_read(struct knx_telegram *telegram);

int knx_write(struct knx_telegram *telegram);

#endif
