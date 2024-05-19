#ifndef APP_H
#define APP_H

#include "knx.h"

void app_init(void);

void app_handle_knx(struct knx_telegram *telegram);
void app_handle_dali(char *dali, int len);
void app_main(void);

void app_usb_command(char *cmd, int len);

#endif
