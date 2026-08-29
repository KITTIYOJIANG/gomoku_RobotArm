#ifndef __SAFE_CONSOLE_H__
#define __SAFE_CONSOLE_H__

#include "stm32f10x_conf.h"

#define SAFE_CONSOLE_LINE_SIZE 64

void safe_console_init(void);
void safe_console_rx_isr(u8 byte);
u8 safe_console_take_line(char *out, u16 out_size);
u8 safe_console_take_stop_request(void);
u8 safe_console_take_overflow(void);
void safe_console_write(const char *text);

#endif
