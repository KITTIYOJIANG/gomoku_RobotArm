#ifndef __ARM_SAFETY_H__
#define __ARM_SAFETY_H__

#include "stm32f10x_conf.h"

void arm_safety_init(void);
void arm_safety_process(void);
void arm_safety_handle_command(const char *command);
void arm_safety_emergency_stop(void);

#endif
