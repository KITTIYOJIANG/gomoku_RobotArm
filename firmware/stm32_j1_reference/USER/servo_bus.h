#ifndef __SERVO_BUS_H__
#define __SERVO_BUS_H__

#include "stm32f10x_conf.h"

#define SERVO_BUS_MIN_ID 0
#define SERVO_BUS_MAX_ID 5
#define SERVO_BUS_MAX_MOVABLE_ID 4
#define SERVO_BUS_MIN_POSITION 500
#define SERVO_BUS_MAX_POSITION 2500

void servo_bus_init(void);
void servo_bus_rx_isr(u8 byte);
void servo_bus_request_position(u8 id);
u8 servo_bus_take_position(u8 *id, u16 *position);
void servo_bus_request_telemetry(u8 id);
u8 servo_bus_take_response(char *out, u8 out_size);
void servo_bus_move(u8 id, u16 position, u16 time_ms);
void servo_bus_stop_joint(u8 id);
void servo_bus_stop_all(void);
void servo_bus_restore_torque(u8 id);

#endif
