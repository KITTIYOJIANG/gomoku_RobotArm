#include "servo_bus.h"

#include <stdio.h>

#include "z_usart.h"

#define SERVO_RESPONSE_SIZE 24

static volatile char response[SERVO_RESPONSE_SIZE];
static volatile u8 response_length;
static volatile u8 response_ready;

static void servo_bus_clear_response(void) {
	response_length = 0;
	response_ready = 0;
	response[0] = '\0';
}

static void servo_bus_send(const char *command) {
	if(command != 0) {
		tb_usart3_send_str((u8 *)command);
	}
}

void servo_bus_init(void) {
	servo_bus_clear_response();
	tb_usart3_init(115200);
	uart3_open();
}

void servo_bus_rx_isr(u8 byte) {
	if(byte == '#') {
		response_length = 0;
		response_ready = 0;
	}

	if(response_ready) {
		return;
	}

	if(response_length == 0 && byte != '#') {
		return;
	}

	if(response_length < (SERVO_RESPONSE_SIZE - 1)) {
		response[response_length++] = (char)byte;
		response[response_length] = '\0';
	} else {
		servo_bus_clear_response();
		return;
	}

	if(byte == '!') {
		response_ready = 1;
	}
}

void servo_bus_request_position(u8 id) {
	char command[16];

	if(id > SERVO_BUS_MAX_ID) {
		return;
	}

	servo_bus_clear_response();
	sprintf(command, "#%03uPRAD!", (unsigned int)id);
	servo_bus_send(command);
}

void servo_bus_request_telemetry(u8 id) {
	char command[16];

	if(id > SERVO_BUS_MAX_ID) {
		return;
	}

	servo_bus_clear_response();
	sprintf(command, "#%03uPRTV!", (unsigned int)id);
	servo_bus_send(command);
}

u8 servo_bus_take_response(char *out, u8 out_size) {
	u8 i;
	u8 copy_length;

	if(!response_ready || out == 0 || out_size == 0) {
		return 0;
	}

	copy_length = response_length;
	if(copy_length >= out_size) {
		copy_length = out_size - 1;
	}
	for(i = 0; i < copy_length; i++) {
		out[i] = response[i];
	}
	out[copy_length] = '\0';
	servo_bus_clear_response();
	return 1;
}

u8 servo_bus_take_position(u8 *id, u16 *position) {
	char local[SERVO_RESPONSE_SIZE];
	u8 i;
	u16 parsed_id;
	u16 parsed_position;

	if(!response_ready || id == 0 || position == 0) {
		return 0;
	}

	for(i = 0; i <= response_length && i < SERVO_RESPONSE_SIZE; i++) {
		local[i] = response[i];
	}
	servo_bus_clear_response();

	if(local[0] != '#' ||
		local[1] < '0' || local[1] > '9' ||
		local[2] < '0' || local[2] > '9' ||
		local[3] < '0' || local[3] > '9' ||
		local[4] != 'P') {
		return 0;
	}

	parsed_id = (u16)((local[1] - '0') * 100 + (local[2] - '0') * 10 + (local[3] - '0'));
	parsed_position = 0;
	i = 5;
	while(local[i] >= '0' && local[i] <= '9') {
		parsed_position = (u16)(parsed_position * 10 + (local[i] - '0'));
		i++;
		if(i >= (SERVO_RESPONSE_SIZE - 1)) {
			return 0;
		}
	}

	if(local[i] != '!' ||
		parsed_id > SERVO_BUS_MAX_ID ||
		parsed_position < SERVO_BUS_MIN_POSITION ||
		parsed_position > SERVO_BUS_MAX_POSITION) {
		return 0;
	}

	*id = (u8)parsed_id;
	*position = parsed_position;
	return 1;
}

void servo_bus_move(u8 id, u16 position, u16 time_ms) {
	char command[24];

	/* Stage-1 hard guard: ID 005 is read-only even if a caller bypasses the GUI. */
	if(id > SERVO_BUS_MAX_MOVABLE_ID ||
		position < SERVO_BUS_MIN_POSITION ||
		position > SERVO_BUS_MAX_POSITION ||
		time_ms > 9999) {
		return;
	}

	sprintf(command, "#%03uP%04uT%04u!", (unsigned int)id,
		(unsigned int)position, (unsigned int)time_ms);
	servo_bus_send(command);
}

void servo_bus_stop_joint(u8 id) {
	char command[16];

	if(id > SERVO_BUS_MAX_ID) {
		return;
	}
	sprintf(command, "#%03uPDST!", (unsigned int)id);
	servo_bus_send(command);
}

void servo_bus_stop_all(void) {
	servo_bus_send("#255PDST!");
}

void servo_bus_restore_torque(u8 id) {
	char command[16];

	if(id > SERVO_BUS_MAX_ID) {
		return;
	}
	sprintf(command, "#%03uPULR!", (unsigned int)id);
	servo_bus_send(command);
}
