#include "safe_console.h"

#include <string.h>

#include "z_usart.h"

static volatile char rx_line[SAFE_CONSOLE_LINE_SIZE];
static volatile char ready_line[SAFE_CONSOLE_LINE_SIZE];
static volatile u16 rx_length;
static volatile u16 ready_length;
static volatile u8 line_ready;
static volatile u8 urgent_stop;
static volatile u8 urgent_line;
static volatile u8 rx_overflow;

static u8 ascii_lower(u8 value) {
	if(value >= 'A' && value <= 'Z') {
		return (u8)(value + ('a' - 'A'));
	}
	return value;
}

static u8 current_line_is_stop(void) {
	if(rx_length == 4 &&
		ascii_lower((u8)rx_line[0]) == 's' &&
		ascii_lower((u8)rx_line[1]) == 't' &&
		ascii_lower((u8)rx_line[2]) == 'o' &&
		ascii_lower((u8)rx_line[3]) == 'p') {
		return 1;
	}

	if(rx_length == 5 &&
		rx_line[0] == '$' &&
		ascii_lower((u8)rx_line[1]) == 'd' &&
		ascii_lower((u8)rx_line[2]) == 's' &&
		ascii_lower((u8)rx_line[3]) == 't' &&
		rx_line[4] == '!') {
		return 1;
	}

	return 0;
}

void safe_console_init(void) {
	rx_length = 0;
	ready_length = 0;
	line_ready = 0;
	urgent_stop = 0;
	urgent_line = 0;
	rx_overflow = 0;
}

void safe_console_rx_isr(u8 byte) {
	u16 i;
	u8 delimiter;

	delimiter = (byte == '\r' || byte == '\n' || byte == '!');

	if(byte != '\r' && byte != '\n') {
		if(rx_length < (SAFE_CONSOLE_LINE_SIZE - 1)) {
			rx_line[rx_length++] = (char)byte;
			rx_line[rx_length] = '\0';
		} else {
			rx_overflow = 1;
			rx_length = 0;
			urgent_line = 0;
		}
	}

	if(current_line_is_stop()) {
		urgent_stop = 1;
		urgent_line = 1;
	}

	if(delimiter) {
		if(rx_length != 0 && !urgent_line && !line_ready) {
			ready_length = rx_length;
			for(i = 0; i < ready_length; i++) {
				ready_line[i] = rx_line[i];
			}
			ready_line[ready_length] = '\0';
			line_ready = 1;
		} else if(rx_length != 0 && !urgent_line && line_ready) {
			rx_overflow = 1;
		}
		rx_length = 0;
		rx_line[0] = '\0';
		urgent_line = 0;
	}
}

u8 safe_console_take_line(char *out, u16 out_size) {
	u16 i;
	u16 copy_length;

	if(!line_ready || out == 0 || out_size == 0) {
		return 0;
	}

	copy_length = ready_length;
	if(copy_length >= out_size) {
		copy_length = out_size - 1;
	}

	for(i = 0; i < copy_length; i++) {
		out[i] = ready_line[i];
	}
	out[copy_length] = '\0';

	ready_length = 0;
	ready_line[0] = '\0';
	line_ready = 0;
	return 1;
}

u8 safe_console_take_stop_request(void) {
	if(!urgent_stop) {
		return 0;
	}
	urgent_stop = 0;
	return 1;
}

u8 safe_console_take_overflow(void) {
	if(!rx_overflow) {
		return 0;
	}
	rx_overflow = 0;
	return 1;
}

void safe_console_write(const char *text) {
	if(text != 0) {
		tb_usart1_send_str((u8 *)text);
	}
}
