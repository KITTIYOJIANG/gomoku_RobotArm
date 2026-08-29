#include "stm32f10x_conf.h"

#include "arm_safety.h"
#include "safe_console.h"
#include "servo_bus.h"
#include "z_gpio.h"
#include "z_timer.h"
#include "z_usart.h"

static void status_led_process(void) {
	static u32 changed_ms = 0;
	static u8 led_on = 0;
	u32 now;

	now = millis();
	if((u32)(now - changed_ms) < 500) {
		return;
	}
	changed_ms = now;
	led_on = !led_on;
	if(led_on) {
		nled_on();
	} else {
		nled_off();
	}
}

void soft_reset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

int main(void) {
	char command[SAFE_CONSOLE_LINE_SIZE];

	/* Hardware initialization. No servo target is transmitted here. */
	/* SystemInit() already selected the clock before main(). */
	tb_gpio_init();
	nled_init();
	nled_off();
	beep_init();
	beep_off();

	safe_console_init();
	tb_usart1_init(115200);
	uart1_open();
	servo_bus_init();
	SysTick_Int_Init();
	tb_interrupt_open();
	IWDG_Init();

	arm_safety_init();

	while(1) {
		if(safe_console_take_stop_request()) {
			arm_safety_emergency_stop();
		}

		if(safe_console_take_overflow()) {
			safe_console_write("ERR RX_OVERFLOW\r\n");
		}

		if(safe_console_take_line(command, sizeof(command))) {
			arm_safety_handle_command(command);
		}

		arm_safety_process();
		status_led_process();
	}
}
