/* silent-stepper-v2-bricklet
 * Copyright (C) 2020-2021 Olaf Lüke <olaf@tinkerforge.com>
 *
 * gpio.c: Driver for GPIO inputs/interrupts
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "gpio.h"
#include "configs/config_gpio.h"

#include "communication.h"
#include "bricklib2/hal/system_timer/system_timer.h"
#include "bricklib2/logging/logging.h"

#include "stepper.h"
#include "xmc_gpio.h"

GPIO gpio;

#define GPIO_PIN_MASK ((1 << GPIO_0_PIN) | (1 << GPIO_1_PIN))

// It was not easily possible to route the GPIOs to ERU (interrupt) pins on the
// Silent Stepper Bricklet 2.0 PCB. Instead of using interrupts we check the gpio state before each step.
inline void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) gpio_check(void) {
	static uint32_t last_interrupt_bitmask = 0;

	// We check if a GPIO value has changed as fast as possible with a bitmask,
	// so this gpio_check function uses the least amount of time possible.
	const uint32_t gpio_port_value = GPIO_PORT->IN;
	if((gpio_port_value & GPIO_PIN_MASK) == last_interrupt_bitmask) {
		return;
	}

	const bool value0 = gpio_port_value & (1 << GPIO_0_PIN);
	if((gpio.last_interrupt_value[0] != value0) && (gpio.last_interrupt_time[0] == 0)) {
		gpio.last_interrupt_value[0] = value0;
		// Check for callback
		if(( value0 && (gpio.action[0] & SILENT_STEPPER_V2_GPIO_ACTION_CALLBACK_RISING_EDGE)) ||
		(!value0 && (gpio.action[0] & SILENT_STEPPER_V2_GPIO_ACTION_CALLBACK_FALLING_EDGE))) {
			gpio.new_callback = true;
		}

		last_interrupt_bitmask = (gpio.last_interrupt_value[0] << GPIO_0_PIN) | (gpio.last_interrupt_value[1] << GPIO_1_PIN);
		if(gpio.debounce[0] != 0) {
			gpio.last_interrupt_time[0] = system_timer_get_ms();
		}

		// Check if action is necessary
		if(value0 && (gpio.action[0] & SILENT_STEPPER_V2_GPIO_ACTION_FULL_BRAKE_RISING_EDGE)) {
			gpio.stop_emergency  = true;
			stepper_full_brake();
		} else if(value0 && (gpio.action[0] & SILENT_STEPPER_V2_GPIO_ACTION_NORMAL_STOP_RISING_EDGE)) {
			gpio.stop_normal     = true;
			stepper.deceleration = gpio.stop_deceleration[0];
			stepper_stop();
		} else if(!value0 && (gpio.action[0] & SILENT_STEPPER_V2_GPIO_ACTION_FULL_BRAKE_FALLING_EDGE)) {
			gpio.stop_emergency  = true;
			stepper_full_brake();
		} else if(!value0 && (gpio.action[0] & SILENT_STEPPER_V2_GPIO_ACTION_NORMAL_STOP_FALLING_EDGE)) {
			gpio.stop_normal     = true;
			stepper.deceleration = gpio.stop_deceleration[0];
			stepper_stop();
		}
	}

	const bool value1 = gpio_port_value & (1 << GPIO_1_PIN);
	if((gpio.last_interrupt_value[1] != value1) && (gpio.last_interrupt_time[1] == 0)) {
		gpio.last_interrupt_value[1] = value1;
		// Check for callback
		if(( value1 && (gpio.action[1] & SILENT_STEPPER_V2_GPIO_ACTION_CALLBACK_RISING_EDGE)) ||
		(!value1 && (gpio.action[1] & SILENT_STEPPER_V2_GPIO_ACTION_CALLBACK_FALLING_EDGE))) {
			gpio.new_callback = true;
		}

		last_interrupt_bitmask = (gpio.last_interrupt_value[0] << GPIO_0_PIN) | (gpio.last_interrupt_value[1] << GPIO_1_PIN);
		if(gpio.debounce[1] != 0) {
			gpio.last_interrupt_time[1] = system_timer_get_ms();
		}

		// Check if action is necessary
		if(value1 && (gpio.action[1] & SILENT_STEPPER_V2_GPIO_ACTION_FULL_BRAKE_RISING_EDGE)) {
			gpio.stop_emergency  = true;
			stepper_full_brake();
		} else if(value1 && (gpio.action[1] & SILENT_STEPPER_V2_GPIO_ACTION_NORMAL_STOP_RISING_EDGE)) {
			gpio.stop_normal     = true;
			stepper.deceleration = gpio.stop_deceleration[1];
			stepper_stop();
		} else if(!value1 && (gpio.action[1] & SILENT_STEPPER_V2_GPIO_ACTION_FULL_BRAKE_FALLING_EDGE)) {
			gpio.stop_emergency  = true;
			stepper_full_brake();
		} else if(!value1 && (gpio.action[1] & SILENT_STEPPER_V2_GPIO_ACTION_NORMAL_STOP_FALLING_EDGE)) {
			gpio.stop_normal     = true;
			stepper.deceleration = gpio.stop_deceleration[1];
			stepper_stop();
		}
	}
}

void gpio_init(void) {
    memset(&gpio, 0, sizeof(GPIO));

	gpio.action[0]                        = SILENT_STEPPER_V2_GPIO_ACTION_NONE;
	gpio.action[1]                        = SILENT_STEPPER_V2_GPIO_ACTION_NONE;
	gpio.debounce[0]                      = 200; // 200ms default
	gpio.debounce[1]                      = 200; // 200ms default
	gpio.stop_deceleration[0]             = 0xFFFF;
	gpio.stop_deceleration[1]             = 0xFFFF;

	// Init GPIO IRQs
	const XMC_GPIO_CONFIG_t gpio_pin_config = {
		.mode             = XMC_GPIO_MODE_INPUT_TRISTATE,
		.input_hysteresis = XMC_GPIO_INPUT_HYSTERESIS_STANDARD
	};
	XMC_GPIO_Init(GPIO_PORT, GPIO_0_PIN, &gpio_pin_config);
	XMC_GPIO_Init(GPIO_PORT, GPIO_1_PIN, &gpio_pin_config);
}

void gpio_tick(void) {
	// if the stepper is not running we call gpio_check from the tick to make
	// sure that the gpio state even updates when the stepper motor is not used.
	if(!stepper_is_currently_running()) {
		gpio_check();
	}

	// Enable gpio_check again after debounce time
	for(uint8_t channel = 0; channel < GPIO_CHANNEL_NUM; channel++) {
		if(gpio.last_interrupt_time[channel] != 0) {
			if(system_timer_is_time_elapsed_ms(gpio.last_interrupt_time[channel], gpio.debounce[channel])) {
				gpio.last_interrupt_time[channel] = 0;
			}
		}
	}
}