/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * stepper.c: Driver for stepper motion
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

#include "stepper.h"

#include "configs/config_stepper.h"
#include "bricklib2/os/coop_task.h"
#include "bricklib2/logging/logging.h"
#include "bricklib2/utility/util_definitions.h"

#include "tmc2130.h"
#include "configs/config_tmc2130.h"
#include "configs/config_stepper.h"

#include "timer_irq.h"
#include "voltage.h"
#include "gpio.h"

Stepper stepper;

extern const XMC_GPIO_CONFIG_t input_pullup_config;
extern const XMC_GPIO_CONFIG_t input_default_config;
extern const XMC_GPIO_CONFIG_t output_high_config;
extern const XMC_GPIO_CONFIG_t output_low_config;

#define STEPPER_MAX_TIMER_VALUE 0xFFFF
#define STEPPER_CCU_CLK 96000000

const uint32_t stepper_timer_frequency[] = {
	STEPPER_CCU_CLK/1,
	STEPPER_CCU_CLK/2,
	STEPPER_CCU_CLK/4,
	STEPPER_CCU_CLK/8,
	STEPPER_CCU_CLK/16,
	STEPPER_CCU_CLK/32,
	STEPPER_CCU_CLK/64,
	STEPPER_CCU_CLK/128,
	STEPPER_CCU_CLK/256,
	STEPPER_CCU_CLK/512,
	STEPPER_CCU_CLK/1024,
	STEPPER_CCU_CLK/2048,
};

const uint32_t stepper_timer_velocity[]  = {
	(STEPPER_CCU_CLK/1)     / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/2)     / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/4)     / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/8)     / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/16)    / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/32)    / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/64)    / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/128)   / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/256)   / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/512)   / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/1024)  / STEPPER_MAX_TIMER_VALUE,
	(STEPPER_CCU_CLK/2048)  / STEPPER_MAX_TIMER_VALUE,
};

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) timer_irq(void) {
	gpio_check();

	stepper.time_base_counter--;
	if(stepper.time_base_counter > 0) {
		timer_irq_start();
		return;
	}

	stepper.time_base_counter = stepper.time_base;

	if(stepper.state != STEPPER_STATE_STOP) {
		stepper_set_next_timer(stepper.velocity);
		stepper_make_step();
	}

	if(stepper.state == STEPPER_STATE_STEPS ||
	   stepper.state == STEPPER_STATE_TARGET) {
		stepper_step_speedramp();
	} else if(stepper.state == STEPPER_STATE_DRIVE) {
		stepper_drive_speedramp();
	}
}

// This is meant to be called by gpio code if non-emergency stop is configured and triggered
void stepper_stop(void) {
	stepper.state = STEPPER_STATE_DRIVE;
	stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);

	if(!stepper_is_currently_running()) {
		// call drive speedramp one time, to get it going
		// (i.e. make velocity != 0)
		stepper_drive_speedramp();
	}	
}

void stepper_make_drive_speedramp(const uint8_t state) {
	stepper.state = STEPPER_STATE_DRIVE;
	stepper.speedramp_state = state;

	if(state == STEPPER_SPEEDRAMP_STATE_STOP) {
		stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);
	} else if((state == STEPPER_SPEEDRAMP_STATE_BACKWARD) || (state == STEPPER_SPEEDRAMP_STATE_FORWARD)) {
		stepper_set_new_api_state(STEPPER_API_STATE_ACCELERATION);
	}

	if(!stepper_is_currently_running()) {
		// call drive speedramp one time, to get it going
		// (i.e. make velocity != 0)
		stepper_drive_speedramp();
		stepper_set_next_timer(stepper.velocity);
	}
}

void stepper_make_step_speedramp(const int32_t steps) {
	if(stepper.velocity_goal == 0) {
		return;
	}

	int32_t use_steps = steps;

	if(use_steps == 0) {
		stepper.position_reached = true;
		stepper.state = STEPPER_STATE_STOP;
		return;
	}
	if(use_steps < 0) {
		stepper_set_direction(STEPPER_DIRECTION_BACKWARD);
		use_steps = -steps;
	} else {
		stepper_set_direction(STEPPER_DIRECTION_FORWARD);
	}

	if(use_steps == 1) {
		// Just make the single step, no need for IRQ
		stepper_make_step();
		stepper.position_reached = true;
		stepper_set_new_api_state(STEPPER_API_STATE_RUN);
		stepper_set_new_api_state(STEPPER_API_STATE_STOP);
		return;
	}

	uint16_t acceleration;
	uint16_t acceleration_sqrt;
	uint16_t deceleration;

	if(stepper.acceleration == 0) {
		acceleration = 0xFFFF;
		acceleration_sqrt = 256; // sqrt(0xFFFF)
	} else {
		acceleration = stepper.acceleration;
		acceleration_sqrt = stepper.acceleration_sqrt;
	}

	if(stepper.deceleration == 0) {
		deceleration = 0xFFFF;
	} else {
		deceleration = stepper.deceleration;
	}

	stepper.acceleration_steps = (stepper.velocity_goal*stepper.velocity_goal) / (2*acceleration);
	if(stepper.acceleration_steps == 0) {
		stepper.acceleration_steps = 1;
	}

	int32_t acceleration_limit = (((int64_t)use_steps)*((int64_t)deceleration)) / (acceleration + deceleration);
	if(acceleration_limit == 0) {
		acceleration_limit = 1;
	}

	if(acceleration_limit <= stepper.acceleration_steps) {
		stepper.deceleration_steps = acceleration_limit - use_steps;
	} else {
		stepper.deceleration_steps = -(((int64_t)stepper.acceleration_steps) * ((int64_t)acceleration) / deceleration);
	}
	if(stepper.deceleration_steps == 0) {
		stepper.deceleration_steps = -1;
	}

	stepper.velocity = acceleration_sqrt;
	stepper.delay = STEPPER_VELOCITY_TO_DELAY(acceleration_sqrt);

	stepper.deceleration_start = use_steps + stepper.deceleration_steps;
	stepper.step_counter = 0;
	stepper.acceleration_counter = 0;
	stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_ACCELERATION;
	stepper_set_new_api_state(STEPPER_API_STATE_ACCELERATION);

	stepper_set_next_timer(stepper.velocity);
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code")))  stepper_set_next_timer(const uint32_t velocity) {
	uint32_t velocity_use = velocity;
	if(velocity == 0) {
		if(stepper.state == STEPPER_STATE_DRIVE && stepper.velocity_goal != 0) {
			// In drive mode we have a transition from backward to forward here.
			// Wait 10ms in that case
			velocity_use = 100;
		} else {
			// In step mode this should not happen, stop tc
			stepper.running = false;
			timer_irq_stop();
			return;
		}
	}

	int8_t i;
	for(i = 11; i > 0; i--) {
		if(velocity_use <= stepper_timer_velocity[i-1]) {
			break;
		}
	}

	XMC_CCU8_SLICE_ClearEvent(CCU80_CC80, XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
	timer_irq_new_time(i, stepper_timer_frequency[i] / velocity_use);

	if(!stepper_is_currently_running()) {
		stepper.running = true;
		timer_irq_start();
	}
}

inline bool stepper_is_currently_running(void) {
	return stepper.running;
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code")))  stepper_make_step(void) {
	// We change step pin back and force for one step each (dedge = 1)
	XMC_GPIO_ToggleOutput(TMC2130_STEP_PIN);

	stepper.position += stepper.direction;
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code")))  stepper_step_speedramp(void) {
	int32_t new_delay = 0;

	switch(stepper.speedramp_state) {
		case STEPPER_SPEEDRAMP_STATE_STOP: {
			stepper.velocity = 0;
			stepper.step_counter = 0;
			stepper.acceleration_counter = 0;
			stepper.delay_rest = 0;

			stepper.running = false;
			timer_irq_stop();
			return;
		}

		case STEPPER_SPEEDRAMP_STATE_ACCELERATION: {
			stepper.step_counter++;
			stepper.acceleration_counter++;

			const int32_t a = (2*stepper.delay + stepper.delay_rest);
			const int32_t b = (4*stepper.acceleration_counter + 1);
			new_delay = stepper.delay - a/b;
			stepper.delay_rest = a % b;

			if(stepper.step_counter >= stepper.deceleration_start) {
				stepper.acceleration_counter = stepper.deceleration_steps;
				stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_DECELERATION;
				stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);
			} else if(new_delay <= STEPPER_VELOCITY_TO_DELAY(stepper.velocity_goal)) {
				stepper.last_delay = new_delay;
				new_delay = STEPPER_VELOCITY_TO_DELAY(stepper.velocity_goal);
				stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_RUN;
				stepper_set_new_api_state(STEPPER_API_STATE_RUN);
			}
			break;
		}

		case STEPPER_SPEEDRAMP_STATE_RUN: {
			stepper.step_counter++;
			if(stepper.step_counter >= stepper.deceleration_start) {
				stepper.acceleration_counter = stepper.deceleration_steps;
				new_delay = stepper.last_delay;
				stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_DECELERATION;
				stepper_set_new_api_state(STEPPER_API_STATE_DECELERATION);
			} else {
				new_delay = STEPPER_VELOCITY_TO_DELAY(stepper.velocity_goal);
			}
			break;
		}

		case STEPPER_SPEEDRAMP_STATE_DECELERATION: {
			stepper.step_counter++;
			stepper.acceleration_counter++;

			const int32_t a = (2*stepper.delay + stepper.delay_rest);
			const int32_t b = (4*stepper.acceleration_counter + 1);
			new_delay = stepper.delay - a/b;
			stepper.delay_rest = a % b;

			if(stepper.acceleration_counter >= 0) {
				stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
				stepper.state = STEPPER_STATE_STOP;
				stepper_set_new_api_state(STEPPER_API_STATE_STOP);
				stepper.position_reached = true;
				stepper.running = false;
				timer_irq_stop();
				stepper.velocity = 0;
				return;
			}
			break;
		}
	}

	stepper.delay = new_delay;
	stepper.velocity = STEPPER_DELAY_TO_VELOCITY(stepper.delay);
}

void stepper_full_brake(void) {
	stepper.running = false;
	timer_irq_stop();
	stepper.state = STEPPER_STATE_STOP;
	stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper_set_new_api_state(STEPPER_API_STATE_STOP);
	stepper.velocity = 0;
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code")))  stepper_drive_speedramp(void) {
	static uint32_t rest = 0;
	uint16_t goal = stepper.velocity_goal;

	if((stepper.speedramp_state == STEPPER_SPEEDRAMP_STATE_STOP) ||
	   (stepper.speedramp_state != stepper.direction)) {
		goal = 0;
	}
    
	if(goal == stepper.velocity) {
		if(stepper.speedramp_state == STEPPER_SPEEDRAMP_STATE_STOP) {
			stepper.running = false;
			timer_irq_stop();
			stepper.state = STEPPER_STATE_STOP;
			stepper.velocity = 0;
			stepper.step_counter = 0;
			stepper.acceleration_counter = 0;
			stepper.delay_rest = 0;
			rest = 0;
			stepper_set_new_api_state(STEPPER_API_STATE_STOP);
			return;
		} else if(stepper.speedramp_state != stepper.direction) {
			goal = stepper.velocity_goal;
			rest = 0;
		} else {
			// If i am at stepper velocity goal and the direction is correct
			// -> We have nothing to do
			return;
		}
	}

	uint16_t acceleration;
	uint16_t acceleration_sqrt;
	uint16_t deceleration;
	int32_t delta;

	if(stepper.acceleration == 0) {
		acceleration = 0xFFFF;
		acceleration_sqrt = 256; // sqrt(0xFFFF)
	} else {
		acceleration = stepper.acceleration;
		acceleration_sqrt = stepper.acceleration_sqrt;
	}
	if(stepper.deceleration == 0) {
		deceleration = 0xFFFF;
	} else {
		deceleration = stepper.deceleration;
	}

	if(stepper.velocity == 0) {
		delta = acceleration_sqrt;
		rest = 0;
		if(stepper.speedramp_state == STEPPER_SPEEDRAMP_STATE_FORWARD) {
			stepper_set_direction(STEPPER_DIRECTION_FORWARD);
		} else if(stepper.speedramp_state == STEPPER_SPEEDRAMP_STATE_BACKWARD) {
			stepper_set_direction(STEPPER_DIRECTION_BACKWARD);
		}
	} else {
		if(stepper.velocity < goal) {
			delta = (acceleration + rest) / stepper.velocity;
			rest = (acceleration + rest) % stepper.velocity;
		} else {
			delta = (deceleration + rest) / stepper.velocity;
			rest = (deceleration + rest) % stepper.velocity;
		}
	}

	if(stepper.velocity < goal) {
		stepper.velocity = MIN(stepper.velocity + delta, goal);

		if(stepper.velocity == goal) {
			stepper_set_new_api_state(STEPPER_API_STATE_RUN);
		}
	} else {
		stepper.velocity = MAX(((int32_t)stepper.velocity) - delta, goal);
	}
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code"))) stepper_set_direction(const int8_t direction) {
	if(direction == stepper.direction) {
		return;
	}

	stepper.direction = direction;
	if(direction == STEPPER_DIRECTION_FORWARD) {
		if(stepper.state == STEPPER_STATE_DRIVE) {
			stepper_set_new_api_state(STEPPER_API_STATE_DIR_CHANGE_FORWARD);
		}
		XMC_GPIO_SetOutputLow(TMC2130_DIR_PIN);
	} else {
		if(stepper.state == STEPPER_STATE_DRIVE) {
			stepper_set_new_api_state(STEPPER_API_STATE_DIR_CHANGE_BACKWARD);
		}
		XMC_GPIO_SetOutputHigh(TMC2130_DIR_PIN);
	}
}

void stepper_enable_pin_apply(void) {
	if(stepper.state == STEPPER_STATE_OFF) {
		XMC_GPIO_Init(TMC2130_ENABLE_PIN, &output_high_config);
	} else {
		XMC_GPIO_Init(TMC2130_ENABLE_PIN, &output_low_config);
	}
}

void stepper_enable(void) {
	stepper.state = STEPPER_STATE_STOP;
	stepper_enable_pin_apply();
	stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper.api_state = STEPPER_API_STATE_STOP;
	stepper.api_prev_state = STEPPER_API_STATE_STOP;
}

void stepper_disable(void) {
	stepper.state = STEPPER_STATE_OFF;
	stepper.speedramp_state = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper_enable_pin_apply();
	stepper_full_brake();
	stepper.state = STEPPER_STATE_OFF;
}

int32_t stepper_get_remaining_steps(void) {
	if(stepper.state == STEPPER_STATE_STEPS) {
		if(stepper.steps > 0) {
			return stepper.steps - stepper.step_counter;
		} else {
			return stepper.steps + stepper.step_counter;
		}
	} else if(stepper.state == STEPPER_STATE_TARGET) {
		return stepper.target_position - stepper.position;
	}

	return 0;
}

void __attribute__((optimize("-O3"))) __attribute__ ((section (".ram_code")))  stepper_set_new_api_state(const uint8_t new_state) {
	stepper.api_prev_state = stepper.api_state;
	stepper.api_state = new_state;
	stepper.api_state_send = true;
}

void stepper_init(void) {
	memset(&stepper, 0, sizeof(Stepper));

	stepper.velocity_goal     = STEPPER_VELOCITY_DEFAULT;
	stepper.acceleration      = STEPPER_ACCELERATION_DEFAULT;
	stepper.acceleration_sqrt = STEPPER_ACCELERATION_SQRT_DEFAULT;
	stepper.deceleration      = STEPPER_DECELERATION_DEFAUL;
	stepper.minimum_voltage   = STEPPER_MINIMUM_VOLTAGE_DEFAULT;

	stepper.output_current    = STEPPER_OUTPUT_CURRENT_DEFAULT;
	stepper.state             = STEPPER_STATE_OFF;
	stepper.speedramp_state   = STEPPER_SPEEDRAMP_STATE_STOP;
	stepper.direction         = STEPPER_DIRECTION_FORWARD;
	stepper.time_base         = 1;
	stepper.time_base_counter = 1;

	tmc2130_set_active(false);
}

void stepper_tick(void) {
	// Nothing
}

