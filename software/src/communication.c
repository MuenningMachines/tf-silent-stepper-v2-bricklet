/* silent-stepper-v2-bricklet
 * Copyright (C) 2020 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.c: TFP protocol message handling
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

#include "communication.h"

#include "bricklib2/utility/communication_callback.h"
#include "bricklib2/utility/util_definitions.h"
#include "bricklib2/utility/sqrt.h"
#include "bricklib2/protocols/tfp/tfp.h"

#include "configs/config_tmc2130.h"

#include "stepper.h"
#include "tmc2130.h"
#include "voltage.h"
#include "gpio.h"

extern TMC2130HighLevel tmc2130_high_level;

extern TMC2130RegGSTAT tmc2130_reg_gstat;
extern TMC2130RegTSTEP tmc2130_reg_tstep;
extern TMC2130RegDRV_STATUS tmc2130_reg_drv_status;
extern TMC2130RegPWM_SCALE tmc2130_reg_pwm_scale;

extern TMC2130RegIHOLD_IRUN tmc2130_reg_ihold_run;
extern TMC2130RegTPOWERDOWN tmc2130_reg_tpowerdown;
extern TMC2130RegTPWMTHRS tmc2130_reg_tpwmthrs;
extern TMC2130RegTCOOLTHRS tmc2130_reg_tcoolthrs;
extern TMC2130RegTHIGH tmc2130_reg_thigh;
extern TMC2130RegCOOLCONF tmc2130_reg_coolconf;
extern TMC2130RegPWMCONF tmc2130_reg_pwmconf;
extern TMC2130RegGCONF tmc2130_reg_gconf;
extern TMC2130RegCHOPCONF tmc2130_reg_chopconf;


extern TMC2130HighLevel tmc2130_high_level;

extern TMC2130RegGSTAT tmc2130_reg_gstat;
extern TMC2130RegTSTEP tmc2130_reg_tstep;
extern TMC2130RegDRV_STATUS tmc2130_reg_drv_status;
extern TMC2130RegPWM_SCALE tmc2130_reg_pwm_scale;

extern TMC2130RegIHOLD_IRUN tmc2130_reg_ihold_run;
extern TMC2130RegTPOWERDOWN tmc2130_reg_tpowerdown;
extern TMC2130RegTPWMTHRS tmc2130_reg_tpwmthrs;
extern TMC2130RegTCOOLTHRS tmc2130_reg_tcoolthrs;
extern TMC2130RegTHIGH tmc2130_reg_thigh;
extern TMC2130RegCOOLCONF tmc2130_reg_coolconf;
extern TMC2130RegPWMCONF tmc2130_reg_pwmconf;
extern TMC2130RegGCONF tmc2130_reg_gconf;
extern TMC2130RegCHOPCONF tmc2130_reg_chopconf;

BootloaderHandleMessageResponse handle_message(const void *message, void *response) {
	switch(tfp_get_fid_from_message(message)) {
		case FID_SET_MAX_VELOCITY: return set_max_velocity(message);
		case FID_GET_MAX_VELOCITY: return get_max_velocity(message, response);
		case FID_GET_CURRENT_VELOCITY: return get_current_velocity(message, response);
		case FID_SET_SPEED_RAMPING: return set_speed_ramping(message);
		case FID_GET_SPEED_RAMPING: return get_speed_ramping(message, response);
		case FID_FULL_BRAKE: return full_brake(message);
		case FID_SET_CURRENT_POSITION: return set_current_position(message);
		case FID_GET_CURRENT_POSITION: return get_current_position(message, response);
		case FID_SET_TARGET_POSITION: return set_target_position(message);
		case FID_GET_TARGET_POSITION: return get_target_position(message, response);
		case FID_SET_STEPS: return set_steps(message);
		case FID_GET_STEPS: return get_steps(message, response);
		case FID_GET_REMAINING_STEPS: return get_remaining_steps(message, response);
		case FID_SET_STEP_CONFIGURATION: return set_step_configuration(message);
		case FID_GET_STEP_CONFIGURATION: return get_step_configuration(message, response);
		case FID_DRIVE_FORWARD: return drive_forward(message);
		case FID_DRIVE_BACKWARD: return drive_backward(message);
		case FID_STOP: return stop(message);
		case FID_GET_INPUT_VOLTAGE: return get_input_voltage(message, response);
		case FID_SET_MOTOR_CURRENT: return set_motor_current(message);
		case FID_GET_MOTOR_CURRENT: return get_motor_current(message, response);
		case FID_SET_ENABLED: return set_enabled(message);
		case FID_GET_ENABLED: return get_enabled(message, response);
		case FID_SET_BASIC_CONFIGURATION: return set_basic_configuration(message);
		case FID_GET_BASIC_CONFIGURATION: return get_basic_configuration(message, response);
		case FID_SET_SPREADCYCLE_CONFIGURATION: return set_spreadcycle_configuration(message);
		case FID_GET_SPREADCYCLE_CONFIGURATION: return get_spreadcycle_configuration(message, response);
		case FID_SET_STEALTH_CONFIGURATION: return set_stealth_configuration(message);
		case FID_GET_STEALTH_CONFIGURATION: return get_stealth_configuration(message, response);
		case FID_SET_COOLSTEP_CONFIGURATION: return set_coolstep_configuration(message);
		case FID_GET_COOLSTEP_CONFIGURATION: return get_coolstep_configuration(message, response);
		case FID_SET_MISC_CONFIGURATION: return set_misc_configuration(message);
		case FID_GET_MISC_CONFIGURATION: return get_misc_configuration(message, response);
		case FID_SET_ERROR_LED_CONFIG: return set_error_led_config(message);
		case FID_GET_ERROR_LED_CONFIG: return get_error_led_config(message, response);
		case FID_GET_DRIVER_STATUS: return get_driver_status(message, response);
		case FID_SET_MINIMUM_VOLTAGE: return set_minimum_voltage(message);
		case FID_GET_MINIMUM_VOLTAGE: return get_minimum_voltage(message, response);
		case FID_SET_TIME_BASE: return set_time_base(message);
		case FID_GET_TIME_BASE: return get_time_base(message, response);
		case FID_GET_ALL_DATA: return get_all_data(message, response);
		case FID_SET_GPIO_CONFIGURATION: return set_gpio_configuration(message);
		case FID_GET_GPIO_CONFIGURATION: return get_gpio_configuration(message, response);
		case FID_SET_GPIO_ACTION: return set_gpio_action(message);
		case FID_GET_GPIO_ACTION: return get_gpio_action(message, response);
		case FID_GET_GPIO_STATE: return get_gpio_state(message, response);
		case FID_SET_ALL_CALLBACK_CONFIGURATION: return set_all_callback_configuration(message);
		case FID_GET_ALL_DATA_CALLBACK_CONFIGURATON: return get_all_data_callback_configuraton(message, response);
		default: return HANDLE_MESSAGE_RESPONSE_NOT_SUPPORTED;
	}
}

BootloaderHandleMessageResponse set_max_velocity(const SetMaxVelocity *data) {
	uint32_t old_velocity_goal = stepper.velocity_goal;
	stepper.velocity_goal = data->velocity;

	if(stepper.state == STEPPER_STATE_DRIVE && old_velocity_goal == 0) {
		stepper_set_next_timer(stepper.velocity);
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_max_velocity(const GetMaxVelocity *data, GetMaxVelocity_Response *response) {
	response->header.length = sizeof(GetMaxVelocity_Response);
	response->velocity      = stepper.velocity_goal;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_current_velocity(const GetCurrentVelocity *data, GetCurrentVelocity_Response *response) {
	response->header.length = sizeof(GetCurrentVelocity_Response);
	response->velocity      = stepper.velocity > 0xFFFF ? 0xFFFF : stepper.velocity;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_speed_ramping(const SetSpeedRamping *data) {
	stepper.acceleration_sqrt = sqrt_integer_precise(data->acceleration);
	stepper.acceleration      = data->acceleration;
	stepper.deceleration      = data->deacceleration;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_speed_ramping(const GetSpeedRamping *data, GetSpeedRamping_Response *response) {
	response->header.length  = sizeof(GetSpeedRamping_Response);
	response->acceleration   = stepper.acceleration;
	response->deacceleration = stepper.deceleration;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse full_brake(const FullBrake *data) {
	stepper_full_brake();

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse set_current_position(const SetCurrentPosition *data) {
	stepper.position = data->position;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_current_position(const GetCurrentPosition *data, GetCurrentPosition_Response *response) {
	response->header.length = sizeof(GetCurrentPosition_Response);
	response->position      = stepper.position;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_target_position(const SetTargetPosition *data) {
	if(stepper_is_currently_running() || stepper.state == STEPPER_STATE_OFF) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	stepper.state = STEPPER_STATE_TARGET;
	stepper.target_position = data->position;
	stepper_make_step_speedramp(stepper.target_position - stepper.position);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_target_position(const GetTargetPosition *data, GetTargetPosition_Response *response) {
	response->header.length = sizeof(GetTargetPosition_Response);
	response->position      = stepper.target_position;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_steps(const SetSteps *data) {
	if(stepper_is_currently_running() || stepper.state == STEPPER_STATE_OFF) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}	

	stepper.state = STEPPER_STATE_STEPS;
	stepper.steps = data->steps;
	stepper_make_step_speedramp(data->steps);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_steps(const GetSteps *data, GetSteps_Response *response) {
	response->header.length = sizeof(GetSteps_Response);
	response->steps         = stepper.steps;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_remaining_steps(const GetRemainingSteps *data, GetRemainingSteps_Response *response) {
	response->header.length = sizeof(GetRemainingSteps_Response);
	response->steps         = stepper_get_remaining_steps();

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_step_configuration(const SetStepConfiguration *data) {
	if(data->step_resolution > SILENT_STEPPER_V2_STEP_RESOLUTION_1) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc2130_reg_chopconf.bit.mres   = data->step_resolution;
	tmc2130_reg_chopconf.bit.intpol = data->interpolation;

	tmc2130.register_to_write_mask |= TMC2130_REG_CHOPCONF_BIT;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_step_configuration(const GetStepConfiguration *data, GetStepConfiguration_Response *response) {
	response->header.length   = sizeof(GetStepConfiguration_Response);
	response->step_resolution = tmc2130_reg_chopconf.bit.mres;
	response->interpolation   = tmc2130_reg_chopconf.bit.intpol;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse drive_forward(const DriveForward *data) {
	if((stepper.state == STEPPER_STATE_STEPS) || (stepper.state == STEPPER_STATE_TARGET) || (stepper.state == STEPPER_STATE_OFF)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	stepper_make_drive_speedramp(STEPPER_SPEEDRAMP_STATE_FORWARD);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse drive_backward(const DriveBackward *data) {
	if((stepper.state == STEPPER_STATE_STEPS) || (stepper.state == STEPPER_STATE_TARGET) || (stepper.state == STEPPER_STATE_OFF)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	stepper_make_drive_speedramp(STEPPER_SPEEDRAMP_STATE_BACKWARD);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse stop(const Stop *data) {
	if(stepper.state == STEPPER_STATE_OFF) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	stepper_make_drive_speedramp(STEPPER_SPEEDRAMP_STATE_STOP);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_input_voltage(const GetInputVoltage *data, GetInputVoltage_Response *response) {
	response->header.length = sizeof(GetInputVoltage_Response);
	response->voltage       = voltage.value;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_motor_current(const SetMotorCurrent *data) {
	tmc2130_set_output_current(data->current);

	// update output current dependent registers
	tmc2130_reg_ihold_run.bit.ihold = MIN(SCALE(tmc2130_high_level.standstill_current, 0, stepper.output_current, 0, 31), 31);
	tmc2130_reg_ihold_run.bit.irun  = MIN(SCALE(tmc2130_high_level.motor_run_current, 0, stepper.output_current, 0, 31), 31);
	tmc2130.register_to_write_mask |= TMC2130_REG_IHOLD_IRUN_BIT;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_motor_current(const GetMotorCurrent *data, GetMotorCurrent_Response *response) {
	response->header.length = sizeof(GetMotorCurrent_Response);
	response->current       = stepper.output_current;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_enabled(const SetEnabled *data) {
	if(data->enabled) {
		stepper_enable();
	} else {
		stepper_disable();
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_enabled(const GetEnabled *data, GetEnabled_Response *response) {
	response->header.length = sizeof(GetEnabled_Response);
	response->enabled       = stepper.state != STEPPER_STATE_OFF;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_basic_configuration(const SetBasicConfiguration *data) {
	if((data->power_down_time > 5222) ||
	   (data->standstill_delay_time > 307)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}


	tmc2130_high_level.standstill_current    = data->standstill_current;
	tmc2130_high_level.motor_run_current     = data->motor_run_current;
	tmc2130_high_level.standstill_delay_time = data->standstill_delay_time;
	tmc2130_high_level.power_down_time       = data->power_down_time;
	tmc2130_high_level.stealth_threshold     = data->stealth_threshold;
	tmc2130_high_level.coolstep_threshold    = data->coolstep_threshold;
	tmc2130_high_level.classic_threshold     = data->classic_threshold;

	tmc2130_reg_ihold_run.bit.ihold       = MIN(SCALE(data->standstill_current, 0, stepper.output_current, 0, 31), 31);
	tmc2130_reg_ihold_run.bit.irun        = MIN(SCALE(data->motor_run_current, 0, stepper.output_current, 0, 31), 31);
	tmc2130_reg_ihold_run.bit.ihold_delay = MIN((data->standstill_delay_time*100)/2048, 15);
	tmc2130_reg_tpowerdown.bit.delay      = MIN((data->power_down_time*100)/2048, 255);
	tmc2130_reg_tpwmthrs.bit.velocity     = MIN(TMC2130_CLOCK_FREQUENCY/(data->stealth_threshold*256), 0xfffff);
	tmc2130_reg_tcoolthrs.bit.velocity    = MIN(TMC2130_CLOCK_FREQUENCY/(data->coolstep_threshold*256), 0xfffff);
	tmc2130_reg_thigh.bit.velocity        = MIN(TMC2130_CLOCK_FREQUENCY/(data->classic_threshold*256), 0xfffff);
	tmc2130_reg_chopconf.bit.vhighchm     = data->high_velocity_chopper_mode;

	tmc2130.register_to_write_mask |= (TMC2130_REG_IHOLD_IRUN_BIT | TMC2130_REG_TPOWERDOWN_BIT | TMC2130_REG_TPWMTHRS_BIT | TMC2130_REG_TCOOLTHRS_BIT | TMC2130_REG_THIGH_BIT | TMC2130_REG_CHOPCONF_BIT);
	
	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_basic_configuration(const GetBasicConfiguration *data, GetBasicConfiguration_Response *response) {
	response->header.length = sizeof(GetBasicConfiguration_Response);
	response->standstill_current         = tmc2130_high_level.standstill_current;
	response->motor_run_current          = tmc2130_high_level.motor_run_current;
	response->standstill_delay_time      = tmc2130_high_level.standstill_delay_time;
	response->power_down_time            = tmc2130_high_level.power_down_time;
	response->stealth_threshold          = tmc2130_high_level.stealth_threshold;
	response->coolstep_threshold         = tmc2130_high_level.coolstep_threshold;
	response->classic_threshold          = tmc2130_high_level.classic_threshold;
	response->high_velocity_chopper_mode = tmc2130_reg_chopconf.bit.vhighchm;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_spreadcycle_configuration(const SetSpreadcycleConfiguration *data) {
	if((data->slow_decay_duration > 15) ||
	   (data->fast_decay_duration > 15) ||
	   (data->hysteresis_start_value > 7) ||
	   (data->hysteresis_end_value < -3 || data->hysteresis_end_value > 12) ||
	   (data->sine_wave_offset < -3 || data->sine_wave_offset > 12) ||
	   (data->chopper_mode > 1) ||
	   (data->comparator_blank_time > 3)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc2130_reg_chopconf.bit.toff      = data->slow_decay_duration;
	tmc2130_reg_chopconf.bit.rndtf     = data->enable_random_slow_decay;
	if(data->chopper_mode) {
		tmc2130_reg_chopconf.bit.hstrt = data->fast_decay_duration & 0b111;
		tmc2130_reg_chopconf.bit.fd3   = (data->fast_decay_duration >> 3) & 0b1;
		tmc2130_reg_chopconf.bit.hend  = data->sine_wave_offset; // TODO: handle signednes correctly!
	} else {
		tmc2130_reg_chopconf.bit.hstrt = data->hysteresis_start_value & 0b111;
		tmc2130_reg_chopconf.bit.fd3   = 0;
		tmc2130_reg_chopconf.bit.hend  = data->hysteresis_end_value; // TODO: handle signednes correctly!
	}
	tmc2130_reg_chopconf.bit.chm       = data->chopper_mode;
	tmc2130_reg_chopconf.bit.tbl       = data->comparator_blank_time;
	tmc2130_reg_chopconf.bit.disfdcc   = data->fast_decay_without_comparator;

	tmc2130.register_to_write_mask |= TMC2130_REG_CHOPCONF_BIT;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_spreadcycle_configuration(const GetSpreadcycleConfiguration *data, GetSpreadcycleConfiguration_Response *response) {
	response->header.length                 = sizeof(GetSpreadcycleConfiguration_Response);
	response->slow_decay_duration           = tmc2130_reg_chopconf.bit.toff;
	response->enable_random_slow_decay      = tmc2130_reg_chopconf.bit.rndtf;
	if(tmc2130_reg_chopconf.bit.chm) {
		response->fast_decay_duration       = tmc2130_reg_chopconf.bit.hstrt | (tmc2130_reg_chopconf.bit.fd3 << 3);
		response->sine_wave_offset          = tmc2130_reg_chopconf.bit.hend;
		response->hysteresis_start_value    = 0;
		response->hysteresis_end_value      = 0;
	} else {
		response->fast_decay_duration       = 0;
		response->sine_wave_offset          = 0;
		response->hysteresis_start_value    = tmc2130_reg_chopconf.bit.hstrt;
		response->hysteresis_end_value      = tmc2130_reg_chopconf.bit.hend;
	}
	response->chopper_mode                  = tmc2130_reg_chopconf.bit.chm;
	response->comparator_blank_time         = tmc2130_reg_chopconf.bit.tbl;
	response->fast_decay_without_comparator = tmc2130_reg_chopconf.bit.disfdcc;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_stealth_configuration(const SetStealthConfiguration *data) {
	if(data->freewheel_mode > 3) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc2130_reg_gconf.bit.en_pwm_mode     = data->enable_stealth;
	tmc2130_reg_pwmconf.bit.pwm_ampl      = data->amplitude;
	tmc2130_reg_pwmconf.bit.pwm_grad      = data->gradient;
	tmc2130_reg_pwmconf.bit.pwm_autoscale = data->enable_autoscale;
	tmc2130_reg_pwmconf.bit.pwm_symmetric = data->force_symmetric;
	tmc2130_reg_pwmconf.bit.freewheel     = data->freewheel_mode;

	tmc2130.register_to_write_mask |= (TMC2130_REG_GCONF_BIT | TMC2130_REG_PWMCONF_BIT);

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_stealth_configuration(const GetStealthConfiguration *data, GetStealthConfiguration_Response *response) {
	response->header.length    = sizeof(GetStealthConfiguration_Response);
	response->enable_stealth   = tmc2130_reg_gconf.bit.en_pwm_mode;
	response->amplitude        = tmc2130_reg_pwmconf.bit.pwm_ampl;
	response->gradient         = tmc2130_reg_pwmconf.bit.pwm_grad;
	response->enable_autoscale = tmc2130_reg_pwmconf.bit.pwm_autoscale;
	response->force_symmetric  = tmc2130_reg_pwmconf.bit.pwm_symmetric;
	response->freewheel_mode   = tmc2130_reg_pwmconf.bit.freewheel;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_coolstep_configuration(const SetCoolstepConfiguration *data) {
	if((data->minimum_stallguard_value > 15) ||
	   (data->maximum_stallguard_value > 15) ||
	   (data->current_up_step_width > 3) ||
	   (data->current_down_step_width > 3) ||
	   (data->minimum_current > 1) ||
	   (data->stallguard_threshold_value < -64 || data->stallguard_threshold_value > 63) ||
	   (data->stallguard_mode > 1)) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc2130_reg_coolconf.bit.semin  = data->minimum_stallguard_value;
	tmc2130_reg_coolconf.bit.semax  = data->maximum_stallguard_value;
	tmc2130_reg_coolconf.bit.seup   = data->current_up_step_width;
	tmc2130_reg_coolconf.bit.sedn   = data->current_down_step_width;
	tmc2130_reg_coolconf.bit.seimin = data->minimum_current;
	tmc2130_reg_coolconf.bit.sgt    = data->stallguard_threshold_value;
	tmc2130_reg_coolconf.bit.sfilt  = data->stallguard_mode;

	tmc2130.register_to_write_mask |= TMC2130_REG_COOLCONF_BIT;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_coolstep_configuration(const GetCoolstepConfiguration *data, GetCoolstepConfiguration_Response *response) {
	response->header.length              = sizeof(GetCoolstepConfiguration_Response);
	response->minimum_stallguard_value   = tmc2130_reg_coolconf.bit.semin;
	response->maximum_stallguard_value   = tmc2130_reg_coolconf.bit.semax;
	response->current_up_step_width      = tmc2130_reg_coolconf.bit.seup;
	response->current_down_step_width    = tmc2130_reg_coolconf.bit.sedn;
	response->minimum_current            = tmc2130_reg_coolconf.bit.seimin;
	response->stallguard_threshold_value = tmc2130_reg_coolconf.bit.sgt;
	response->stallguard_mode            = tmc2130_reg_coolconf.bit.sfilt;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_misc_configuration(const SetMiscConfiguration *data) {
	if(data->synchronize_phase_frequency > 15) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc2130_reg_chopconf.bit.diss2g = data->disable_short_to_ground_protection;
	tmc2130_reg_chopconf.bit.sync   = data->synchronize_phase_frequency;

	tmc2130.register_to_write_mask |= TMC2130_REG_CHOPCONF_BIT;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_misc_configuration(const GetMiscConfiguration *data, GetMiscConfiguration_Response *response) {
	response->header.length                      = sizeof(GetMiscConfiguration_Response);
	response->disable_short_to_ground_protection = tmc2130_reg_chopconf.bit.diss2g;
	response->synchronize_phase_frequency        = tmc2130_reg_chopconf.bit.sync;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_error_led_config(const SetErrorLEDConfig *data) {
	if(data->config > SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	tmc2130.error_led_flicker_state.config = data->config;
	switch(data->config) {
		case SILENT_STEPPER_V2_ERROR_LED_CONFIG_OFF:
			XMC_GPIO_SetOutputHigh(TMC2130_ERROR_LED_PIN);
			break;

		case SILENT_STEPPER_V2_ERROR_LED_CONFIG_ON:
			XMC_GPIO_SetOutputLow(TMC2130_ERROR_LED_PIN);
			break;

		default: break;
	}

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_error_led_config(const GetErrorLEDConfig *data, GetErrorLEDConfig_Response *response) {
	response->header.length = sizeof(GetErrorLEDConfig_Response);
	response->config        = tmc2130.error_led_flicker_state.config;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_driver_status(const GetDriverStatus *data, GetDriverStatus_Response *response) {
	response->header.length             = sizeof(GetDriverStatus_Response);

	response->open_load                 = (tmc2130_reg_drv_status.bit.ola << 0) | (tmc2130_reg_drv_status.bit.olb << 1);
	response->short_to_ground           = (tmc2130_reg_drv_status.bit.s2ga << 0) | (tmc2130_reg_drv_status.bit.s2gb << 1);
	response->over_temperature          = 0;
	if(tmc2130_reg_drv_status.bit.otpw) {
		response->over_temperature      = 1;
	}
	if(tmc2130_reg_drv_status.bit.ot) {
		response->over_temperature      = 2;
	}
	response->motor_stalled             = tmc2130_reg_drv_status.bit.stall_guard;
	response->actual_motor_current      = tmc2130_reg_drv_status.bit.cs_actual;
	response->full_step_active          = tmc2130_reg_drv_status.bit.fsactive;
	response->stallguard_result         = tmc2130_reg_drv_status.bit.sg_result;
	response->stealth_voltage_amplitude = tmc2130_reg_pwm_scale.bit.amplitude_scalar;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_minimum_voltage(const SetMinimumVoltage *data) {
	stepper.minimum_voltage = data->voltage;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_minimum_voltage(const GetMinimumVoltage *data, GetMinimumVoltage_Response *response) {
	response->header.length = sizeof(GetMinimumVoltage_Response);
	response->voltage       = stepper.minimum_voltage;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_time_base(const SetTimeBase *data) {
	stepper.time_base = data->time_base;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_time_base(const GetTimeBase *data, GetTimeBase_Response *response) {
	response->header.length = sizeof(GetTimeBase_Response);
	response->time_base     = stepper.time_base;

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_all_data(const GetAllData *data, GetAllData_Response *response) {
	response->header.length = sizeof(GetAllData_Response);
	response->current_velocity    = stepper.velocity > 0xFFFF ? 0xFFFF : stepper.velocity;
	response->current_position    = stepper.position;
	response->remaining_steps     = stepper_get_remaining_steps();
	response->input_voltage       = voltage.value;
	if(stepper.state == STEPPER_STATE_OFF) {
		response->current_consumption = 0;
	} else {
		response->current_consumption = (tmc2130_high_level.motor_run_current * (tmc2130_reg_drv_status.bit.cs_actual + 1)) / 32;
	}

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_all_callback_configuration(const SetAllCallbackConfiguration *data) {
	// TODO

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_all_data_callback_configuraton(const GetAllDataCallbackConfiguraton *data, GetAllDataCallbackConfiguraton_Response *response) {
	response->header.length = sizeof(GetAllDataCallbackConfiguraton_Response);
	// TODO

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_gpio_configuration(const SetGPIOConfiguration *data) {
	if(data->channel >= GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	gpio.debounce[data->channel]          = data->debounce;
	gpio.stop_deceleration[data->channel] = data->stop_deceleration;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_gpio_configuration(const GetGPIOConfiguration *data, GetGPIOConfiguration_Response *response) {
	if(data->channel >= GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	response->header.length     = sizeof(GetGPIOConfiguration_Response);
	response->debounce          = gpio.debounce[data->channel];
	response->stop_deceleration = gpio.stop_deceleration[data->channel];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse set_gpio_action(const SetGPIOAction *data) {
	if(data->channel >= GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	gpio.action[data->channel] = data->action;

	return HANDLE_MESSAGE_RESPONSE_EMPTY;
}

BootloaderHandleMessageResponse get_gpio_action(const GetGPIOAction *data, GetGPIOAction_Response *response) {
	if(data->channel >= GPIO_CHANNEL_NUM) {
		return HANDLE_MESSAGE_RESPONSE_INVALID_PARAMETER;
	}

	response->header.length = sizeof(GetGPIOAction_Response);
	response->action        = gpio.action[data->channel];

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}

BootloaderHandleMessageResponse get_gpio_state(const GetGPIOState *data, GetGPIOState_Response *response) {
	response->header.length = sizeof(GetGPIOState_Response);
	response->gpio_state[0] = gpio.last_interrupt_value[0] | (gpio.last_interrupt_value[1] << 1);

	return HANDLE_MESSAGE_RESPONSE_NEW_MESSAGE;
}



bool handle_under_voltage_callback(void) {
	static bool is_buffered = false;
	static UnderVoltage_Callback cb;

	if(!is_buffered) {
		tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(UnderVoltage_Callback), FID_CALLBACK_UNDER_VOLTAGE);
		// TODO: Implement UnderVoltage callback handling

		return false;
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(UnderVoltage_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_position_reached_callback(void) {
	static bool is_buffered = false;
	static PositionReached_Callback cb;

	if(!is_buffered) {
		tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(PositionReached_Callback), FID_CALLBACK_POSITION_REACHED);
		// TODO: Implement PositionReached callback handling

		return false;
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(PositionReached_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_all_data_callback(void) {
	static bool is_buffered = false;
	static AllData_Callback cb;

	if(!is_buffered) {
		tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(AllData_Callback), FID_CALLBACK_ALL_DATA);
		// TODO: Implement AllData callback handling

		return false;
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(AllData_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

bool handle_new_state_callback(void) {
	static bool is_buffered = false;
	static NewState_Callback cb;

	if(!is_buffered) {
		tfp_make_default_header(&cb.header, bootloader_get_uid(), sizeof(NewState_Callback), FID_CALLBACK_NEW_STATE);
		// TODO: Implement NewState callback handling

		return false;
	}

	if(bootloader_spitfp_is_send_possible(&bootloader_status.st)) {
		bootloader_spitfp_send_ack_and_message(&bootloader_status, (uint8_t*)&cb, sizeof(NewState_Callback));
		is_buffered = false;
		return true;
	} else {
		is_buffered = true;
	}

	return false;
}

void communication_tick(void) {
	communication_callback_tick();
}

void communication_init(void) {
	communication_callback_init();
}
