/* silent-stepper-v2-bricklet
 * Copyright (C) 2020-2021 Olaf Lüke <olaf@tinkerforge.com>
 *
 * communication.h: TFP protocol message handling
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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>

#include "bricklib2/protocols/tfp/tfp.h"
#include "bricklib2/bootloader/bootloader.h"

// Default functions
BootloaderHandleMessageResponse handle_message(const void *data, void *response);
void communication_tick(void);
void communication_init(void);

// Constants

#define SILENT_STEPPER_V2_STEP_RESOLUTION_1 8
#define SILENT_STEPPER_V2_STEP_RESOLUTION_2 7
#define SILENT_STEPPER_V2_STEP_RESOLUTION_4 6
#define SILENT_STEPPER_V2_STEP_RESOLUTION_8 5
#define SILENT_STEPPER_V2_STEP_RESOLUTION_16 4
#define SILENT_STEPPER_V2_STEP_RESOLUTION_32 3
#define SILENT_STEPPER_V2_STEP_RESOLUTION_64 2
#define SILENT_STEPPER_V2_STEP_RESOLUTION_128 1
#define SILENT_STEPPER_V2_STEP_RESOLUTION_256 0

#define SILENT_STEPPER_V2_CHOPPER_MODE_SPREAD_CYCLE 0
#define SILENT_STEPPER_V2_CHOPPER_MODE_FAST_DECAY 1

#define SILENT_STEPPER_V2_FREEWHEEL_MODE_NORMAL 0
#define SILENT_STEPPER_V2_FREEWHEEL_MODE_FREEWHEELING 1
#define SILENT_STEPPER_V2_FREEWHEEL_MODE_COIL_SHORT_LS 2
#define SILENT_STEPPER_V2_FREEWHEEL_MODE_COIL_SHORT_HS 3

#define SILENT_STEPPER_V2_CURRENT_UP_STEP_INCREMENT_1 0
#define SILENT_STEPPER_V2_CURRENT_UP_STEP_INCREMENT_2 1
#define SILENT_STEPPER_V2_CURRENT_UP_STEP_INCREMENT_4 2
#define SILENT_STEPPER_V2_CURRENT_UP_STEP_INCREMENT_8 3

#define SILENT_STEPPER_V2_CURRENT_DOWN_STEP_DECREMENT_1 0
#define SILENT_STEPPER_V2_CURRENT_DOWN_STEP_DECREMENT_2 1
#define SILENT_STEPPER_V2_CURRENT_DOWN_STEP_DECREMENT_8 2
#define SILENT_STEPPER_V2_CURRENT_DOWN_STEP_DECREMENT_32 3

#define SILENT_STEPPER_V2_MINIMUM_CURRENT_HALF 0
#define SILENT_STEPPER_V2_MINIMUM_CURRENT_QUARTER 1

#define SILENT_STEPPER_V2_STALLGUARD_MODE_STANDARD 0
#define SILENT_STEPPER_V2_STALLGUARD_MODE_FILTERED 1

#define SILENT_STEPPER_V2_OPEN_LOAD_NONE 0
#define SILENT_STEPPER_V2_OPEN_LOAD_PHASE_A 1
#define SILENT_STEPPER_V2_OPEN_LOAD_PHASE_B 2
#define SILENT_STEPPER_V2_OPEN_LOAD_PHASE_AB 3

#define SILENT_STEPPER_V2_SHORT_TO_GROUND_NONE 0
#define SILENT_STEPPER_V2_SHORT_TO_GROUND_PHASE_A 1
#define SILENT_STEPPER_V2_SHORT_TO_GROUND_PHASE_B 2
#define SILENT_STEPPER_V2_SHORT_TO_GROUND_PHASE_AB 3

#define SILENT_STEPPER_V2_OVER_TEMPERATURE_NONE 0
#define SILENT_STEPPER_V2_OVER_TEMPERATURE_WARNING 1
#define SILENT_STEPPER_V2_OVER_TEMPERATURE_LIMIT 2

#define SILENT_STEPPER_V2_STATE_STOP 1
#define SILENT_STEPPER_V2_STATE_ACCELERATION 2
#define SILENT_STEPPER_V2_STATE_RUN 3
#define SILENT_STEPPER_V2_STATE_DEACCELERATION 4
#define SILENT_STEPPER_V2_STATE_DIRECTION_CHANGE_TO_FORWARD 5
#define SILENT_STEPPER_V2_STATE_DIRECTION_CHANGE_TO_BACKWARD 6

#define SILENT_STEPPER_V2_GPIO_ACTION_NONE 0
#define SILENT_STEPPER_V2_GPIO_ACTION_NORMAL_STOP_RISING_EDGE 1
#define SILENT_STEPPER_V2_GPIO_ACTION_NORMAL_STOP_FALLING_EDGE 2
#define SILENT_STEPPER_V2_GPIO_ACTION_FULL_BRAKE_RISING_EDGE 4
#define SILENT_STEPPER_V2_GPIO_ACTION_FULL_BRAKE_FALLING_EDGE 8
#define SILENT_STEPPER_V2_GPIO_ACTION_CALLBACK_RISING_EDGE 16
#define SILENT_STEPPER_V2_GPIO_ACTION_CALLBACK_FALLING_EDGE 32

#define SILENT_STEPPER_V2_ERROR_LED_CONFIG_OFF 0
#define SILENT_STEPPER_V2_ERROR_LED_CONFIG_ON 1
#define SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_HEARTBEAT 2
#define SILENT_STEPPER_V2_ERROR_LED_CONFIG_SHOW_ERROR 3

#define SILENT_STEPPER_V2_BOOTLOADER_MODE_BOOTLOADER 0
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_FIRMWARE 1
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_BOOTLOADER_WAIT_FOR_REBOOT 2
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_REBOOT 3
#define SILENT_STEPPER_V2_BOOTLOADER_MODE_FIRMWARE_WAIT_FOR_ERASE_AND_REBOOT 4

#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_OK 0
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_INVALID_MODE 1
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_NO_CHANGE 2
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_ENTRY_FUNCTION_NOT_PRESENT 3
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_DEVICE_IDENTIFIER_INCORRECT 4
#define SILENT_STEPPER_V2_BOOTLOADER_STATUS_CRC_MISMATCH 5

#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_OFF 0
#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_ON 1
#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_SHOW_HEARTBEAT 2
#define SILENT_STEPPER_V2_STATUS_LED_CONFIG_SHOW_STATUS 3

// Function and callback IDs and structs
#define FID_SET_MAX_VELOCITY 1
#define FID_GET_MAX_VELOCITY 2
#define FID_GET_CURRENT_VELOCITY 3
#define FID_SET_SPEED_RAMPING 4
#define FID_GET_SPEED_RAMPING 5
#define FID_FULL_BRAKE 6
#define FID_SET_CURRENT_POSITION 7
#define FID_GET_CURRENT_POSITION 8
#define FID_SET_TARGET_POSITION 9
#define FID_GET_TARGET_POSITION 10
#define FID_SET_STEPS 11
#define FID_GET_STEPS 12
#define FID_GET_REMAINING_STEPS 13
#define FID_SET_STEP_CONFIGURATION 14
#define FID_GET_STEP_CONFIGURATION 15
#define FID_DRIVE_FORWARD 16
#define FID_DRIVE_BACKWARD 17
#define FID_STOP 18
#define FID_GET_INPUT_VOLTAGE 19
#define FID_SET_MOTOR_CURRENT 22
#define FID_GET_MOTOR_CURRENT 23
#define FID_SET_ENABLED 24
#define FID_GET_ENABLED 25
#define FID_SET_BASIC_CONFIGURATION 26
#define FID_GET_BASIC_CONFIGURATION 27
#define FID_SET_SPREADCYCLE_CONFIGURATION 28
#define FID_GET_SPREADCYCLE_CONFIGURATION 29
#define FID_SET_STEALTH_CONFIGURATION 30
#define FID_GET_STEALTH_CONFIGURATION 31
#define FID_SET_COOLSTEP_CONFIGURATION 32
#define FID_GET_COOLSTEP_CONFIGURATION 33
#define FID_SET_MISC_CONFIGURATION 34
#define FID_GET_MISC_CONFIGURATION 35
#define FID_SET_ERROR_LED_CONFIG 36
#define FID_GET_ERROR_LED_CONFIG 37
#define FID_GET_DRIVER_STATUS 38
#define FID_SET_MINIMUM_VOLTAGE 39
#define FID_GET_MINIMUM_VOLTAGE 40
#define FID_SET_TIME_BASE 43
#define FID_GET_TIME_BASE 44
#define FID_GET_ALL_DATA 45
#define FID_SET_ALL_CALLBACK_CONFIGURATION 46
#define FID_GET_ALL_DATA_CALLBACK_CONFIGURATON 47
#define FID_SET_GPIO_CONFIGURATION 48
#define FID_GET_GPIO_CONFIGURATION 49
#define FID_SET_GPIO_ACTION 50
#define FID_GET_GPIO_ACTION 51
#define FID_GET_GPIO_STATE 52

#define FID_CALLBACK_UNDER_VOLTAGE 41
#define FID_CALLBACK_POSITION_REACHED 42
#define FID_CALLBACK_ALL_DATA 53
#define FID_CALLBACK_NEW_STATE 54
#define FID_CALLBACK_GPIO_STATE 55

typedef struct {
	TFPMessageHeader header;
	uint16_t velocity;
} __attribute__((__packed__)) SetMaxVelocity;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMaxVelocity;

typedef struct {
	TFPMessageHeader header;
	uint16_t velocity;
} __attribute__((__packed__)) GetMaxVelocity_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetCurrentVelocity;

typedef struct {
	TFPMessageHeader header;
	uint16_t velocity;
} __attribute__((__packed__)) GetCurrentVelocity_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t acceleration;
	uint16_t deacceleration;
} __attribute__((__packed__)) SetSpeedRamping;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSpeedRamping;

typedef struct {
	TFPMessageHeader header;
	uint16_t acceleration;
	uint16_t deacceleration;
} __attribute__((__packed__)) GetSpeedRamping_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) FullBrake;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) SetCurrentPosition;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetCurrentPosition;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) GetCurrentPosition_Response;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) SetTargetPosition;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetTargetPosition;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) GetTargetPosition_Response;

typedef struct {
	TFPMessageHeader header;
	int32_t steps;
} __attribute__((__packed__)) SetSteps;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSteps;

typedef struct {
	TFPMessageHeader header;
	int32_t steps;
} __attribute__((__packed__)) GetSteps_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetRemainingSteps;

typedef struct {
	TFPMessageHeader header;
	int32_t steps;
} __attribute__((__packed__)) GetRemainingSteps_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t step_resolution;
	bool interpolation;
} __attribute__((__packed__)) SetStepConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetStepConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint8_t step_resolution;
	bool interpolation;
} __attribute__((__packed__)) GetStepConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) DriveForward;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) DriveBackward;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) Stop;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetInputVoltage;

typedef struct {
	TFPMessageHeader header;
	uint16_t voltage;
} __attribute__((__packed__)) GetInputVoltage_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t current;
} __attribute__((__packed__)) SetMotorCurrent;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMotorCurrent;

typedef struct {
	TFPMessageHeader header;
	uint16_t current;
} __attribute__((__packed__)) GetMotorCurrent_Response;

typedef struct {
	TFPMessageHeader header;
	bool enabled;
} __attribute__((__packed__)) SetEnabled;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetEnabled;

typedef struct {
	TFPMessageHeader header;
	bool enabled;
} __attribute__((__packed__)) GetEnabled_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t standstill_current;
	uint16_t motor_run_current;
	uint16_t standstill_delay_time;
	uint16_t power_down_time;
	uint16_t stealth_threshold;
	uint16_t coolstep_threshold;
	uint16_t classic_threshold;
	bool high_velocity_chopper_mode;
} __attribute__((__packed__)) SetBasicConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetBasicConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint16_t standstill_current;
	uint16_t motor_run_current;
	uint16_t standstill_delay_time;
	uint16_t power_down_time;
	uint16_t stealth_threshold;
	uint16_t coolstep_threshold;
	uint16_t classic_threshold;
	bool high_velocity_chopper_mode;
} __attribute__((__packed__)) GetBasicConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t slow_decay_duration;
	bool enable_random_slow_decay;
	uint8_t fast_decay_duration;
	uint8_t hysteresis_start_value;
	int8_t hysteresis_end_value;
	int8_t sine_wave_offset;
	uint8_t chopper_mode;
	uint8_t comparator_blank_time;
	bool fast_decay_without_comparator;
} __attribute__((__packed__)) SetSpreadcycleConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetSpreadcycleConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint8_t slow_decay_duration;
	bool enable_random_slow_decay;
	uint8_t fast_decay_duration;
	uint8_t hysteresis_start_value;
	int8_t hysteresis_end_value;
	int8_t sine_wave_offset;
	uint8_t chopper_mode;
	uint8_t comparator_blank_time;
	bool fast_decay_without_comparator;
} __attribute__((__packed__)) GetSpreadcycleConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	bool enable_stealth;
	uint8_t amplitude;
	uint8_t gradient;
	bool enable_autoscale;
	bool force_symmetric;
	uint8_t freewheel_mode;
} __attribute__((__packed__)) SetStealthConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetStealthConfiguration;

typedef struct {
	TFPMessageHeader header;
	bool enable_stealth;
	uint8_t amplitude;
	uint8_t gradient;
	bool enable_autoscale;
	bool force_symmetric;
	uint8_t freewheel_mode;
} __attribute__((__packed__)) GetStealthConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t minimum_stallguard_value;
	uint8_t maximum_stallguard_value;
	uint8_t current_up_step_width;
	uint8_t current_down_step_width;
	uint8_t minimum_current;
	int8_t stallguard_threshold_value;
	uint8_t stallguard_mode;
} __attribute__((__packed__)) SetCoolstepConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetCoolstepConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint8_t minimum_stallguard_value;
	uint8_t maximum_stallguard_value;
	uint8_t current_up_step_width;
	uint8_t current_down_step_width;
	uint8_t minimum_current;
	int8_t stallguard_threshold_value;
	uint8_t stallguard_mode;
} __attribute__((__packed__)) GetCoolstepConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	bool disable_short_to_ground_protection;
	uint8_t synchronize_phase_frequency;
} __attribute__((__packed__)) SetMiscConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMiscConfiguration;

typedef struct {
	TFPMessageHeader header;
	bool disable_short_to_ground_protection;
	uint8_t synchronize_phase_frequency;
} __attribute__((__packed__)) GetMiscConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t config;
} __attribute__((__packed__)) SetErrorLEDConfig;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetErrorLEDConfig;

typedef struct {
	TFPMessageHeader header;
	uint8_t config;
} __attribute__((__packed__)) GetErrorLEDConfig_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetDriverStatus;

typedef struct {
	TFPMessageHeader header;
	uint8_t open_load;
	uint8_t short_to_ground;
	uint8_t over_temperature;
	bool motor_stalled;
	uint8_t actual_motor_current;
	bool full_step_active;
	uint8_t stallguard_result;
	uint8_t stealth_voltage_amplitude;
} __attribute__((__packed__)) GetDriverStatus_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t voltage;
} __attribute__((__packed__)) SetMinimumVoltage;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetMinimumVoltage;

typedef struct {
	TFPMessageHeader header;
	uint16_t voltage;
} __attribute__((__packed__)) GetMinimumVoltage_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t voltage;
} __attribute__((__packed__)) UnderVoltage_Callback;

typedef struct {
	TFPMessageHeader header;
	int32_t position;
} __attribute__((__packed__)) PositionReached_Callback;

typedef struct {
	TFPMessageHeader header;
	uint32_t time_base;
} __attribute__((__packed__)) SetTimeBase;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetTimeBase;

typedef struct {
	TFPMessageHeader header;
	uint32_t time_base;
} __attribute__((__packed__)) GetTimeBase_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAllData;

typedef struct {
	TFPMessageHeader header;
	uint16_t current_velocity;
	int32_t current_position;
	int32_t remaining_steps;
	uint16_t input_voltage;
	uint16_t current_consumption;
} __attribute__((__packed__)) GetAllData_Response;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
} __attribute__((__packed__)) SetAllCallbackConfiguration;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetAllDataCallbackConfiguraton;

typedef struct {
	TFPMessageHeader header;
	uint32_t period;
} __attribute__((__packed__)) GetAllDataCallbackConfiguraton_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t channel;
	uint16_t debounce;
	uint16_t stop_deceleration;
} __attribute__((__packed__)) SetGPIOConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint8_t channel;
} __attribute__((__packed__)) GetGPIOConfiguration;

typedef struct {
	TFPMessageHeader header;
	uint16_t debounce;
	uint16_t stop_deceleration;
} __attribute__((__packed__)) GetGPIOConfiguration_Response;

typedef struct {
	TFPMessageHeader header;
	uint8_t channel;
	uint32_t action;
} __attribute__((__packed__)) SetGPIOAction;

typedef struct {
	TFPMessageHeader header;
	uint8_t channel;
} __attribute__((__packed__)) GetGPIOAction;

typedef struct {
	TFPMessageHeader header;
	uint32_t action;
} __attribute__((__packed__)) GetGPIOAction_Response;

typedef struct {
	TFPMessageHeader header;
} __attribute__((__packed__)) GetGPIOState;

typedef struct {
	TFPMessageHeader header;
	uint8_t gpio_state[1];
} __attribute__((__packed__)) GetGPIOState_Response;

typedef struct {
	TFPMessageHeader header;
	uint16_t current_velocity;
	int32_t current_position;
	int32_t remaining_steps;
	uint16_t input_voltage;
	uint16_t current_consumption;
} __attribute__((__packed__)) AllData_Callback;

typedef struct {
	TFPMessageHeader header;
	uint8_t state_new;
	uint8_t state_previous;
} __attribute__((__packed__)) NewState_Callback;

typedef struct {
	TFPMessageHeader header;
	uint8_t gpio_state[1];
} __attribute__((__packed__)) GPIOState_Callback;


// Function prototypes
BootloaderHandleMessageResponse set_max_velocity(const SetMaxVelocity *data);
BootloaderHandleMessageResponse get_max_velocity(const GetMaxVelocity *data, GetMaxVelocity_Response *response);
BootloaderHandleMessageResponse get_current_velocity(const GetCurrentVelocity *data, GetCurrentVelocity_Response *response);
BootloaderHandleMessageResponse set_speed_ramping(const SetSpeedRamping *data);
BootloaderHandleMessageResponse get_speed_ramping(const GetSpeedRamping *data, GetSpeedRamping_Response *response);
BootloaderHandleMessageResponse full_brake(const FullBrake *data);
BootloaderHandleMessageResponse set_current_position(const SetCurrentPosition *data);
BootloaderHandleMessageResponse get_current_position(const GetCurrentPosition *data, GetCurrentPosition_Response *response);
BootloaderHandleMessageResponse set_target_position(const SetTargetPosition *data);
BootloaderHandleMessageResponse get_target_position(const GetTargetPosition *data, GetTargetPosition_Response *response);
BootloaderHandleMessageResponse set_steps(const SetSteps *data);
BootloaderHandleMessageResponse get_steps(const GetSteps *data, GetSteps_Response *response);
BootloaderHandleMessageResponse get_remaining_steps(const GetRemainingSteps *data, GetRemainingSteps_Response *response);
BootloaderHandleMessageResponse set_step_configuration(const SetStepConfiguration *data);
BootloaderHandleMessageResponse get_step_configuration(const GetStepConfiguration *data, GetStepConfiguration_Response *response);
BootloaderHandleMessageResponse drive_forward(const DriveForward *data);
BootloaderHandleMessageResponse drive_backward(const DriveBackward *data);
BootloaderHandleMessageResponse stop(const Stop *data);
BootloaderHandleMessageResponse get_input_voltage(const GetInputVoltage *data, GetInputVoltage_Response *response);
BootloaderHandleMessageResponse set_motor_current(const SetMotorCurrent *data);
BootloaderHandleMessageResponse get_motor_current(const GetMotorCurrent *data, GetMotorCurrent_Response *response);
BootloaderHandleMessageResponse set_enabled(const SetEnabled *data);
BootloaderHandleMessageResponse get_enabled(const GetEnabled *data, GetEnabled_Response *response);
BootloaderHandleMessageResponse set_basic_configuration(const SetBasicConfiguration *data);
BootloaderHandleMessageResponse get_basic_configuration(const GetBasicConfiguration *data, GetBasicConfiguration_Response *response);
BootloaderHandleMessageResponse set_spreadcycle_configuration(const SetSpreadcycleConfiguration *data);
BootloaderHandleMessageResponse get_spreadcycle_configuration(const GetSpreadcycleConfiguration *data, GetSpreadcycleConfiguration_Response *response);
BootloaderHandleMessageResponse set_stealth_configuration(const SetStealthConfiguration *data);
BootloaderHandleMessageResponse get_stealth_configuration(const GetStealthConfiguration *data, GetStealthConfiguration_Response *response);
BootloaderHandleMessageResponse set_coolstep_configuration(const SetCoolstepConfiguration *data);
BootloaderHandleMessageResponse get_coolstep_configuration(const GetCoolstepConfiguration *data, GetCoolstepConfiguration_Response *response);
BootloaderHandleMessageResponse set_misc_configuration(const SetMiscConfiguration *data);
BootloaderHandleMessageResponse get_misc_configuration(const GetMiscConfiguration *data, GetMiscConfiguration_Response *response);
BootloaderHandleMessageResponse set_error_led_config(const SetErrorLEDConfig *data);
BootloaderHandleMessageResponse get_error_led_config(const GetErrorLEDConfig *data, GetErrorLEDConfig_Response *response);
BootloaderHandleMessageResponse get_driver_status(const GetDriverStatus *data, GetDriverStatus_Response *response);
BootloaderHandleMessageResponse set_minimum_voltage(const SetMinimumVoltage *data);
BootloaderHandleMessageResponse get_minimum_voltage(const GetMinimumVoltage *data, GetMinimumVoltage_Response *response);
BootloaderHandleMessageResponse set_time_base(const SetTimeBase *data);
BootloaderHandleMessageResponse get_time_base(const GetTimeBase *data, GetTimeBase_Response *response);
BootloaderHandleMessageResponse get_all_data(const GetAllData *data, GetAllData_Response *response);
BootloaderHandleMessageResponse set_all_callback_configuration(const SetAllCallbackConfiguration *data);
BootloaderHandleMessageResponse get_all_data_callback_configuraton(const GetAllDataCallbackConfiguraton *data, GetAllDataCallbackConfiguraton_Response *response);
BootloaderHandleMessageResponse set_gpio_configuration(const SetGPIOConfiguration *data);
BootloaderHandleMessageResponse get_gpio_configuration(const GetGPIOConfiguration *data, GetGPIOConfiguration_Response *response);
BootloaderHandleMessageResponse set_gpio_action(const SetGPIOAction *data);
BootloaderHandleMessageResponse get_gpio_action(const GetGPIOAction *data, GetGPIOAction_Response *response);
BootloaderHandleMessageResponse get_gpio_state(const GetGPIOState *data, GetGPIOState_Response *response);

// Callbacks
bool handle_under_voltage_callback(void);
bool handle_position_reached_callback(void);
bool handle_all_data_callback(void);
bool handle_new_state_callback(void);
bool handle_gpio_state_callback(void);

#define COMMUNICATION_CALLBACK_TICK_WAIT_MS 1
#define COMMUNICATION_CALLBACK_HANDLER_NUM 5
#define COMMUNICATION_CALLBACK_LIST_INIT \
	handle_under_voltage_callback, \
	handle_position_reached_callback, \
	handle_all_data_callback, \
	handle_new_state_callback, \
	handle_gpio_state_callback, \


#endif
