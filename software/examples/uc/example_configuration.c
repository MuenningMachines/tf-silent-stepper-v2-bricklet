// This example is not self-contained.
// It requires usage of the example driver specific to your platform.
// See the HAL documentation.

#include "src/bindings/hal_common.h"
#include "src/bindings/bricklet_silent_stepper_v2.h"

void check(int rc, const char *msg);
void example_setup(TF_HAL *hal);
void example_loop(TF_HAL *hal);

static TF_SilentStepperV2 ss;

void example_setup(TF_HAL *hal) {
	// Create device object
	check(tf_silent_stepper_v2_create(&ss, NULL, hal), "create device object");

	check(tf_silent_stepper_v2_set_motor_current(&ss,
	                                             800), "call set_motor_current"); // 800 mA
	check(tf_silent_stepper_v2_set_step_configuration(&ss,
	                                                  TF_SILENT_STEPPER_V2_STEP_RESOLUTION_8,
	                                                  true), "call set_step_configuration"); // 1/8 steps (interpolated)
	check(tf_silent_stepper_v2_set_max_velocity(&ss,
	                                            2000), "call set_max_velocity"); // Velocity 2000 steps/s

	// Slow acceleration (500 steps/s^2),
	// Fast deacceleration (5000 steps/s^2)
	check(tf_silent_stepper_v2_set_speed_ramping(&ss, 500,
	                                             5000), "call set_speed_ramping");

	check(tf_silent_stepper_v2_set_enabled(&ss,
	                                       true), "call set_enabled"); // Enable motor power
	check(tf_silent_stepper_v2_set_steps(&ss,
	                                     60000), "call set_steps"); // Drive 60000 steps forward
}

void example_loop(TF_HAL *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
