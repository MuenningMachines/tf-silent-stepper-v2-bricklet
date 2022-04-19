// This example is not self-contained.
// It requres usage of the example driver specific to your platform.
// See the HAL documentation.

#include "bindings/hal_common.h"
#include "bindings/bricklet_silent_stepper_v2.h"

// FIXME: This example is incomplete

#define UID "XYZ" // Change XYZ to the UID of your Silent Stepper Bricklet 2.0

void check(int rc, const char* msg);

void example_setup(TF_HAL *hal);
void example_loop(TF_HAL *hal);


// Use position reached callback to program random movement
static void position_reached_handler(TF_SilentStepperV2 *device, int32_t position,
                                     void *user_data) {
	(void)device; (void)user_data; // avoid unused parameter warning

	tf_hal_printf("Position: %I32d\n", position);
}

static TF_SilentStepperV2 ss;

void example_setup(TF_HAL *hal) {
	// Create device object
	check(tf_silent_stepper_v2_create(&ss, UID, hal), "create device object");

	// Register position reached callback to function position_reached_handler
	tf_silent_stepper_v2_register_position_reached_callback(&ss,
	                                                        position_reached_handler,
	                                                        NULL);

	check(tf_silent_stepper_v2_set_step_configuration(&ss,
	                                                  TF_SILENT_STEPPER_V2_STEP_RESOLUTION_8,
	                                                  true), "call set_step_configuration"); // 1/8 steps (interpolated)
	check(tf_silent_stepper_v2_set_enabled(&ss,
	                                       true), "call set_enabled"); // Enable motor power
	check(tf_silent_stepper_v2_set_steps(&ss,
	                                     1), "call set_steps"); // Drive one step forward to get things going
}

void example_loop(TF_HAL *hal) {
	// Poll for callbacks
	tf_hal_callback_tick(hal, 0);
}
