package main

import (
	"fmt"
	"github.com/Tinkerforge/go-api-bindings/ipconnection"
	"github.com/Tinkerforge/go-api-bindings/silent_stepper_v2_bricklet"
	"time"
)

const ADDR string = "localhost:4223"
const UID string = "XYZ" // Change XYZ to the UID of your Silent Stepper Bricklet 2.0.

func main() {
	ipcon := ipconnection.New()
	defer ipcon.Close()
	ss, _ := silent_stepper_v2_bricklet.New(UID, &ipcon) // Create device object.

	ipcon.Connect(ADDR) // Connect to brickd.
	defer ipcon.Disconnect()
	// Don't use device before ipcon is connected.

	ss.SetMotorCurrent(800) // 800 mA
	ss.SetStepConfiguration(silent_stepper_v2_bricklet.StepResolution8,
		true) // 1/8 steps (interpolated)
	ss.SetMaxVelocity(2000) // Velocity 2000 steps/s

	// Slow acceleration (500 steps/s^2),
	// Fast deacceleration (5000 steps/s^2)
	ss.SetSpeedRamping(500, 5000)

	ss.SetEnabled(true) // Enable motor power
	ss.SetSteps(60000)  // Drive 60000 steps forward

	fmt.Print("Press enter to exit.")
	fmt.Scanln()

	// Stop motor before disabling motor power
	ss.Stop()                          // Request motor stop
	ss.SetSpeedRamping(500, 5000)      // Fast deacceleration (5000 steps/s^2) for stopping
	time.Sleep(400 * time.Millisecond) // Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
	ss.SetEnabled(false)               // Disable motor power
}
