package main

import (
	"fmt"
	"github.com/Tinkerforge/go-api-bindings/ipconnection"
	"github.com/Tinkerforge/go-api-bindings/silent_stepper_v2_bricklet"
	"math/rand"
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

	rng := rand.New(rand.NewSource(time.Now().UnixNano()))

	ss.RegisterPositionReachedCallback(func(position int32) {
		steps := int32(0)
		if rng.Int31()%2 == 0 {
			steps = rng.Int31n(4001) + 1000
			fmt.Printf("Driving forward: %d steps\n", steps)
		} else {
			steps = -(rng.Int31n(4001) + 1000)
			fmt.Printf("Driving backward: %d steps\n", steps)
		}

		vel := uint16(rng.Int31n(1800) + 200)
		acc := uint16(rng.Int31n(900) + 100)
		dec := uint16(rng.Int31n(900) + 100)

		fmt.Printf("Configuration: (vel, acc, dec): (%d, %d, %d)\n", vel, acc, dec)
		ss.SetSpeedRamping(acc, dec)
		ss.SetMaxVelocity(vel)
		ss.SetSteps(steps)
	})

	ss.SetStepConfiguration(silent_stepper_v2_bricklet.StepResolution8,
		true) // 1/8 steps (interpolated)
	ss.SetEnabled(true) // Enable motor power
	ss.SetSteps(1)      // Drive one step forward to get things going

	fmt.Print("Press enter to exit.")
	fmt.Scanln()

	// Stop motor before disabling motor power
	ss.Stop()                          // Request motor stop
	ss.SetSpeedRamping(500, 5000)      // Fast deacceleration (5000 steps/s^2) for stopping
	time.Sleep(400 * time.Millisecond) // Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
	ss.SetEnabled(false)               // Disable motor power
}
