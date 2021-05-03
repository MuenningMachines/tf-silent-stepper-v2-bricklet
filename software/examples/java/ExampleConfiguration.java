import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickletSilentStepperV2;

public class ExampleConfiguration {
	private static final String HOST = "localhost";
	private static final int PORT = 4223;

	// Change XYZ to the UID of your Silent Stepper Bricklet 2.0
	private static final String UID = "XYZ";

	// Note: To make the example code cleaner we do not handle exceptions. Exceptions
	//       you might normally want to catch are described in the documentation
	public static void main(String args[]) throws Exception {
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletSilentStepperV2 ss =
		  new BrickletSilentStepperV2(UID, ipcon); // Create device object

		ipcon.connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		ss.setMotorCurrent(800); // 800 mA
		ss.setStepConfiguration(BrickletSilentStepperV2.STEP_RESOLUTION_8,
		                        true); // 1/8 steps (interpolated)
		ss.setMaxVelocity(2000); // Velocity 2000 steps/s

		// Slow acceleration (500 steps/s^2),
		// Fast deacceleration (5000 steps/s^2)
		ss.setSpeedRamping(500, 5000);

		ss.setEnabled(true); // Enable motor power
		ss.setSteps(60000); // Drive 60000 steps forward

		System.out.println("Press key to exit"); System.in.read();

		// Stop motor before disabling motor power
		ss.stop(); // Request motor stop
		ss.setSpeedRamping(500, 5000); // Fast deacceleration (5000 steps/s^2) for stopping
		Thread.sleep(400); // Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
		ss.setEnabled(false); // Disable motor power

		ipcon.disconnect();
	}
}
