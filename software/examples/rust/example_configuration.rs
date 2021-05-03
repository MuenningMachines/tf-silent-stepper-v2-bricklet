use std::{error::Error, io, thread, time::Duration};
use tinkerforge::{ip_connection::IpConnection, silent_stepper_v2_bricklet::*};

const HOST: &str = "localhost";
const PORT: u16 = 4223;
const UID: &str = "XYZ"; // Change XYZ to the UID of your Silent Stepper Bricklet 2.0.

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection.
    let ss = SilentStepperV2Bricklet::new(UID, &ipcon); // Create device object.

    ipcon.connect((HOST, PORT)).recv()??; // Connect to brickd.
                                          // Don't use device before ipcon is connected.

    ss.set_motor_current(800).recv()?; // 800 mA
    ss.set_step_configuration(SILENT_STEPPER_V2_BRICKLET_STEP_RESOLUTION_8, true).recv()?; // 1/8 steps (interpolated)
    ss.set_max_velocity(2000).recv()?; // Velocity 2000 steps/s

    // Slow acceleration (500 steps/s^2),
    // Fast deacceleration (5000 steps/s^2)
    ss.set_speed_ramping(500, 5000).recv()?;

    ss.set_enabled(true).recv()?; // Enable motor power
    ss.set_steps(60000).recv()?; // Drive 60000 steps forward

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;

    // Stop motor before disabling motor power
    ss.stop().recv()?; // Request motor stop
    ss.set_speed_ramping(500, 5000).recv()?; // Fast deacceleration (5000 steps/s^2) for stopping
    thread::sleep(Duration::from_millis(400)); // Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
    ss.set_enabled(false).recv()?; // Disable motor power

    ipcon.disconnect();
    Ok(())
}
