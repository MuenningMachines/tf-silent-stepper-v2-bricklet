function octave_example_configuration()
    more off;

    HOST = "localhost";
    PORT = 4223;
    UID = "XYZ"; % Change XYZ to the UID of your Silent Stepper Bricklet 2.0

    ipcon = javaObject("com.tinkerforge.IPConnection"); % Create IP connection
    ss = javaObject("com.tinkerforge.BrickletSilentStepperV2", UID, ipcon); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    ss.setMotorCurrent(800); % 800 mA
    ss.setStepConfiguration(ss.STEP_RESOLUTION_8, true); % 1/8 steps (interpolated)
    ss.setMaxVelocity(2000); % Velocity 2000 steps/s

    % Slow acceleration (500 steps/s^2),
    % Fast deacceleration (5000 steps/s^2)
    ss.setSpeedRamping(500, 5000);

    ss.setEnabled(true); % Enable motor power
    ss.setSteps(60000); % Drive 60000 steps forward

    input("Press key to exit\n", "s");

    % Stop motor before disabling motor power
    ss.stop(); % Request motor stop
    ss.setSpeedRamping(500, 5000); % Fast deacceleration (5000 steps/s^2) for stopping
    pause(0.4); % Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
    ss.setEnabled(false); % Disable motor power

    ipcon.disconnect();
end
