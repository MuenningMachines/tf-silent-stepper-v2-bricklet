#!/usr/bin/perl

use strict;
use Tinkerforge::IPConnection;
use Tinkerforge::BrickletSilentStepperV2;

use constant HOST => 'localhost';
use constant PORT => 4223;
use constant UID => 'XYZ'; # Change XYZ to the UID of your Silent Stepper Bricklet 2.0

my $ipcon = Tinkerforge::IPConnection->new(); # Create IP connection
my $ss = Tinkerforge::BrickletSilentStepperV2->new(&UID, $ipcon); # Create device object

$ipcon->connect(&HOST, &PORT); # Connect to brickd
# Don't use device before ipcon is connected

$ss->set_motor_current(800); # 800 mA
$ss->set_step_configuration($ss->STEP_RESOLUTION_8, 1); # 1/8 steps (interpolated)
$ss->set_max_velocity(2000); # Velocity 2000 steps/s

# Slow acceleration (500 steps/s^2),
# Fast deacceleration (5000 steps/s^2)
$ss->set_speed_ramping(500, 5000);

$ss->set_enabled(1); # Enable motor power
$ss->set_steps(60000); # Drive 60000 steps forward

print "Press key to exit\n";
<STDIN>;

# Stop motor before disabling motor power
$ss->stop(); # Request motor stop
$ss->set_speed_ramping(500, 5000); # Fast deacceleration (5000 steps/s^2) for stopping
select(undef, undef, undef, 0.4); # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
$ss->set_enabled(0); # Disable motor power

$ipcon->disconnect();
