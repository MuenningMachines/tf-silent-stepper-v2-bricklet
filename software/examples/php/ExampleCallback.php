<?php

require_once('Tinkerforge/IPConnection.php');
require_once('Tinkerforge/BrickletSilentStepperV2.php');

use Tinkerforge\IPConnection;
use Tinkerforge\BrickletSilentStepperV2;

const HOST = 'localhost';
const PORT = 4223;
const UID = 'XYZ'; // Change XYZ to the UID of your Silent Stepper Bricklet 2.0

// Use position reached callback to program random movement
function cb_positionReached($position, $user_data)
{
    $ss = $user_data;

    if (rand(0, 1)) {
        $steps = rand(1000, 5000); // steps (forward)
        echo "Driving forward: $steps steps\n";
    } else {
        $steps = rand(-5000, -1000); // steps (backward)
        echo "Driving backward: $steps steps\n";
    }

    $vel = rand(200, 2000); // steps/s
    $acc = rand(100, 1000); // steps/s^2
    $dec = rand(100, 1000); // steps/s^2
    echo "Configuration (vel, acc, dec): $vel, $acc, $dec\n";

    $ss->setSpeedRamping($acc, $dec);
    $ss->setMaxVelocity($vel);
    $ss->setSteps($steps);
}

$ipcon = new IPConnection(); // Create IP connection
$ss = new BrickletSilentStepperV2(UID, $ipcon); // Create device object

$ipcon->connect(HOST, PORT); // Connect to brickd
// Don't use device before ipcon is connected

// Register position reached callback to function cb_positionReached
$ss->registerCallback(BrickletSilentStepperV2::CALLBACK_POSITION_REACHED,
                      'cb_positionReached', $ss);

$ss->setStepConfiguration(BrickletSilentStepperV2::STEP_RESOLUTION_8,
                          TRUE); // 1/8 steps (interpolated)
$ss->setEnabled(TRUE); // Enable motor power
$ss->setSteps(1); // Drive one step forward to get things going

echo "Press ctrl+c to exit\n";
$ipcon->dispatchCallbacks(-1); // Dispatch callbacks forever

?>
