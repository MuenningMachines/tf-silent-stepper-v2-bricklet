program ExampleConfiguration;

{$ifdef MSWINDOWS}{$apptype CONSOLE}{$endif}
{$ifdef FPC}{$mode OBJFPC}{$H+}{$endif}

uses
  SysUtils, IPConnection, BrickletSilentStepperV2;

type
  TExample = class
  private
    ipcon: TIPConnection;
    ss: TBrickletSilentStepperV2;
  public
    procedure Execute;
  end;

const
  HOST = 'localhost';
  PORT = 4223;
  UID = 'XYZ'; { Change XYZ to the UID of your Silent Stepper Bricklet 2.0 }

var
  e: TExample;

procedure TExample.Execute;
begin
  { Create IP connection }
  ipcon := TIPConnection.Create;

  { Create device object }
  ss := TBrickletSilentStepperV2.Create(UID, ipcon);

  { Connect to brickd }
  ipcon.Connect(HOST, PORT);
  { Don't use device before ipcon is connected }

  ss.SetMotorCurrent(800); { 800 mA }
  ss.SetStepConfiguration(BRICKLET_SILENT_STEPPER_V2_STEP_RESOLUTION_8,
                          true); { 1/8 steps (interpolated) }
  ss.SetMaxVelocity(2000); { Velocity 2000 steps/s }

  { Slow acceleration (500 steps/s^2),
    Fast deacceleration (5000 steps/s^2) }
  ss.SetSpeedRamping(500, 5000);

  ss.SetEnabled(true); { Enable motor power }
  ss.SetSteps(60000); { Drive 60000 steps forward }

  WriteLn('Press key to exit');
  ReadLn;

  { Stop motor before disabling motor power }
  ss.Stop; { Request motor stop }
  ss.SetSpeedRamping(500, 5000); { Fast deacceleration (5000 steps/s^2) for stopping }
  Sleep(400); { Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s }
  ss.SetEnabled(false); { Disable motor power }

  ipcon.Destroy; { Calls ipcon.Disconnect internally }
end;

begin
  e := TExample.Create;
  e.Execute;
  e.Destroy;
end.
