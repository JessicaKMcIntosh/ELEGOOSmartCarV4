# Elegoo Smart Robot Car V4.0 Python Library

This is a Python library for interfacing with the Elegoo Smart Robot Car V4.0.

This is very much a work in progress. I'm also using this to learn Python,
so it will probably not be the best Python out there.

## Modules

### SmartCar.Commands

This module is used to generate JSON to send commands to the car. This JSON
can then be sent to the car over Serial or TCP to the ESP module to control
the car. See the documentation in the file `Protocol.md` for details on each
command.

**Example:**

```python
import SmartCar.Commands

# Center the first servo.
# #This moves the "head" to the center.
json = SmartCar.Commands.ServoControl(1, 90)
print('Center the head servo: ', json)
```

**Commands:**

Number  | Command                                   | Description
--------|-------------------------------------------|--------------------------------------
106     | CameraRotation(direction)                 | Set the rotation direction of the camera.
3       | CarControl(direction, speed)              | Set the direction and speed of the car.
2       | CarControlTime(direction, speed, time)    | Set the direction and speed of the car for a specified amount of time.
22      | InfraredStatus(sensor)                    | Check the value of the infrared sensor.
100     | JoystickClear()                           | Clear all functions being executed.
102     | JoystickMovement(direction)               | Make the car move in a certain direction at the default maximum speed.
23      | LeftGroud()                               | Check if the car has left the ground.
1       | MotorControl(motor, speed, direction)     | Select the motor to set the rotation direction and speed.
4       | MotorControlSpeed(leftsped, rightsped)    | Set the speed of the left and right motors separately.
110     | ProgramingClear()                         | Clear all the functions being executed, and do not enter the standby mode.
5       | ServoControl(servo, angle)                | Select the rotation angle of the servo motor.
101     | SwitchMode(mode)                          | Switch the car mode.
21      | UltrasonicStatus(mode)                    | Check whether an obstacle is detected by the Ultrasonic module.
