# Smart Car Serial Communications Protocol

The serial communications protocol for the [Elegoo](http://www.elegoo.com)
Smart Robot CAR 4.0. See their
[Download page](https://www.elegoo.com/pages/arduino-kits-support-files)
for files. Taken from the  PDF File
`Communication protocol for Smart Robot Car.pdf`
included in the download file.

Commands and responses are JSON.

**NOTE:** The `"H": "ID"` parameter must always be given as a string.
For example `"H": "123"`.

## Motor

* **Function:** Select the motor to set the rotation direction and speed.
  * **Command:** `{ "H": "ID", "N": 1, "D1": MOTOR, "D2": SPEED, "D3": DIRECTION }`
  * **Python:** `SmartCar.Commands.MotorControl(motor, speed, direction)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **MOTOR:** Select the corresponding motor.
      * **0** - All motors
      * **1** - Left motor
      * **2** - Right motor
    * **SPEED:** The rotation speed value of the selected motor.
      * The range of speed value: 0 ~ 255
    * **DIRECTION:** The rotation direction of the selected motor.
      * **1** - Clockwise
      * **2** - Counterclockwise

* **Function:** Set the direction and speed of the car for a specified amount of time.
  * **Note:** This is not documented in the communications protocol PDF file.
  * **Command:** `{ "H": "ID", "N": 2, "D1": DIRECTION, "D2": SPEED, "T": TIME }`
  * **Python:** `SmartCar.Commands.CarControlTime(direction, speed, time)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **DIRECTION:** The rotation direction of the selected motor.
      * **1** - Turn left
      * **2** - Turn right
      * **3** - Go forward
      * **4** - Back
    * **SPEED:** The rotation speed value of the selected motor.
      * The range of speed value: 0 ~ 255
    * **TIME:** The amount of time to move the car in milliseconds.

* **Function:** Set the direction and speed of the car.
  * **Command:** `{ "H": "ID", "N": 3, "D1": DIRECTION, "D2": SPEED }`
  * **Python:** `SmartCar.Commands.CarControl(direction, speed)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **DIRECTION:** The rotation direction of the selected motor.
      * **1** - Turn left
      * **2** - Turn right
      * **3** - Go forward
      * **4** - Back
    * **SPEED:** The rotation speed value of the selected motor.
      * The range of speed value: 0 ~ 255

* **Function:** Set the speed of the left and right motors separately.
  * **Command:** `{ "H": "ID", "N": 4, "D1": LEFT_SPEED, "D2": RIGHT_SPEED }`
  * **Python:** `SmartCar.Commands.MotorControlSpeed(leftsped, rightsped)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **LEFT_SPEED** - The speed of left wheel.
      * The range of speed value: 0~ 255
    * **RIGHT_SPEED** - The speed of right wheel.
      * The range of speed value: 0~ 255

## Servo motor

* **Function:** Select the rotation angle of the servo motor.
  * **Command:** `{ "H": "ID", "N": 5, "D1": SERVO, "D2": ANGLE }`
  * **Python:** `SmartCar.Commands.ServoControl(servo, angle)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **SERVO** - Select the servo motor.
      * **1** - Servo motor that can turn left and right.
      * **2** - Servo motor that can turn up and down.
    * **ANGLE** - The rotation angle of the servo motor: 0-180.
  * **Examples:**
    * **Send:** `{ "H": "1", "N": 5, "D1": 1, "D2" : 0 }`
      * **Receive:** `{1_ok}`
      * **Action:** Head rotates all the way to the right.
    * **Send:** `{ "H": "1", "N": 5, "D1": 1, "D2" :  180}`
      * **Receive:** `{1_ok}`
      * **Action:** Head rotates all the way to the Left.
    * **Send:** `{ "H": "1", "N": 5, "D1": 1, "D2" : 90 }`
      * **Receive:** `{1_ok}`
      * **Action:** Head rotates to the center.

## Command for remotely switching the car mode

* **Function:** Switch the car mode.
  * **Command:** `{ "N": 101, "D1": MODE }`
  * **Python:** `SmartCar.Commands.SwitchMode(mode)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **MODE:** - The mode to switch the car to.
      * **1** - Tracking mode.
      * **2** - Obstacle-avoidance mode.
      * **3** - Follow mode.

## Joystick clear mode

* **Function:** Clear all functions being executed.
  * **Command:** `{ "N": 100 }`
  * **Python:** `SmartCar.Commands.JoystickClear()`
  * **Return:** `{ID_ok}`

## Joystick movement command

* **Function:** Make the car move in a certain direction at the default maximum speed.
  * **Command:** `{ "N": 102, "D1": DIRECTION, "D2": SPEED }`
  * **Python:** `SmartCar.Commands.JoystickMovement(direction)`
  * **Return:** `{ID_ok}`.
  * **Parameters:**
    * **DIRECTION** - Direction of the car.
      * **1** - Go forward
      * **2** - Back
      * **3** - Turn left
      * **4** - Turn right
      * **5** - Left front
      * **6** - Rear left
      * **7** - Right front
      * **8** - Rear Right
      * **9** - Stop / Standby
      * Any other value is interpreted as stop.
    * **SPEED** - Speed value. (**NOTE:** The code ignores ths parameter.)

## Remote control - Threshold adjustment

* **Function:** Adjust the tracking sensitivity of the car.
  * **Note:** This is not implemented.
  * **Command:** `{ "N": 104, "D1": SENSITIVITY }`
  * **Return:** No return.
  * **Parameters:**
    * **SENSITIVITY** - 50-1000

## FastLED - Brightness adjustment

* **Function:** Adjust the brightness of the FasLED.
  * **Note:** This is not documented in the communications protocol PDF file.
              From testing this command does not work.
  * **Command:** `{ "N": 105, "D1": ADJUSTMENT }`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **ADJUSTMENT** - The direction to adjust the brightness.
      * **1** - Adjust the brightness up by 5, to a maximum of 250.
      * **2** - Adjust the brightness down by 5, to a minimum of 0.

## Camera rotation

* **Function:** Set the rotation direction of the camera.
  * **Command:** `{ "N": 106, "D1": DIRECTION }`
  * **Python:** `SmartCar.Commands.CameraRotation(direction)`
  * **Return:** `{ID_ok}`
  * **Parameters:**
    * **DIRECTION** - The rotation direction of the camera.
      * **1** - Turn up
      * **2** - Turn down
      * **3** - Turn left
      * **4** - Turn right
      * **5** - Turn to the center

## Ultrasonic module

* **Function:** Check whether an obstacle is detected.
  * **Command:** `{ "H": "ID", "N": 21, "D1": MODE }`
  * **Python:** `SmartCar.Commands.UltrasonicStatus(mode)`
  * **Return:**
    * `{ID_false}` - No obstacles detected
    * `{ID_true}` - Obstacles detected
    * `{ID_Ultrasonic value}`
  * **Parameters**
    * **MODE:**
      * **1** - Check whether an obstacle is detected.
      * **2** - Check the value of the ultrasonic sensor.

## Infrared module

* **Function:** Check the value of the infrared sensor.
  * **Command:** `{ "H": "ID", "N": 22, "D1": SENSOR }`
  * **Python:** `SmartCar.Commands.InfraredStatus(sensor)`
  * **Return:** `{ID_Infrared sensor value}`
  * **Parameters:**
    * **SENSOR** - The line following sensor to query.
      * **0** - The value of the L infrared sensor
      * **1** - The value of the M infrared sensor
      * **2** - The value of the R infrared sensor

* **Function:** Check if the car has left the ground.
  * **Note:** This uses the line following sensors.
  * **Command:** `{ "H": "ID", "N": 23 }`
  * **Python:** `SmartCar.Commands.LeftGroud()`
  * **Return:**
    * `{ID_false}` - The car has left the ground.
    * `{ID_true }` - The car is on the ground.

## Programming mode clear all states

* **Function:** Clear all the functions being executed, and do not enter the standby mode.
  * **Command:** `{ "H": "ID", "N": 110 }`
  * **Python:** `SmartCar.Commands.ProgramingClear()`
  * **Return:** `{ID_ok}`
