"""Commands for the Smart Car. JSON is returned ready to be sent to the robot."""

import json

# This is the sequence number for commands that take an ID.
SequenceNumber = 0

def CameraRotation(direction):
    """
    Returns the JSON to set the rotation direction of the camera.

    Args:
        direction (int): The mode to set the car to.
            1 - Turn up
            2 - Turn down
            3 - Turn left
            4 - Turn right
    """

    # Verify the parameters

    if direction < 1 or direction > 4:
        raise ValueError("The direction must be between 1 and 4, %d is out of range." % direction)

    # Create and return the command.
    command = {
        "N": 106,
        "D1": direction
    }
    return json.dumps(command)

def CarControl(direction, speed):
    """
    Returns the JSON to set the direction and speed of the car.


    Args:
        direction (int): The direction to rotate the motor.
            1 - Turn left
            2 - Turn right
            3 - Go forward
            4 - Back
        speed (int): The speed to set the motor to. 0 - 255
    """

    # Verify the parameters
    if direction < 0 or direction > 4:
        raise ValueError("The direction can only be between 1 and 4, %d is out of range." % direction)
    if speed < 0 or speed > 255:
        raise ValueError("The speed must be between 0 and 255, %d is out of range." % speed)

    # Create and return the command.
    global SequenceNumber
    SequenceNumber += 1
    command = {
        "H": str(SequenceNumber),
        "N": 3,
        "D1": direction,
        "D2": speed
    }
    return json.dumps(command)

def CarControlTime(direction, speed, time):
    """
    Returns the JSON to set the direction and speed of the car for a specified amount of time.


    Args:
        direction (int): The direction to rotate the motor.
            1 - Turn left
            2 - Turn right
            3 - Go forward
            4 - Back
        speed (int): The speed to set the motor to. 0 - 255
        time (int): The amount of time to move the car in milliseconds.
    """

    # Verify the parameters
    if direction < 0 or direction > 4:
        raise ValueError("The direction can only be between 1 and 4, %d is out of range." % direction)
    if speed < 0 or speed > 255:
        raise ValueError("The speed must be between 0 and 255, %d is out of range." % speed)

    # Create and return the command.
    global SequenceNumber
    SequenceNumber += 1
    command = {
        "H": str(SequenceNumber),
        "N": 2,
        "D1": direction,
        "D2": speed,
        "D3": time
    }
    return json.dumps(command)

def InfraredStatus(sensor):
    """
    Returns the JSON to check  the value of the infrared sensor.

    Args:
        sensor (int): The mode check.
            0 - The value of the L infrared sensor
            1 - The value of the M infrared sensor
            2 - The value of the R infrared sensor
    """

    # Verify the parameters

    if sensor < 0 or sensor > 2:
        raise ValueError("The sensor must be between 0 and 2, %d is out of range." % sensor)

    # Create and return the command.
    command = {
        "N": 22,
        "D1": sensor
    }
    return json.dumps(command)

def JoystickClear():
    """Return the JSON to clear the joystick mode."""
    # Create and return the command.
    command = {
        "N": 100
    }
    return json.dumps(command)

def JoystickMovement(direction, speed=None):
    """
    Returns the JSON to command the Smart car to move in a direction.

    Args:
        direction (int): The direction to move the car in.
            0 - Stop
            1 - Go forward
            2 - Back
            3 - Turn left
            4 - Turn right
            5 - Left front
            6 - Rear left
            7 - Right front
            8 - Rear Right
            9 - Stop / Standby
        speed (int): Speed is defined in the documentation but currently ignored in the code.
    """

    # Verify the parameters
    if direction < 0 or direction > 9:
        raise ValueError("The direction must be between 0 and 9, %d is out of range." % angle)

    # Create and return the command.
    command = {
        "N": 102,
        "D1": direction
    }
    return json.dumps(command)

def LeftGroud():
    """Returns the JSON to check if the car has left the ground."""
    # Create and return the command.
    command = {
        "N": 23
    }
    return json.dumps(command)

def MotorControl(motor, speed, direction):
    """
    Returns the JSON to set the speed and direction of a motor.

    Args:
        motor (int): The motor to control.
            0 - All motors
            1 - Left motor
            2 - Right motor
        speed (int): The speed to set the motor to. 0 - 255
        direction (int): The direction to rotate the motor.
            1 - Clockwise
            2 - Counter clockwise
    """

    # Verify the parameters
    if motor < 0 or motor > 2:
        raise ValueError("The motor must be between 0 and 2, %d is out of range." % motor)
    if speed < 0 or speed > 255:
        raise ValueError("The speed must be between 0 and 255, %d is out of range." % speed)
    if direction != 1 and direction != 2:
        raise ValueError("The direction can only be 1 or 2, %d is out of range." % direction)

    # Create and return the command.
    global SequenceNumber
    SequenceNumber += 1
    command = {
        "H": str(SequenceNumber),
        "N": 1,
        "D1": motor,
        "D2": speed,
        "D3": direction
    }
    return json.dumps(command)

def MotorControlSpeed(leftsped, rightsped):
    """
    Returns the JSON to set the speed of the left and right motors separately.

    Args:
        leftsped (int): The speed to set the motor to. 0 - 255
        rightsped (int): The speed to set the motor to. 0 - 255
    """

    # Verify the parameters
    if leftsped < 0 or leftsped > 255:
        raise ValueError("The leftsped must be between 0 and 255, %d is out of range." % leftsped)
    if rightsped < 0 or rightsped > 255:
        raise ValueError("The rightsped must be between 0 and 255, %d is out of range." % rightsped)

    # Create and return the command.
    global SequenceNumber
    SequenceNumber += 1
    command = {
        "H": str(SequenceNumber),
        "N": 4,
        "D1": leftsped,
        "D2": rightsped
    }
    return json.dumps(command)

def ProgramingClear():
    """Returns the JSON to clear programming mode states."""
    # Create and return the command.
    global SequenceNumber
    SequenceNumber += 1
    command = {
        "H": str(SequenceNumber),
        "N": 110
    }
    return json.dumps(command)

def ServoControl(servo, angle):
    """
    Returns the JSON to command the Smart car to move one of the servos.

    Args:
        servo (int): The servo to select. Either 1 or 2.
        angle (int): The angle to move the servo to. 0 - 180
    """

    # Verify the parameters
    if servo != 1 and servo != 2:
        raise ValueError("The servo can only be 1 or 2, %d is out of range." % servo)
    if angle < 0 or angle > 180:
        raise ValueError("The angle must be between 0 and 180, %d is out of range." % angle)

    # Create and return the command.
    global SequenceNumber
    SequenceNumber += 1
    command = {
        "H": str(SequenceNumber),
        "N": 5,
        "D1": servo,
        "D2": angle
    }
    return json.dumps(command)

def SwitchMode(mode):
    """
    Returns the JSON to switch the car mode.

    Args:
        mode (int): The mode to set the car to.
            1 - Tracking mode.
            2 - Obstacle-avoidance mode.
            3 - Follow mode.
    """

    # Verify the parameters

    if mode < 1 or mode > 3:
        raise ValueError("The mode must be between 1 and 3, %d is out of range." % mode)

    # Create and return the command.
    command = {
        "N": 101,
        "D1": mode
    }
    return json.dumps(command)

def UltrasonicStatus(mode):
    """
    Returns the JSON to check whether an obstacle is detected.

    Args:
        mode (int): The mode check.
            1 - Check whether an obstacle is detected.
            2 - Check the value of the ultrasonic sensor.
    """

    # Verify the parameters

    if mode < 1 or mode > 2:
        raise ValueError("The mode must be between 1 and 2, %d is out of range." % mode)

    # Create and return the command.
    command = {
        "N": 21,
        "D1": mode
    }
    return json.dumps(command)
