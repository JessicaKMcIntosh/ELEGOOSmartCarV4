#!/usr/bin/env python

# This is an example of using the Smart Car Python Library.
# I'm using this to learn Python so please pardon the mess.
# NOTE This currently only works on Windows.

# You can interact with the Robot with the following keys:
# w = Move forward with speed 100 for 1000 milliseconds.
# s = Move backward with speed 100 for 1000 milliseconds.
# a = Move to the left with speed 100 for 200 milliseconds.
# d = Move to the right with speed 100 for 200 milliseconds.
# W = Go forward with speed of 100. No time limit.
# S = Go backward with speed of 100. No time limit.
# A = Turn right with speed of 100. No time limit.
# D = Turn left with speed of 100. No time limit.
# SPACE = Clear all movement and commands. Stop the car.
# Q = Quit this program.
# [ = Rotate the camera left.
# ] = Rotate the camera right.

import json
import msvcrt
import queue
import signal
import socket
import sys
import SmartCar.TCP
import SmartCar.Commands

# Main program.
def main():
    # Configuration:
    # This is the default for the Smart Car ESP.
    Robot_IP = "192.168.4.1"
    Robot_Port = 100
    # Set a short timeout. No sense waiting forever and the ESP sends heartbeats.
    Socket_Timeout = 5
    # A queue of commands for the Queue callback.
    QueuePrepare()

    # Connect to the Smart Car ESP.
    Robot_Socket = SmartCar.TCP.OpenSocket(Robot_IP, Robot_Port, Socket_Timeout)

    # Setup a signal handler for Control-C.
    setup_signal(Robot_Socket)

    # Talk to the Smart Car ESP.
    SmartCar.TCP.Interact(Robot_Socket, Callback)

    # Demonstration using a Queue.
    # To use comment out the previous statement and uncomment this one.
    #SmartCar.TCP.Interact(Robot_Socket, QueueCallback)

# Handle Control-C in a more graceful manner.
def setup_signal(Robot_Socket):
    # Create the signal handler to close the socket before quitting.
    def signal_handler(sig, frame):
        print('')
        print('Exiting Gracefully...')
        SmartCar.TCP.CloseSocket(Robot_Socket)
        quit()

    # Attach the signal handler.
    signal.signal(signal.SIGINT, signal_handler)

# Callback for SmartCar.TCP.Interact to allow the user to control the Smart Car.
def Callback(client_socket):
    # Check for a key press.
    if msvcrt.kbhit():
        command = None
        key = str(msvcrt.getch())
        (description, command) = KeyPress(key, client_socket)

        # If that was a valid key press send the command to the Smart Car.
        if command:
            print(description, command)
            client_socket.send(command.encode())

# Process a keypress.
# Return the JSON to send to the robot and a description for the user.
# Invalid keys are simply ignored.
def KeyPress(key, client_socket):
    description = ''
    command = None

    if key == "b'w'":
        description = 'Move forward.'
        command = SmartCar.Commands.CarControlTime(3, 100, 1000)
    if key == "b's'":
        description = 'Move backward.'
        command = SmartCar.Commands.CarControlTime(4, 100, 1000)
    if key == "b'a'":
        description = 'Move left.'
        command = SmartCar.Commands.CarControlTime(1, 100, 200)
    if key == "b'd'":
        description = 'Move right.'
        command = SmartCar.Commands.CarControlTime(2, 100, 200)
    if key == "b'W'":
        description = 'Do forward...'
        command = SmartCar.Commands.CarControl(3, 100)
    if key == "b'S'":
        description = 'Go backward...'
        command = SmartCar.Commands.CarControl(4, 100)
    if key == "b'A'":
        description = 'Go left...'
        command = SmartCar.Commands.CarControl(1, 100)
    if key == "b'D'":
        description = 'Go right...'
        command = SmartCar.Commands.CarControl(2, 100)
    if key == "b' '":
        description = 'STOP!'
        command = SmartCar.Commands.JoystickClear()
    if key == "b'['":
        description = 'Camera left.'
        command = SmartCar.Commands.CameraRotation(3)
    if key == "b']'":
        description = 'Camera left.'
        command = SmartCar.Commands.CameraRotation(4)
    if key == "b'Q'":
        print('Bye.')
        SmartCar.TCP.CloseSocket(client_socket)
        quit()

    return (description, command)

# XXX An example of using a queue.

# This sends the commands in the queue to the car.
# The queue is created with QueuePrepare().
def QueueCallback(client_socket):
    if not Command_Queue.empty():
        message = Command_Queue.get()
        print("Sending command:", message)
        client_socket.send(message.encode())
    else:
        SmartCar.TCP.CloseSocket(client_socket)
        quit()

# Creates a queue of commands for the Queue Callback.
def QueuePrepare():
    global Command_Queue
    Command_Queue = queue.Queue()
    Command_Queue.put(SmartCar.Commands.ServoControl(1, 60))  # Rotate the head right.
    Command_Queue.put(SmartCar.Commands.ServoControl(1, 90))  # Center the head.
    Command_Queue.put(SmartCar.Commands.ServoControl(1, 120)) # Rotate the head left.
    Command_Queue.put(SmartCar.Commands.ServoControl(1, 90))  # Center the head.
    Command_Queue.put(SmartCar.Commands.JoystickMovement(1))  # Move forward at full speed.
    Command_Queue.put(SmartCar.Commands.JoystickMovement(9))  # Stop.
    Command_Queue.put(SmartCar.Commands.JoystickMovement(2))  # Move backward at full speed.
    Command_Queue.put(SmartCar.Commands.JoystickMovement(9))  # Stop.
    Command_Queue.put(SmartCar.Commands.JoystickClear())      # Clear all movement commands.

# Run the main program.
if __name__ == '__main__':
    main()
