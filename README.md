# ELEGOOSmartCarV4

Elegoo Smart Robot Car V4.0

I got this car for Christmas. It's a fun little kit. Pretty easy to assemble.
It can be controlled with a remote and from a phone/tablet app over Wifi.
The kit I got is running version SmartRobotCarV4.0_20200825.

This is based on the source version SmartRobotCarV4.0_20201218.
From the file ELEGOO Smart Robot Car Kit V4.0 2020.12.18.zip downloaded from the
[ELEGOO website](https://www.elegoo.com/pages/arduino-kits-support-files).

Currently this version does not build properly because it compiles to be too large.
Arduino compile output:
> Sketch uses 32440 bytes (100%) of program storage space. Maximum is 32256 bytes.text section exceeds available space in board
>
> Global variables use 1223 bytes (59%) of dynamic memory, leaving 825 bytes for local variables. Maximum is 2048 bytes.
> Sketch too big; see http://www.arduino.cc/en/Guide/Troubleshooting#size for tips on reducing it.
> Error compiling for board Arduino Uno.

From messing with the source the IR remote is taking most of the space.
Deleting it allows the code to build.
Upgrading FastLED from 3.2.10 to 3.4.0 reduces the size to 32324 bytes.

I was able to get this version to build by deleting the Tracking call.
When uploaded to an Arduino the robot has different behavior.
The car stops responding to the commands from the Android app for several seconds.
The wheels also spin much faster.
