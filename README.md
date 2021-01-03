# ELEGOOSmartCarV4

Elegoo Smart Robot Car V4.0

I got this car for Christmas. It's a fun little kit. Pretty easy to assemble.
It can be controlled with a remote and from a phone/tablet app over Wifi. The
kit I got is running version SmartRobotCarV4.0_20200825.

This is based on the source version SmartRobotCarV4.0_20201218.
From the file `ELEGOO Smart Robot Car Kit V4.0 2020.12.18.zip` downloaded from the
[ELEGOO website](https://www.elegoo.com/pages/arduino-kits-support-files).

The Python library `SmartCar` is my own creation for controlling the robot.
Currently it only supports TCP/IP commucations with the Smart Car.
There is an example program `SmartCar/example.py` for using the library.
Right now the example program only supports Windows.
See the file `SmartCar/README.md` for more details.

**UPDATE 2020-12-30:**
I had emailed support about the downloaded source not building. They sent me a
new file that has custom IR Remote files. With these new files the code builds
just fine. And seems to work just the same as what the robot came with. This
has been uploaded to the support site with the file name
`ELEGOO Smart Robot Car Kit V4.0 2020.12.30.zip`.
