# Arduino Pinout

Pinout of the Elegoo shield on the Arduino side.

Taken from the  PDF File `SmartRobot-Shield.pdf` included in the download file.
Most of the labels are defined in  the file `DeviceDriverSet_xxx0.h`.

Pin | Description                       | Label
----|-----------------------------------|---------------------------
A0  | Line Tracking Right               | PIN_ITR20001xxxL
A1  | Line Tracking Middle              | PIN_ITR20001xxxM
A2  | Line Tracking Left                | PIN_ITR20001xxxL
A3  | Power Module (Battery level)      | PIN_Voltage
A4  | MPU6050 SCL                       | N/A
A5  | MPU6050 SDA                       | N/A
0   | UART TX to Wifi module            | N/A
1   | UART RX from Wifi module          | N/A
2   | Mode Switch                       | PIN_Key
3   | UNUSED
4   | RGB LED DIN                       | PIN_RBGLED
5   | DRV8835 Motor Driver AIN2/EN      | PIN_Motor_PWMA
6   | DRV8835 Motor Driver BIN2/EN      | PIN_Motor_PWMB
7   | DRV8835 Motor Driver BIN1/PH      | PIN_Motor_BIN_1
8   | DRV8835 Motor Driver AIN1/PH      | PIN_Motor_AIN_1
9   | Infrared Receive pin              | RECV_PIN
10  | Servo #1 Z axis                   | PIN_Servo_z
11  | Servo #2 y axis                   | PIN_Servo_y
12  | Ultrasonic echo pin               | ECHO_PIN
13  | Ultrasonic trigger pin            | TRIG_PIN
