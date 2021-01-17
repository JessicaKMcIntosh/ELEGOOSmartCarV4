/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-14 15:38:01
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include <avr/wdt.h>
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

// 硬件设备成员对象序列 - Hardware device member object sequence
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;

// Return true if test is between low and high
static boolean
is_between(long test, long low, long high)
{
  if (low <= test && test <= high)
    return true;
  else
    return false;
}

static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

/*运动方向控制序列 - Motion direction control sequence*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it        //(9)
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列 - Mode control sequence*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,                           // 空闲模式 - Idle mode
  TraceBased_mode,                        // 循迹模式 - Tracking mode
  ObstacleAvoidance_mode,                 // 避障模式 - Obstacle Avoidance Mode
  Follow_mode,                            // 跟随模式 - Follow mode
  Rocker_mode,                            // 摇杆模式 - Joystick mode
  CMD_inspect,
  CMD_Programming_mode,                   // 编程模式 - Programming mode
  CMD_ClearAllFunctions_Standby_mode,     // 清除所有功能：进入空闲模式 - Clear all functions: enter idle mode
  CMD_ClearAllFunctions_Programming_mode, // 清除所有功能：进入编程模式 - Clear all functions: enter programming mode
  CMD_MotorControl,                       // 电机控制模式 - Motor control mode
  CMD_CarControl_TimeLimit,               // 小车方向控制：有时间限定模式 - Car direction control: time limited mode
  CMD_CarControl_NoTimeLimit,             // 小车方向控制：无时间限定模式 - Car direction control: no time limit mode
  CMD_MotorControl_Speed,                 // 电机控制:控制转速模式 - Motor control: control speed mode
  CMD_ServoControl,                       // 舵机控制:模式  - Servo control: mode
  CMD_LightingControl_TimeLimit,          // 灯光控制:模式 - Lighting control: mode
  CMD_LightingControl_NoTimeLimit,        // 灯光控制:模式 - Lighting control: mode

};

/*控制管理成员 - Mode control sequence*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  AppIRrecv.DeviceDriverSet_IRrecv_Init();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

/*ITR20001 检测小车是否离开地面 - Check whether the car leaves the ground*/
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}
/*
  直线运动控制：
  direction：方向选择 前/后
  directionRecord：方向记录（作用于首次进入该函数时更新方向位置数据，即:yaw偏航）
  speed：输入速度 （0--255）
  Kp：位置误差放大比例常数项（提高位置回复状态的反映，输入时根据不同的运动工作模式进行修改）
  UpperLimit：最大输出控制量上限

  Google Translate:
  Linear motion control:
  direction: direction selection front/back
  directionRecord: direction record (acting to update the direction and position data when entering the function for the first time, namely: yaw yaw)
  speed: input speed (0--255)
  Kp: Position error amplification proportional constant item (improve the reflection of the position return state, and modify it according to different motion work modes when input)
  UpperLimit: the upper limit of the maximum output control amount
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航 - Yaw
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp - Add proportional constant Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //前进 - Go forward.
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退 - Go Backward.
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)

  Google Translate:
  sport control:
  1# direction: forward (1), backward (2), left front (3), right front (4), back left (5), back right (6)
  2# speed (0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;
  //需要进行直线运动调整的控制模式（在以下工作运动模式小车前后方向运动时容易产生位置偏移，运动达不到相对直线方向的效果，因此需要加入控制调节）
/*
    Google Translate:
    The control mode that needs linear motion adjustment
    (in the following work motion modes, the car is prone to position
    deviation when moving forward and backward, and the motion cannot
    achieve the effect of the relative linear direction, so control
    adjustment is required)
*/
  switch (Application_SmartRobotCarxxx0.Functional_Mode)
  {
  case Rocker_mode:
    Kp = 10;
    UpperLimit = 255;
    break;
  case ObstacleAvoidance_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case Follow_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_TimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_NoTimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  default:
    Kp = 10;
    UpperLimit = 255;
    break;
  }
  switch (direction)
  {
  case Forward:
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //前进时进入方向位置逼近控制环处理
      //Enter the direction and position approach control loop processing when advancing
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case Backward:
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //后退时进入方向位置逼近控制环处理
      //Enter the direction position approach control loop processing when retreating
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case Left:
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case Right:
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case LeftForward:
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case LeftBackward:
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case RightForward:
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case RightBackward:
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case stop_it:
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control

    break;
  default:
    directionRecord = 10;
    break;
  }
}
/*
  传感器数据更新:局部更新(选择性更新)
  Sensor data update: partial update (selective update)
*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{
  { // 电压状态更新 - Voltage status update
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    //10ms 采集并更新一次 - 10ms acquisition and update once
    if (millis() - VoltageData_time > 10)
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection)
      {
        VoltageData_number++;
        //连续性多次判断最新的电压值...
        // Continuity to determine the latest voltage value multiple times...
        if (VoltageData_number == 500)
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      }
      else
      {
        VoltageDetectionStatus = false;
      }
    }
  }

  // { /*避障状态更新*/
  //   Obstacle avoidance status update
  //   AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/);
  //   UltrasoundDetectionStatus = is_between(UltrasoundData_cm, 0, ObstacleDetection);
  // }

  { /*R循迹状态更新 - R tracking status update*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = is_between(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = is_between(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = is_between(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    //ITR20001 检测小车是否离开地面 - Detects whether the car is off the ground
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }

  // //获取时间戳 timestamp
  // static unsigned long Test_time;
  // if (millis() - Test_time > 200)
  // {
  //   Test_time = millis();
  //   //AppITR20001.DeviceDriverSet_ITR20001_Test();
  // }
}
/*
  开机动作需求：
  Boot action requirements:
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void)
{
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

static void CMD_Lighting(uint8_t is_LightingSequence, int8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  switch (is_LightingSequence)
  {
  case 0:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(NUM_LEDS, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 1: /* 左 - Left */
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(3, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 2: /* 前 - Front */
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(2, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 3: /* 右 - Right */
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(1, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 4: /* 后 - Rear */
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 5: /* 中 - Center?? */
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(4, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  default:
    break;
  }
}

/*RBG_LED 集合*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus) //低电压？
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (is_between((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    switch (temp)
    {
    case 0 ... 49:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case 50 ... 99:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case 100 ... 149:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case 150 ... 199:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case 200 ... 249:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case 250 ... 299:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case 300 ... 349:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case 350 ... 399:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case 400 ... 449:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case 450 ... 499:
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    default:
      break;
    }
  }
  else if (((is_between((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (Application_SmartRobotCarxxx0.Functional_Mode) //Act on 模式控制序列 - Mode control sequence
    {
    case Standby_mode:
      {
        if (VoltageDetectionStatus == true)
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay(30);
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay(30);
        }
        else
        {
          static uint8_t setBrightness = 0;
          static boolean et = false;
          static unsigned long time = 0;

          if ((millis() - time) > 30)
          {
            time = millis();
            if (et == false)
            {
              setBrightness += 1;
              if (setBrightness == 80)
                et = true;
            }
            else if (et == true)
            {
              setBrightness -= 1;
              if (setBrightness == 1)
                et = false;
            }
          }

          AppRBG_LED.leds[1] = CRGB::Blue;
          AppRBG_LED.leds[0] = CRGB::Violet;
          FastLED.setBrightness(setBrightness);
          FastLED.show();
        }
      }
      break;
    case CMD_Programming_mode:
      {
      }
      break;
    case TraceBased_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Green);
      }
      break;
    case ObstacleAvoidance_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      }
      break;
    case Follow_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
      }
      break;
    case Rocker_mode:
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Violet);
      }
      break;
    default:
      break;
    }
  }
}

/*摇杆*/
void ApplicationFunctionSet::ApplicationFunctionSet_Rocker(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == Rocker_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, 250 /*speed*/);
  }
}

/*循迹*/
void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void)
{
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) //车子离开地面了？ - The car left the ground?
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    // int getAnaloguexxx_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    // int getAnaloguexxx_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    // int getAnaloguexxx_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    // #if _Test_print
    //     static unsigned long print_time = 0;
    //     if (millis() - print_time > 500)
    //     {
    //       print_time = millis();
    //       Serial.print("ITR20001_getAnaloguexxx_L=");
    //       Serial.println(getAnaloguexxx_L);
    //       Serial.print("ITR20001_getAnaloguexxx_M=");
    //       Serial.println(getAnaloguexxx_M);
    //       Serial.print("ITR20001_getAnaloguexxx_R=");
    //       Serial.println(getAnaloguexxx_R);
    //     }
    // #endif
    if (is_between(TrackingData_M, TrackingDetection_S, TrackingDetection_E))
    {
      // 控制左右电机转动：实现匀速直行
      // Control the rotation of the left and right motors
      // to achieve a uniform speed straight
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (is_between(TrackingData_R, TrackingDetection_S, TrackingDetection_E))
    {
      // 控制左右电机转动：前右
      // Control left and right motor rotation: front right
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (is_between(TrackingData_L, TrackingDetection_S, TrackingDetection_E))
    {
      // 控制左右电机转动：前左
      // Control left and right motor rotation: front left
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else //不在黑线上的时候。。。 - When not on the black line. . .
    {
      if (timestamp == true) //获取时间戳 - Get timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      /*Blind Detection*/
      if ((is_between((millis() - MotorRL_time), 0, 200) || is_between((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      }
      else if (((is_between((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      }
      else if ((is_between((millis() - MotorRL_time), 3000, 3500))) // Blind Detection ...s ?
      {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  }
  else if (false == timestamp)
  {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
}

/*
  避障功能 - Obstacle avoidance function
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  static boolean first_is = true;
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    uint8_t switc_ctrl = 0;
    uint16_t get_Distance;
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      first_is = false;
    }

    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
    if (is_between(get_Distance, 0, 20))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

      for (int i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
      {
        AppServo.DeviceDriverSet_Servo_control(30 * i /*Position_angle*/);
        delay_xxx(1);
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);

        if (is_between(get_Distance, 0, 20))
        {
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          if (5 == i)
          {
            ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
            delay_xxx(500);
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            delay_xxx(100);
            first_is = true;
            break;
          }
        }
        else
        {
          switc_ctrl = 0;
          switch (i)
          {
          case 1:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            break;
          case 3:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
            break;
          case 5:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
            break;
          }
          delay_xxx(100);
          first_is = true;
          break;
        }
      }
    }
    else //if (is_between(get_Distance, 20, 50))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
    }
  }
  else
  {
    first_is = true;
  }
}

/*
  跟随模式： - Follow mode:
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void)
{
  static uint16_t ULTRASONIC_Get = 0;
  static unsigned long ULTRASONIC_time = 0;
  static uint8_t Position_Servo = 1;
  static uint8_t timestamp = 3;
  static uint8_t OneCycle = 1;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Follow_mode)
  {

    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ULTRASONIC_Get /*out*/);
    // 前方 20 cm内无障碍物？ - There are no obstacles within 20 cm ahead?
    if (false == is_between(ULTRASONIC_Get, 0, 20))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      static unsigned long time_Servo = 0;
      static uint8_t Position_Servo_xx = 0;

      if (timestamp == 3)
      {
        // 作用于舵机：避免循环执行 - Acting on the steering gear: avoid loop execution
        if (Position_Servo_xx != Position_Servo)
        {
          // 作用于舵机：转向角记录 - Acting on the steering gear: steering angle record
          Position_Servo_xx = Position_Servo;

          if (Position_Servo == 1)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(80 /*Position_angle*/);
          }
          else if (Position_Servo == 2)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(20 /*Position_angle*/);
          }
          else if (Position_Servo == 3)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(80 /*Position_angle*/);
          }
          else if (Position_Servo == 4)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(150 /*Position_angle*/);
          }
        }
      }
      else
      {
        if (timestamp == 1)
        {
          timestamp = 2;
          time_Servo = millis();
        }
      }
      // 作用于舵机停留位置时长_2s - Acting on the stay position of the steering gear_2s
      if (millis() - time_Servo > 1000)
      {
        timestamp = 3;
        Position_Servo += 1;
        OneCycle += 1;
        if (OneCycle > 4)
        {
          Position_Servo = 1;
          OneCycle = 5;
        }
      }
    }
    else
    {
      OneCycle = 1;
      timestamp = 1;
      if ((Position_Servo == 1))
      { // 控制左右电机转动：前进 - Control the rotation of the left and right motors: forward
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      }
      else if ((Position_Servo == 2))
      { // 控制左右电机转动：前右 - Control left and right motor rotation: front right
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
      }
      else if ((Position_Servo == 3))
      {
        // 控制左右电机转动：前进 - Control the rotation of the left and right motors: forward
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      }
      else if ((Position_Servo == 4))
      { // 控制左右电机转动：前左 - Control left and right motor rotation: front left
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
      }
    }
  }
  else
  {
    ULTRASONIC_Get = 0;
    ULTRASONIC_time = 0;
  }
}

// 舵机控制 - Steering gear control
void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo)
{
  static int z_angle = 9;
  static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo; //防止被优化 - Prevent being optimized

  switch (temp_Set_Servo)
  {
  case 1 ... 2:
  {
    if (1 == temp_Set_Servo)
    {
      y_angle -= 1;
    }
    else if (2 == temp_Set_Servo)
    {
      y_angle += 1;
    }
    if (y_angle <= 3) //下限控制 - Lower limit control
    {
      y_angle = 3;
    }
    if (y_angle >= 11) //上下限控制 - Upper and lower limit control
    {
      y_angle = 11;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ y_angle);
  }
  break;

  case 3 ... 4:
  {
    if (3 == temp_Set_Servo)
    {
      z_angle += 1;
    }
    else if (4 == temp_Set_Servo)
    {
      z_angle -= 1;
    }

    if (z_angle <= 1) //下限控制 - Lower limit control
    {
      z_angle = 1;
    }
    if (z_angle >= 17) //上下限控制 - Upper and lower limit control
    {
      z_angle = 17;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ z_angle);
  }
  break;
  case 5:
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ 9);
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 9);
    break;
  default:
    break;
  }
}
/*待机*/
void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void)
{
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    // 作用于偏航原始数据调零(确认小车放置在静止平面！)
    // Act on the yaw raw data to zero (make sure the car is placed on a stationary plane!)
    if (true == is_ED)
    {
      static unsigned long timestamp; // 获取时间戳 Get timestamp.
      if (millis() - timestamp > 20)
      {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround() /* condition */)
        {
          cout += 1;
        }
        else
        {
          cout = 0;
        }
        if (cout > 10)
        {
          is_ED = false;
          AppMPU6050getdata.MPU6050_calibration();
        }
      }
    }
  }
}

/*
 * Begin:CMD
 * Graphical programming and command control module
 * $ Elegoo & SmartRobot & 2020-06
*/

void ApplicationFunctionSet::CMD_inspect_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_inspect)
  {
    Serial.println("CMD_inspect");
    delay(100);
  }
}
/*
  N1:指令
  CMD模式：运动模式 <电机控制> 接收并根据 APP端控制命令   执行对电机的单方向驱动
  输入：uint8_t is_MotorSelection,  电机选择   1左  2右  0全部
        uint8_t is_MotorDirection, 电机转向  1正  2反  0停止
        uint8_t is_MotorSpeed,     电机速度   0-250
        无时间限定

  Google Translate:
  N1: instruction
  CMD mode: motion mode <motor control> receives and executes unidirectional drive of the motor according to the APP control command
  Input: uint8_t is_MotorSelection, motor selection 1 left 2 right 0 all
    uint8_t is_MotorDirection, motor direction 1 forward 2 reverse 0 stop
    uint8_t is_MotorSpeed, motor speed 0-250
    No time limit
*/
// void ApplicationFunctionSet::CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
// {
//   static boolean MotorControl = false;
//   static uint8_t is_MotorSpeed_A = 0;
//   static uint8_t is_MotorSpeed_B = 0;
//   if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl)
//   {
//     MotorControl = true;
//     if (0 == is_MotorDirection)
//     {
//       ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
//     }
//     else
//     {
//       switch (is_MotorSelection) //电机选择
//       {
//       case 0:
//       {
//         is_MotorSpeed_A = is_MotorSpeed;
//         is_MotorSpeed_B = is_MotorSpeed;
//         if (1 == is_MotorDirection)
//         { //正转
//           AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
//                                                  /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
//                                                  /*controlED*/ control_enable); //Motor control
//         }
//         else if (2 == is_MotorDirection)
//         { //反转
//           AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
//                                                  /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
//                                                  /*controlED*/ control_enable); //Motor control
//         }
//         else
//         {
//           return;
//         }
//       }
//       break;
//       case 1:
//       {
//         is_MotorSpeed_A = is_MotorSpeed;
//         if (1 == is_MotorDirection)
//         { //正转
//           AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
//                                                  /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
//                                                  /*controlED*/ control_enable); //Motor control
//         }
//         else if (2 == is_MotorDirection)
//         { //反转
//           AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
//                                                  /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
//                                                  /*controlED*/ control_enable); //Motor control
//         }
//         else
//         {
//           return;
//         }
//       }
//       break;
//       case 2:
//       {
//         is_MotorSpeed_B = is_MotorSpeed;
//         if (1 == is_MotorDirection)
//         { //正转
//           AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
//                                                  /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
//                                                  /*controlED*/ control_enable); //Motor control
//         }
//         else if (2 == is_MotorDirection)
//         { //反转
//           AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
//                                                  /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
//                                                  /*controlED*/ control_enable); //Motor control
//         }
//         else
//         {
//           return;
//         }
//       }
//       break;
//       default:
//         break;
//       }
//     }
//   }
//   else
//   {
//     if (MotorControl == true)
//     {
//       MotorControl = false;
//       is_MotorSpeed_A = 0;
//       is_MotorSpeed_B = 0;
//     }
//   }
// }
void ApplicationFunctionSet::CMD_MotorControl_xxx0(void)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == CMD_is_MotorDirection)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (CMD_is_MotorSelection) //电机选择
      {
      case 0:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //正转
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //反转
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //正转
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //反转
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //正转
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //反转
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}

static void CMD_CarControl(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  switch (is_CarDirection)
  {
  case 1: // 运动模式 左前 - Sports mode front left
    ApplicationFunctionSet_SmartRobotCarMotionControl(Left, is_CarSpeed);
    break;
  case 2: // 运动模式 右前 - Sports mode front right
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, is_CarSpeed);
    break;
  case 3: // 运动模式 前进 - Sport mode forward
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, is_CarSpeed);
    break;
  case 4: // 运动模式 后退 - Sport mode back
    ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, is_CarSpeed);
    break;
  default:
    break;
  }
}
/*
  N2：指令
  CMD模式：<车子控制> 接收并根据 APP端控制命令   执行对车的单方向驱动
  有时间限定
*/
// void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer)
// {
//   static boolean CarControl = false;
//   static boolean CarControl_TE = false; //还有时间标志
//   static boolean CarControl_return = false;
//   if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit) //进入车子有时间限定控制模式
//   {
//     CarControl = true;
//     if (is_Timer != 0) //#1设定时间不为..时 (空)
//     {
//       if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (is_Timer)) //判断时间戳
//       {
//         CarControl_TE = true;
//         ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

//         Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*进入编程模式提示符<等待下一组控制命令的到来>*/
//         if (CarControl_return == false)
//         {

// #if _is_print
//           Serial.print('{' + CommandSerialNumber + "_ok}");
// #endif
//           CarControl_return = true;
//         }
//       }
//       else
//       {
//         CarControl_TE = false; //还有时间
//         CarControl_return = false;
//       }
//     }
//     if (CarControl_TE == false)
//     {
//       CMD_CarControl(is_CarDirection, is_CarSpeed);
//     }
//   }
//   else
//   {
//     if (CarControl == true)
//     {
//       CarControl_return = false;
//       CarControl = false;
//       Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
//     }
//   }
// }

/*
  N2：指令
  CMD模式：<车子控制> 接收并根据 APP端控制命令   执行对车的单方向驱动
  有时间限定

  Google Translate:
  N2: instruction
  CMD mode: <car control> receives and executes unidirectional driving of the car according to the APP control command
  Time limit
*/
void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false; // 还有时间标志 - There is a time stamp
  static boolean CarControl_return = false;
  // 进入车子有时间限定控制模式 - Time-limited control mode when entering the car
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit)
  {
    CarControl = true;
    // #1设定时间不为..时 (空) - #1 Set time is not: time (empty)
    if (CMD_is_CarTimer != 0)
    {
      // 判断时间戳 - Judgment timestamp
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (CMD_is_CarTimer))
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        // 进入编程模式提示符<等待下一组控制命令的到来>
        // Enter the programming mode prompt <waiting for the arrival of the next set of control commands>
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //还有时间
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}
/*
  N3：指令
  CMD模式：<车子控制> 接收并根据 APP端控制命令   执行对车的单方向驱动
  无时间限定
*/
// void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed)
// {
//   static boolean CarControl = false;
//   if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit) //进入小车无时间限定控制模式
//   {
//     CarControl = true;
//     CMD_CarControl(is_CarDirection, is_CarSpeed);
//   }
//   else
//   {
//     if (CarControl == true)
//     {
//       CarControl = false;
//     }
//   }
// }

/*
  N3：指令
  CMD模式：<车子控制> 接收并根据 APP端控制命令   执行对车的单方向驱动
  无时间限定

  Google Translate:
  N3: instruction
  CMD mode: <car control> receives and executes unidirectional driving of the car according to the APP control command
  No time limit
*/
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  // 进入小车无时间限定控制模式 - Enter the trolley timeless control mode
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit)
  {
    CarControl = true;
    CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}

/*
  N4 : 指令
  CMD模式：运动模式<电机控制>
  接收并根据 APP端控制命令 执行对左右电机转速的控制

  Google Translate:
  N4: Command
  CMD mode: motion mode <motor control>
  Receive and execute the control of the left and right motor speed according to the APP control command
*/
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R)
{
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (is_Speed_L == 0 && is_Speed_R == 0)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_Speed_L,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_Speed_R,
                                             /*controlED*/ control_enable); //Motor control
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(void)
{
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (CMD_is_MotorSpeed_L == 0 && CMD_is_MotorSpeed_R == 0)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ CMD_is_MotorSpeed_L,
                                             /*direction_B*/ direction_just, /*speed_B*/ CMD_is_MotorSpeed_R,
                                             /*controlED*/ control_enable); //Motor control
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}

/*
  N5:指令
  CMD模式：<舵机控制>

  Google Translate:
  N5: instruction
  CMD mode: <Servo control>
*/
void ApplicationFunctionSet::CMD_ServoControl_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ServoControl)
  {
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo*/ CMD_is_Servo, /*unsigned int Position_angle*/ CMD_is_Servo_angle / 10);
    // 进入编程模式提示符<等待下一组控制命令的到来>
    // Enter the programming mode prompt <waiting for the arrival of the next set of control commands
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}
/*
  N7:指令
  CMD模式：<灯光控制>
  有时间限定：时间结束后进入编程模式

  Google Translate:
  N7: Command
  CMD mode: <light control>
  Time limit: enter programming mode after time is over
*/
void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B,
                                                               uint32_t is_LightingTimer)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false; //还有时间标志 - There is a time stamp
  static boolean LightingControl_return = false;

  // 进入灯光有时间限定控制模式 - Enter the light control mode with time limit
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit)
  {
    LightingControl = true;
    // #1设定时间不为..时 (空) - #1 Set time is not: time (empty)
    if (is_LightingTimer != 0)
    {
      // 判断时间戳 - Judgment timestamp
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (is_LightingTimer))
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        // 进入编程模式提示符<等待下一组控制命令的到来>
        // Enter the programming mode prompt <waiting for the arrival of the next set of control commands>
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false; // 还有时间 - Still time
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false; // 还有时间标志 - There is a time stamp
  static boolean LightingControl_return = false;

  // 进入灯光有时间限定控制模式 - Enter the light control mode with time limit
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit)
  {
    LightingControl = true;
    // #1设定时间不为..时 (空) - #1Set time is not: time (empty)
    if (CMD_is_LightingTimer != 0)
    {
      // 判断时间戳 - Judgment timestamp
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (CMD_is_LightingTimer))
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        // 进入编程模式提示符<等待下一组控制命令的到来>
        // Enter the programming mode prompt <waiting for the arrival of the next set of control commands>
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false; // 还有时间 - Still time
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}
/*
  N8:指令
  CMD模式：<灯光控制>
  无时间限定

  N8: instruction
  CMD mode: <light control>
  No time limit
*/
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  static boolean LightingControl = false;
  // 进入灯光无时间限定控制模式 - Enter the light control mode without time limit
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit)
  {
    LightingControl = true;
    CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  // 进入灯光无时间限定控制模式 - Enter the light control mode without time limit
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit)
  {
    LightingControl = true;
    CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}

/*
  N100/N110:指令
  CMD模式：清除所有功能

  Google Translate:
  N100/N110: Command
  CMD mode: clear all functions
*/
void ApplicationFunctionSet::CMD_ClearAllFunctions_xxx0(void)
{
  // 清除所有功能：进入空闲模式    N100:指令
  // Clear all functions: enter idle mode N100: command
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
  }
  // 清除所有功能：进入编程模式     N110:指令
  // Clear all functions: enter programming mode N110: instruction
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Programming_mode)
  {

    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}

/*
  N21:指令
  CMD模式：超声波模块处理 接收并根据 APP端控制命令   反馈超声波状态及数据
  输入：

  Google Translate:
  N21: Command
  CMD mode: Ultrasonic module processing, receiving and feeding back the
  ultrasonic status and data according to APP control commands enter:
*/
void ApplicationFunctionSet::CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get)
{
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/); //超声波数据 - Ultrasound data
  UltrasoundDetectionStatus = is_between(UltrasoundData_cm, 0, ObstacleDetection);
  // 超声波  is_get Start     true：有障碍物 / false:无障碍物
  // Ultrasonic is_get Start true: Obstacles / false: Obstacles
  if (1 == is_get)
  {
    if (true == UltrasoundDetectionStatus)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }
  }
  else if (2 == is_get) // 超声波 is_get data - Ultrasound is_get data
  {
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
}
/*
  N22:指令
  CMD模式：循迹模块 接收并根据 APP端控制命令   反馈循迹状态及数据
  输入：

  Google Translate:
  N22: Command
    CMD mode: The tracking module receives and feeds back tracking status and data according to APP control commands
    enter:
*/
void ApplicationFunctionSet::CMD_TraceModuleStatus_xxx0(uint8_t is_get)
{
  char toString[10];
  if (0 == is_get) // 循迹状态获取左边 - Tracking state gets left
  {
    sprintf(toString, "%d", TrackingData_L);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
    if (true == TrackingDetectionStatus_L)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  else if (1 == is_get) // 循迹状态获取中间 - Tracking state acquisition intermediate
  {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
    if (true == TrackingDetectionStatus_M)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  else if (2 == is_get) // 循迹状态获取右边 - Tracking state gets right
  {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
        if (true == TrackingDetectionStatus_R)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  // 进入编程模式提示符<等待下一组控制命令的到来>
  // Enter the programming mode prompt <waiting for the arrival of the next set of control commands>
  Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
}

/* 
 * End:CMD
 * Graphical programming and command control module
 $ Elegoo & SmartRobot & 2020-06
 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*按键命令*/
void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue) {
    temp_keyValue = get_keyValue;
    switch (get_keyValue) {
    case 1:
      Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case 2:
      Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case 3:
      Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
      break;
    case 4:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    default:

      break;
    }
  }
}
// 红外遥控 - Infrared remote control
void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void)
{
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button /*out*/))
  {
    IRrecv_en = true;
    //Serial.println(IRrecv_button);
  }
  if (true == IRrecv_en)
  {
    switch (IRrecv_button)
    {
    case 1:
      Application_SmartRobotCarxxx0.Motion_Control = Forward;
      break;
    case 2:
      Application_SmartRobotCarxxx0.Motion_Control = Backward;
      break;
    case 3:
      Application_SmartRobotCarxxx0.Motion_Control = Left;
      break;
    case 4:
      Application_SmartRobotCarxxx0.Motion_Control = Right;
      break;
    case 5:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    case  6:
      Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case 7:
      Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case 8:
      Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
      break;
    case 9:
      // 调节适配循迹模块敏感数据段响应 - Adjust the sensitive data segment response of adaptive tracking module
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S += 10;
        if (TrackingDetection_S > 600)
        {
          TrackingDetection_S = 600;
        }
        else if (TrackingDetection_S < 30)
        {
          TrackingDetection_S = 30;
        }
      }

      break;
    case 10:
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S = 250;
      }
      break;
    case 11:
      if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S -= 10;
        if (TrackingDetection_S > 600)
        {
          TrackingDetection_S = 600;
        }
        else if (TrackingDetection_S < 30)
        {
          TrackingDetection_S = 30;
        }
      }
      break;

    default:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    }
    // 方向控制部分实现时长约束控制 - The direction control part realizes time constraint control
    if (IRrecv_button < 5)
    {
      Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
      if (millis() - AppIRrecv.IR_PreMillis > 300)
      {
        IRrecv_en = false;
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        AppIRrecv.IR_PreMillis = millis();
      }
    }
    else
    {
      IRrecv_en = false;
      AppIRrecv.IR_PreMillis = millis();
    }
  }
}

// 串口数据解析 - Serial data analysis
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void)
{
  static String SerialPortData = "";
  char c = ' ';
  if (Serial.available() > 0)
  {
    while (c != '}' && Serial.available() > 0)
    {
      c = Serial.read();
      // Skip newline and carriage return.
      if (c != '\n' && c != '\r') {
        SerialPortData += (char)c;
      }
    }
  }
  if (c == '}') // 数据帧尾部校验 - Data frame tail check.
  {
#if _Test_print
    Serial.println("Received: " + SerialPortData);
#endif
    if (true == SerialPortData.equals("{f}") || true == SerialPortData.equals("{b}") || true == SerialPortData.equals("{l}") || true == SerialPortData.equals("{r}"))
    {
      Serial.print(SerialPortData);
      SerialPortData = "";
      return;
    }
    // 避让测试架 - Avoidance test stand
    if (true == SerialPortData.equals("{Factory}") || true == SerialPortData.equals("{WA_NO}") || true == SerialPortData.equals("{WA_OK}"))
    {
      SerialPortData = "";
      return;
    }
    StaticJsonDocument<200> doc;                                       // 声明一个JsonDocument对象 - Declare a JsonDocument object.
    DeserializationError error = deserializeJson(doc, SerialPortData); // 反序列化JSON数据 - Deserialize JSON data.
    SerialPortData = "";
    if (error)
    {
      Serial.println("error:deserializeJson");
      return;
    }
    else if (!error) // 检查反序列化是否成功 - Check whether the deserialization is successful.
    {
      int control_mode_N = doc["N"];
      char *temp = doc["H"];
      CommandSerialNumber = temp; // 获取新命令的序号 - Get the serial number of the new command.

      // 以下代码块请结合通讯协议V.docx 查看
      //Please view the following code blocks in conjunction with the communication protocol V.docx
      switch (control_mode_N)
      {
      case 1: // <命令：N 1> 电机控制模式 - <Command: N 1> motor control mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl;
        CMD_is_MotorSelection = doc["D1"];
        CMD_is_MotorSpeed = doc["D2"];
        CMD_is_MotorDirection = doc["D3"];

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 2: // <命令：N 2> 小车方向控制：有时间限定模式 - <Command: N 2> Car direction control: time limited mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_CarControl_TimeLimit;
        CMD_is_CarDirection = doc["D1"];
        CMD_is_CarSpeed = doc["D2"];
        CMD_is_CarTimer = doc["T"];
        Application_SmartRobotCarxxx0.CMD_CarControl_Millis = millis();
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 3: // <命令：N 3> 小车方向控制：无时间限定模式 - <Command: N 3> Car direction control: no time limit mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_CarControl_NoTimeLimit;
        CMD_is_CarDirection = doc["D1"];
        CMD_is_CarSpeed = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 4: // <命令：N 4> 电机控制:控制转速模式 - <Command: N 4> Motor control: control speed mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_MotorControl_Speed;
        CMD_is_MotorSpeed_L = doc["D1"];
        CMD_is_MotorSpeed_R = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 5: // <命令：N 5> 编程控制舵机 - <Command: N 5> Program control servo
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_ServoControl;
        CMD_is_Servo = doc["D1"];
        CMD_is_Servo_angle = doc["D2"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 7: // <命令：N 7> 灯光控制:有时间限定模式 - <Command: N 7> Light control: time limited mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_LightingControl_TimeLimit;

        CMD_is_LightingSequence = doc["D1"]; //Lighting (Left, front, right, back and center)
        CMD_is_LightingColorValue_R = doc["D2"];
        CMD_is_LightingColorValue_G = doc["D3"];
        CMD_is_LightingColorValue_B = doc["D4"];
        CMD_is_LightingTimer = doc["T"];
        Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = millis();
#if _is_print
        // Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 8: // <命令：N 8> 灯光控制:无时间限定模式 - <Command: N 8> Light control: no time limit mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_LightingControl_NoTimeLimit;

        CMD_is_LightingSequence = doc["D1"]; //Lighting (Left, front, right, back and center)
        CMD_is_LightingColorValue_R = doc["D2"];
        CMD_is_LightingColorValue_G = doc["D3"];
        CMD_is_LightingColorValue_B = doc["D4"];
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 21: // <命令：N 21>：超声波模块:测距 - <Command: N 21>: Ultrasonic Module: Ranging
        CMD_UltrasoundModuleStatus_xxx0(doc["D1"]);
#if _is_print
        // Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 22: // <命令：N 22>：红外模块：寻迹 - <Command: N 22>: Infrared Module: Tracking
        CMD_TraceModuleStatus_xxx0(doc["D1"]);
#if _is_print
        // Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 23: // <命令：N 23>：是否离开地面 - <Command: N 23>: Whether to leave the ground
        if (true == Car_LeaveTheGround)
        {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_false}");
#endif
        }
        else if (false == Car_LeaveTheGround)
        {
#if _is_print
          Serial.print('{' + CommandSerialNumber + "_true}");
#endif
        }
        break;

      case 110: // <命令：N 110> 清除功能:进入编程模式 - <Command: N 110> Clear function: enter programming mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Programming_mode;
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 100: // <命令：N 100> 清除功能：进入空闲模式 - <Command: N 100> Clear function: enter idle mode
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_ClearAllFunctions_Standby_mode;
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 101: // <命令：N 101> :遥控切换命令 - <Command: N 101>: Remote control switching command
        if (1 == doc["D1"])
        {
          Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
        }
        else if (2 == doc["D1"])
        {
          Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
        }
        else if (3 == doc["D1"])
        {
          Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
        }

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 105: // <命令：N 105> :FastLED亮度调节控制命令 - <Command: N 105>: FastLED brightness adjustment control command
        if (1 == doc["D1"] && (CMD_is_FastLED_setBrightness < 250))
        {
          CMD_is_FastLED_setBrightness += 5;
        }
        else if (2 == doc["D1"] && (CMD_is_FastLED_setBrightness > 0))
        {
          CMD_is_FastLED_setBrightness -= 5;
        }
        FastLED.setBrightness(CMD_is_FastLED_setBrightness);

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      case 106: // <命令：N 106> - <Command: N 106> Camera rotation
      {
        uint8_t temp_Set_Servo = doc["D1"];
        if (temp_Set_Servo > 5 || temp_Set_Servo < 1)
          return;
        ApplicationFunctionSet_Servo(temp_Set_Servo);
      }

#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;
      case 102: // <命令：N 102> :摇杆控制命令 - <Command: N 102>: Joystick control command
        Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
        Rocker_temp = doc["D1"];

        switch (Rocker_temp)
        {
        case 1:
          Application_SmartRobotCarxxx0.Motion_Control = Forward;
          break;
        case 2:
          Application_SmartRobotCarxxx0.Motion_Control = Backward;
          break;
        case 3:
          Application_SmartRobotCarxxx0.Motion_Control = Left;
          break;
        case 4:
          Application_SmartRobotCarxxx0.Motion_Control = Right;
          break;
        case 5:
          Application_SmartRobotCarxxx0.Motion_Control = LeftForward;
          break;
        case 6:
          Application_SmartRobotCarxxx0.Motion_Control = LeftBackward;
          break;
        case 7:
          Application_SmartRobotCarxxx0.Motion_Control = RightForward;
          break;
        case 8:
          Application_SmartRobotCarxxx0.Motion_Control = RightBackward;
          break;
        case 9:
          Application_SmartRobotCarxxx0.Motion_Control = stop_it;
          Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
          break;
        default:
          Application_SmartRobotCarxxx0.Motion_Control = stop_it;
          break;
        }
#if _is_print
        Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
        break;

      default:
        break;
      }
    }
  }
}
