/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-14 15:18:58
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Bootup(void);
  void ApplicationFunctionSet_RGB(void);
  void ApplicationFunctionSet_Expression(void);
  void ApplicationFunctionSet_Rocker(void);             //摇杆
  void ApplicationFunctionSet_Tracking(void);           //循迹
  void ApplicationFunctionSet_Obstacle(void);           //避障
  void ApplicationFunctionSet_Follow(void);             //跟随
  void ApplicationFunctionSet_Servo(uint8_t Set_Servo); //舵机
  void ApplicationFunctionSet_Standby(void);            //待机
  void ApplicationFunctionSet_KeyCommand(void);         //按键命令
  void ApplicationFunctionSet_SensorDataUpdate(void);   //传感器数据更新
  void ApplicationFunctionSet_SerialPortDataAnalysis(void);
  void ApplicationFunctionSet_IRrecv(void);

public: /*CMD*/
  void CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get);
  void CMD_TraceModuleStatus_xxx0(uint8_t is_get);
  void CMD_Car_LeaveTheGround_xxx0(uint8_t is_get);

  void CMD_inspect_xxx0(void);
  void CMD_MotorControl_xxx0(void);
  //void CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed);
  void CMD_CarControlTimeLimit_xxx0(void);
  //void CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer);
  void CMD_CarControlNoTimeLimit_xxx0(void);
  //void CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed);
  void CMD_MotorControlSpeed_xxx0(void);
  void CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R);
  void CMD_ServoControl_xxx0(void);
  void CMD_VoiceControl_xxx0(uint16_t is_VoiceName, uint32_t is_VoiceTimer);
  void CMD_LightingControlTimeLimit_xxx0(void);
  void CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B, uint32_t is_LightingTimer);
  void CMD_LightingControlNoTimeLimit_xxx0(void);
  void CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B);
  void CMD_LEDCustomExpressionControl_xxx0(void);
  void CMD_ClearAllFunctions_xxx0(void);
  void CMD_LEDNumberDisplayControl_xxx0(uint8_t is_LEDNumber);
  void CMD_TrajectoryControl_xxx0(void);

private:
  /*传感器原数据*/
  volatile float VoltageData_V;        //电压数据
  volatile uint16_t UltrasoundData_mm; //超声波数据
  volatile uint16_t UltrasoundData_cm; //超声波数据
  volatile int TrackingData_L;         //循迹数据
  volatile int TrackingData_M;         //循迹数据
  volatile int TrackingData_R;
  /*传感器状态*/
  boolean VoltageDetectionStatus = false;
  boolean UltrasoundDetectionStatus = false;
  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;

public:
  boolean Car_LeaveTheGround = true;

  /*传感器检测*/
  const float VoltageDetection = 7.00;
  const int ObstacleDetection = 20;

  String CommandSerialNumber;
  //uint8_t Rocker_CarSpeed = 255;
  uint8_t Rocker_temp;

public:
  uint16_t TrackingDetection_S = 250;
  uint16_t TrackingDetection_E = 850;
  uint16_t TrackingDetection_V = 950;

public:
  uint8_t CMD_is_Servo;
  uint8_t CMD_is_Servo_angle;

public:
  uint8_t CMD_is_MotorSelection; //motor
  uint8_t CMD_is_MotorDirection;
  uint8_t CMD_is_MotorSpeed;
  uint32_t CMD_is_MotorTimer;

public:
  uint8_t CMD_is_CarDirection; //car
  uint8_t CMD_is_CarSpeed;
  uint32_t CMD_is_CarTimer;

public:
  uint8_t CMD_is_MotorSpeed_L; //motor
  uint8_t CMD_is_MotorSpeed_R;

public:
  uint8_t CMD_is_VoiceName; //voice
  uint16_t CMD_is_controlAudio;
  uint32_t CMD_is_VoiceTimer;

public:
  uint8_t CMD_is_LightingSequence; //Lighting (Left, front, right, back and center)
  uint8_t CMD_is_LightingColorValue_R;
  uint8_t CMD_is_LightingColorValue_G;
  uint8_t CMD_is_LightingColorValue_B;
  uint32_t CMD_is_LightingTimer;

public:
  //LED Custom Expression Control

  // uint8_t CMD_is_LEDCustomExpression_arry[8];
  // uint8_t CMD_is_LEDNumber;

public:
  //Trajectory
  // uint16_t CMD_is_TrajectoryControl_axisPlaneData_X;
  // uint16_t CMD_is_TrajectoryControl_axisPlaneData_Y;

private:
  uint8_t CMD_is_FastLED_setBrightness = 20;
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif