/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-14 15:34:07
 * @LastEditors: Changhua
 * @Description: conqueror robot tank
 * @FilePath: 
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"
#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

bool MPU6050_getdata::MPU6050_dveInit(void)
{
  Wire.begin();
  // uint8_t chip_id = 0x00;
  // uint8_t cout;
  // do
  // {
  //   chip_id = accelgyro.getDeviceID();
  //   Serial.print("MPU6050_chip_id: ");
  //   Serial.println(chip_id);
  //   delay(10);
  //   cout += 1;
  //   if (cout > 10)
  //   {
  //     return true;
  //   }
  // } while (chip_id == 0X00 || chip_id == 0XFF); //确保从机设备在线（强行等待 获取 ID ）
  accelgyro.initialize();
  // unsigned short times = 100; //采样次数
  // for (int i = 0; i < times; i++)
  // {
  //   gz = accelgyro.getRotationZ();
  //   gzo += gz;
  // }
  // gzo /= times; //计算陀螺仪偏移
  return false;
}
bool MPU6050_getdata::MPU6050_calibration(void)
{
  unsigned short times = 100; //采样次数
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    gzo += gz;
  }
  gzo /= times; //计算陀螺仪偏移

  // gzo = accelgyro.getRotationZ();
  return false;
}
bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw)
{
  unsigned long now = millis();           //当前时间(ms)
  dt = (now - lastTime) / 1000.0;         //微分时间(s)
  lastTime = now;                         //上一次采样时间(ms)
  gz = accelgyro.getRotationZ();          //获取原始数据
  float gyroz = -(gz - gzo) / 131.0 * dt; //z轴角速度
  if (fabs(gyroz) < 0.05)                 //清除瞬间零漂信号
  {
    gyroz = 0.00;
  }
  agz += gyroz; //z轴角速度积分
  *Yaw = agz;
  return false;
}
