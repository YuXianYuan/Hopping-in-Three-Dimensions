/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 单腿跳跃机器人 仿真环境 接口文件
**  Version    : 
**  Notes      : 
**  Author     : 于宪元
**********************************************************/
#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>

#include "webotsInterface.h"

//-----------------------------------------------------------device
WbDeviceTag spring_motor;
WbDeviceTag spring_pos_sensor;
WbDeviceTag touch_sensor;
WbDeviceTag IMU;
WbDeviceTag X_motor;
WbDeviceTag Z_motor;
WbDeviceTag X_motor_pos_sensor;
WbDeviceTag Z_motor_pos_sensor;
/*
函数功能：初始化devices
*/
void webots_device_init()
{
  //get device
  spring_motor       = wb_robot_get_device("linear motor");
  spring_pos_sensor  = wb_robot_get_device("position sensor");
  touch_sensor       = wb_robot_get_device("touch sensor");
  IMU                = wb_robot_get_device("inertial unit");
  X_motor            = wb_robot_get_device("X rotational motor");
  Z_motor            = wb_robot_get_device("Z rotational motor");
  X_motor_pos_sensor = wb_robot_get_device("X position sensor");
  Z_motor_pos_sensor = wb_robot_get_device("Z position sensor");
  //enable
  wb_position_sensor_enable(spring_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(X_motor_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(Z_motor_pos_sensor, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor,TIME_STEP);
  wb_keyboard_enable(TIME_STEP);
  wb_inertial_unit_enable(IMU,TIME_STEP);
}
//-----------------------------------------------------------motor
/*
函数功能：设置虚拟弹簧的力
注    意：弹簧力正方向定义为：腿不动，将机身向y轴方向推动的力
*/
void set_spring_force(double force)
{
  wb_motor_set_force(spring_motor, -force);
}
/*
函数功能：设置臀部 X 轴电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 X 轴旋转的力
*/
void set_X_torque(double torque)
{
  wb_motor_set_torque(X_motor, torque);
}
/*
函数功能：设置臀部 Z 轴电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 Z 轴旋转的力
*/
void set_Z_torque(double torque)
{
  wb_motor_set_torque(Z_motor, torque);
  
}
//-----------------------------------------------------------sensor
/*
函数功能：获取弹簧长度
*/
double get_spring_length()
{
  double length = wb_position_sensor_get_value(spring_pos_sensor);
  return -length + 0.8;
}
/*
函数功能：获取 X 轴电机角度,角度制
*/
double get_X_motor_angle()
{
  double angle = wb_position_sensor_get_value(X_motor_pos_sensor);
  return angle*180.0f/PI;
}
/*
函数功能：获取 Z 轴电机角度,角度制
*/
double get_Z_motor_angle()
{
  double angle = wb_position_sensor_get_value(Z_motor_pos_sensor);
  return angle*180.0f/PI;
}
/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching()
{
  return wb_touch_sensor_get_value(touch_sensor);
}

/*
函数功能：读取IMU数据
*/
eulerAngleTypeDef get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  
  eulerAngleTypeDef eulerAngle;
  eulerAngle.roll  = data[0]*180.0f/PI;
  eulerAngle.pitch = data[1]*180.0f/PI;
  eulerAngle.yaw   = data[2]*180.0f/PI;
  
  return eulerAngle;
}

//-----------------------------------------------------------keyboard
/*
函数功能：读取键盘键值
*/
int get_keyboard()
{
  return wb_keyboard_get_key();
}

