/**********************************************************
**             Email:@qq.com   QQ:1069841355
**---------------------------------------------------------
**  Description: 此文件为 单腿跳跃机器人 逻辑 文件
**  Version    : 
**  Notes      : 使用键盘 ↑ ↓ ← → 控制机器人跳动方向
**  Author     : 于宪元
**********************************************************/
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/keyboard.h>

#include "webotsInterface.h"
#include "controller.h"

int main(int argc, char **argv) {
  
  wb_robot_init();
  webots_device_init();                           //webots设备初始化
  robot_init();                                   //机器人初始化
  while (wb_robot_step(TIME_STEP) != -1) {

  updateRobotState();                             //机器人状态更新
  robot_control();                                //机器人控制  
  }
  robot_free();                                   //机器人释放内存空间
  wb_robot_cleanup();
  return 0;
}


