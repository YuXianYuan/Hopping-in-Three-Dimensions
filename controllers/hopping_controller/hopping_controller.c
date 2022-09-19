/*
 * File:          hopping_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/keyboard.h>

#include "webotsInterface.h"
#include "controller.h"

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
    /* necessary to initialize webots stuff */
    wb_robot_init();

    /*
     * You should declare here WbDeviceTag variables for storing
     * robot devices like this:
     *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
     *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
     */

    webots_device_init(); // webots设备初始化
    robot_init();         //机器人初始化

    /* main loop
     * Perform simulation steps of TIME_STEP milliseconds
     * and leave the loop when the simulation is over
     */
    while (wb_robot_step(TIME_STEP) != -1)
    {
        /*
         * Read the sensors :
         * Enter here functions to read sensor data, like:
         *  double val = wb_distance_sensor_get_value(my_sensor);
         */
        printf("update\n");

        /* Process sensor data here */
        updateRobotState(); //机器人状态更新

        /*
         * Enter here functions to send actuator commands, like:
         * wb_motor_set_position(my_actuator, 10.0);
         */
        robot_control();    //机器人控制
    };

    /* Enter your cleanup code here */
    robot_free(); //机器人释放内存空间

    /* This is necessary to cleanup webots resources */
    wb_robot_cleanup();

    return 0;
}
