/**
 * @file robot_controller.h
 * @author Nicolas Filliol <nicolas.filliol@icloud.com>, Erwin Lejeune <erwin.lejeune15@gmail.com>, 
 *         Alexander Koreiba, Jan Tiepelt, 
 *         Giovanni Alexander Bergamaschi
 * @brief Robot Controller Class
 * @version 0.1
 * @date 2019-08-11
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <iostream>
#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define THRESHOLD_DISTANCE 0.3


class RobotController {

private:

    ros::NodeHandle node_handle;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    float left_distance;
    float front_distance;
    float right_distance;

    // PID control
    float old_prop_error;
    float integral_error;
    
    float target_value = THRESHOLD_DISTANCE;
    float KP = 10.0;
    float KI = 0.0;
    float KD = 0.0;
    float time_interval = 0.1;

    bool robot_lost;
    int lost_counter;

    geometry_msgs::Twist calculateCommand();


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    float calculateGain(float value);
    void calculateRobotLost();

public:

    RobotController();
    void run();

};

#endif /** ROBOT_CONTROLLER_H **/