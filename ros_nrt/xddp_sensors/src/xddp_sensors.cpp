#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "xddp_ros.h"
#include "ChatterXddp.h"
#include "ListenerXddp.h"

#define XDDP_PORT_LASER 0
#define XDDP_PORT_ODOM  1

float left_distance;
float front_distance;
float right_distance;

std::string laser_msg;
std::string odom_msg;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // We want to emulate three IR sensors with the LIDAR Scan data by splitting it in three front ranges
    int ranges_len = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int split_size = ranges_len/3;

    // Split sensor data into three areas and extract smallest distance (safety)
    right_distance = *std::min_element(msg->ranges.begin(), msg->ranges.begin() + split_size);
    front_distance = *std::min_element(msg->ranges.begin() + split_size, msg->ranges.begin() + split_size*2);
    left_distance = *std::min_element(msg->ranges.begin() + 2* split_size, msg->ranges.begin() + split_size*3);

    laser_msg = "[L=" + std::to_string(left_distance) + " | F=" + std::to_string(front_distance) + " | R=" + std::to_string(right_distance) + "]";
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float px, py, oz;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    oz = msg->pose.pose.orientation.z;
    
    odom_msg = "[PX=" + std::to_string(px) + " | PY=" + std::to_string(py) + " | OZ=" + std::to_string(oz) + "]";
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "sensor_exchange");
    char* buffer;
    ros::NodeHandle nh_;
    nh_.subscribe<sensor_msgs::LaserScan>("base_scan", 10, &laserCallback);
    nh_.subscribe<nav_msgs::Odometry>("odom", 10, &odomCallback);
    
    ChatterXDDP laser_chatter("/laser_nrt", 0);
    ChatterXDDP odom_chatter("/odom_nrt", 1);

    std::size_t laser_fail_count = 0, odom_fail_count = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        laser_chatter.nrt_thread_write(&laser_msg[0]);
        buffer = laser_chatter.nrt_thread_read();
        if (buffer != "ack")
        {
            ROS_INFO("Laser Data Failed to be acknowledged");
            laser_fail_count++;
        }

        odom_chatter.nrt_thread_write(&odom_msg[0]);
        buffer = odom_chatter.nrt_thread_read();
        if (buffer != "ack") 
        {
            ROS_INFO("Laser Data Failed to be acknowledged");
            odom_fail_count++;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
