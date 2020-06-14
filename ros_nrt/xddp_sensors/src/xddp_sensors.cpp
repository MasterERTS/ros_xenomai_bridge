#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "xddp_ros.h"
#include "ChatterXddp.h"
#include "ListenerXddp.h"

#define XDDP_PORT_LASER 0
#define XDDP_PORT_ODOM  1

float left_distance = 0.0;
float front_distance = 0.0;
float right_distance = 0.0;

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
    left_distance = *std::min_element(msg->ranges.begin() + 2* split_size, msg->ranges.begin() + ranges_len);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float px, py, oz;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    oz = msg->pose.pose.orientation.z;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "xddp_sensors");
    char* buffer;
    char fwd_buffer[256];
    ros::NodeHandle nh_;
    ros::Subscriber subLaser = nh_.subscribe<sensor_msgs::LaserScan>("/base_scan", 10, &laserCallback);
    //ros::Subscriber subOdometry = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &odomCallback);
    
    ChatterXDDP laser_chatter("/laser_nrt", XDDP_PORT_LASER);
    //ChatterXDDP odom_chatter("/odom_nrt", XDDP_PORT_ODOM);

    std::size_t laser_fail_count = 0, odom_fail_count = 0;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        sprintf(fwd_buffer, "[%f | %f | %f]", left_distance, front_distance, right_distance);
        laser_chatter.nrt_thread_write(fwd_buffer);
        buffer = laser_chatter.nrt_thread_read();
        if (!(buffer[0] == 'a' && buffer[1] == 'c' && buffer[2] == 'k'))
        {
            laser_fail_count++;
            if (laser_fail_count % 10 == 0){
                ROS_INFO("Laser Data Failed to be acknowledged %d times !!", laser_fail_count);
            } else {
                ROS_INFO("RT Thread |ACK| received...")
            }
        }
        /*
        sprintf(fwd_buffer, "[%f | %f | %f]", px, py, oz);
        odom_chatter.nrt_thread_write(fwd_buffer);
        buffer = odom_chatter.nrt_thread_read();
        if (buffer != "ack") 
        {
            ROS_INFO("Laser Data Failed to be acknowledged");
            odom_fail_count++;
        }
        */
        ros::spinOnce();
        loop_rate.sleep();
    }
}
