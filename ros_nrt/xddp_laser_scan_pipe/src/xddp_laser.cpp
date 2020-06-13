#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "xddp_ros.h"
#include "ChatterXddp.h"
#include "ListenerXddp.h"

// This node subscribe to base_scan, sends data through the xddp pipe, reads it back from a rt thread and publishes the new data as laser_scan

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "sensor_exchange");
}
