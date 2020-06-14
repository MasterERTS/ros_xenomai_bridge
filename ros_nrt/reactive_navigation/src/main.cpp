#include "robot_controller.h"

/**
 * @brief Run the robot controller node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "robot_controller");

    std::size_t i = 0;

    // Create our controller object and run it
    auto controller = RobotController();
    controller.run();

    // And make good on our promise
    return 0;
}