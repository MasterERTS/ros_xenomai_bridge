#include "xddp_utilities/ChatterXddp.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "chatter");
    xddp_utilities::MinimalPublisher chatter("/chatter");
    ros::Rate loop_rate(10);
    std::size_t i = 0;
    while(ros::ok())
    {
        std::string s = std::to_string(i);
        std::string msg = "Loop " + s;
        std::string strr = "Sent [Loop " + s + "]";
        chatter.publish(msg);
        ROS_INFO(strr.c_str());
        i++;
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}