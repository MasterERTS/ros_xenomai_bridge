#include "xddp_utilities/ChatterXddp.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "xddp_talker");
    xddp_utilities::ChatterXDDP chatter("/xddp");

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        char* buf = chatter.nrt_thread();
        chatter.publish(buf);        
        ros::spinOnce();
        loop_rate.sleep();
    }
}