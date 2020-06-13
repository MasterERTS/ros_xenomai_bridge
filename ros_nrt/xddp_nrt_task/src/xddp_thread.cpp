#include "ChatterXddp.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "xddp_nrt_task");
    ChatterXDDP chatter("/xddp");

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        char* buf = chatter.nrt_thread();
        chatter.publish(buf);        
        ros::spinOnce();
        loop_rate.sleep();
    }
}