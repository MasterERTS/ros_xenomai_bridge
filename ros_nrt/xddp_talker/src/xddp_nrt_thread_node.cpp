#include "ChatterXddp.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "xddp_talker");
    ChatterXDDP chatter("/xddp", 2);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        char* buf = chatter.nrt_thread_read_write();
        chatter.publish(buf);        
        ros::spinOnce();
        loop_rate.sleep();
    }
}