#include "xddp_utilities/ListenerXddp.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "listener");
    xddp_utilities::ListenerXDDP listener("/xddp");
    ros::Rate loop_rate(10);
    ros::spin();
}
