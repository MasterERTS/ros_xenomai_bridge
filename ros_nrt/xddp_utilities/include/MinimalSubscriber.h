#ifndef MINIMAL_SUBSCRIBER_H
#define MINIMAL_SUBSCRIBER_H

#include "xddp_ros.h"

/**
 * @brief 
 * 
 */
class MinimalSubscriber
{
    public:
        MinimalSubscriber(std::string topic);
        std_msgs::String GetMessage();
    private:
        void subCallback(const std_msgs::String::ConstPtr &msg);
        ros::NodeHandle nh_;
        ros::Subscriber sub;
    protected:
        std::string topic;
        std_msgs::String cb_msg;
};

#endif