#ifndef MINIMAL_PUBLISHER_H
#define MINIMAL_PUBLISHER_H

#include "xddp_ros.h"

/**
 * @brief 
 * 
 */
class MinimalPublisher
{
    public:
        MinimalPublisher(std::string topic);
        std::string GetTopic();
        void publish(std::string msg);
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub;
    protected:
        std::string topic;
};

#endif