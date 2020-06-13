
#include "MinimalSubscriber.h"

/**
 * @brief Construct a new Minimal Subscriber:: Minimal Subscriber object
 * 
 * @param topic 
 */
MinimalSubscriber::MinimalSubscriber(std::string topic) 
{
    this->topic = topic;
    this->sub = nh_.subscribe<std_msgs::String>(topic, 10, &MinimalSubscriber::subCallback, this);
}

/**
 * @brief 
 * 
 * @return std_msgs::String 
 */
std_msgs::String MinimalSubscriber::GetMessage()
{
    return(cb_msg);
}

/**
 * @brief Message callback
 * 
 * @param msg 
 */
void MinimalSubscriber::subCallback(const std_msgs::String::ConstPtr &msg)
{
    cb_msg = *msg;
    std::string chr_msg = msg->data;
    std::string str_info = "Received [" + chr_msg + "]";
    ROS_INFO(str_info.c_str());
}