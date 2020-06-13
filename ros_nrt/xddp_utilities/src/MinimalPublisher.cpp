#include "MinimalPublisher.h"

/**
 * @brief Construct a new Minimal Publisher:: Minimal Publisher object
 * 
 */
MinimalPublisher::MinimalPublisher(std::string topic) {
    this->pub = nh_.advertise<std_msgs::String>(topic, 10);
}

/**
 * @brief 
 * 
 * @return std::string 
 */
std::string MinimalPublisher::GetTopic() {
    return(topic);
}

/**
 * @brief 
 * 
 * @param str_msg 
 */
void MinimalPublisher::publish(std::string str_msg) {
    std_msgs::String msg;
    msg.data = str_msg;
    this->pub.publish(msg);
}