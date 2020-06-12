#ifndef XDDP_ROS_H
#define XDDP_ROS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string>

#define PIPE_XDDP 2

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

/**
 * @brief Construct a new Minimal Publisher:: Minimal Publisher object
 * 
 */
MinimalPublisher::MinimalPublisher(std::string topic) {
    this->pub = nh_.advertise<std_msgs::String>(topic, 10);
}

std::string MinimalPublisher::GetTopic() {
    return(topic);
}

void MinimalPublisher::publish(std::string str_msg) {
    std_msgs::String msg;
    msg.data = str_msg;
    this->pub.publish(msg);
}


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

#endif