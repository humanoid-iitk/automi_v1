#include<iostream>
#include<vector>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "objective_func.hpp"

std::vector<float> histogram;
void histocb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    for(std::vector<float>::const_iterator it = msg->data.begin(); it!=msg->data.end(); ++it)
    histogram.push_back(*it);
}

int main(int argc, char** argv){
    ros::init(argc,argv, "controller");
    ros::NodeHandle nh;

    ros::Publisher loss_pub = nh.advertise<std_msgs::Float64>("loss",);
    ros::Subscriber histo_sub = nh.subscribe("histogram_topic", ,histo_cb);


    ros::Rate loop_rate(10);

    while(ros:ok)
    {
        std_msgs::Float64 loss;
        loss.data = loss_func(histogram, x0 , uref , u);
        loss_pub.publish(loss);

        ros::SpinOnce();
        loop_rate.sleep();
    }


}