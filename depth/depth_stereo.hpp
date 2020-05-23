#pragma once

#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/calib3d.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<string>
#include<image_transport/image_transport.h>

namespace huro{
    
    class depth_generator{
        public:
            depth_generator(const ros::NodeHandle& nh, const std::string& image_left_topic, const std::string& image_right_topic);      //initialise parameters
            cv::Mat calc_depth();

        // protected:
            const float min_disparity_ = 0;
            const int num_disparities_ = 6*16;
            const int block_size_ = 11;
            const int window_size_ = 6;
            const float focus_ = 1; 
            const float baseline_ = 3;

            void left_update_callback(const sensor_msgs::ImageConstPtr& left);
            void right_update_callback(const sensor_msgs::ImageConstPtr& right);

            cv::Mat im_left_;
            cv::Mat im_right_;
    
            image_transport::ImageTransport it_;

            image_transport::Subscriber image_left_sub_;
            image_transport::Subscriber image_right_sub_; 
    };

}