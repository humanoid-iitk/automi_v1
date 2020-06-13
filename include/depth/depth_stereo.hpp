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
            depth_generator(const ros::NodeHandle& nh, 
                            const std::string& image_left_topic = "image1", 
                            const std::string& image_right_topic = "image2");//initialise parameters
            
            depth_generator(const ros::NodeHandle& nh,
                            const std::map<std::string, float>& params,
                            const std::string& image_left_topic = "image1",
                            const std::string& image_right_topic = "image2");
            
            void calc_depth(cv::Mat& depth, cv::Mat& disparity);


        protected:
            int num_disparities_ = 2*16;
            int block_size_ = 5;
            float focus_ = 1.3962; 
            float baseline_ = 0.05;
            int lambda_ = 8000;       //Got to configure these values properly
            int sigma_ = 5; 
            bool USE_FILTER = true;
            
            //cv::Mat depth = cv::Mat();
            //cv::Mat disparity = cv::Mat();
            void left_update_callback(const sensor_msgs::ImageConstPtr& left);
            void right_update_callback(const sensor_msgs::ImageConstPtr& right);

            cv::Mat im_left_;
            cv::Mat im_right_;
    
            image_transport::ImageTransport it_;
            ros::NodeHandle nh_private_;
            image_transport::ImageTransport it_private_;

            image_transport::Subscriber image_left_sub_;
            image_transport::Subscriber image_right_sub_; 
            image_transport::Publisher depth_pub_;
    };

}
