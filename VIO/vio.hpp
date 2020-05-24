#pragma once

#include<string>
#include<opencv2/opencv.hpp>
#include<visual_odometry.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<image_transport/image_transport.h>
#include<eigen3/Eigen/Dense>
#include<sensor_msgs/Imu.h>

namespace huro{
    class vio_estimator{
        public:
            vio_estimator(const ros::NodeHandle& nh, 
                          const std::string& image_topic, 
                          const std::string& imu_topic);

        protected:
            Eigen::Vector3d pos_k_;
            Eigen::Vector3d pos_k_1_;
            Eigen::Vector3d vel_k_;
            Eigen::Vector3d vel_k_1_;

            void image_update_cb(const sensor_msgs::ImageConstPtr& frame);
            void imu_update_cb(const sensor_msgs::Imu::ConstPtr& msg);

            cv::Mat im_k_;
            cv::Mat im_k_1_;
            Eigen::Vector3d acc_;             
            Eigen::Matrix3d acc_cov_;           
            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            
            image_transport::Subscriber image_sub_;
            ros::Subscriber imu_sub_;
    };
}