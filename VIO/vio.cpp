#include<vio.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Imu.h>
#include<cv_bridge/cv_bridge.h>
#include<eigen3/Eigen/Dense>
#include<string>
#include<opencv2/opencv.hpp>

namespace huro{
    vio_estimator::vio_estimator(const ros::NodeHandle& nh, 
            const std::string& image_topic, 
            const std::string& imu_topic): 
            nh_(nh), it_(nh){

        // image_left_sub_ = it_.subscribe(image_left_topic, 1, &depth_generator::left_update_callback, this);
        image_sub_ = it_.subscribe(image_topic, 1, &vio_estimator::image_update_cb, this);
        imu_sub_ = nh_.subscribe(imu_topic, 1, &vio_estimator::imu_update_cb, this);
    }

    void vio_estimator::image_update_cb(const sensor_msgs::ImageConstPtr& frame){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }

        im_k_1_ = im_k_;
        cv::cvtColor(ptr->image, im_k_, cv::COLOR_BGR2GRAY);
        return;
    }

    void vio_estimator::imu_update_cb(const sensor_msgs::Imu::ConstPtr& msg){
        acc_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        acc_cov_ << msg->linear_acceleration_covariance[0],
                    msg->linear_acceleration_covariance[1],
                    msg->linear_acceleration_covariance[2],
                    msg->linear_acceleration_covariance[3],
                    msg->linear_acceleration_covariance[4],
                    msg->linear_acceleration_covariance[5],
                    msg->linear_acceleration_covariance[6],
                    msg->linear_acceleration_covariance[7],
                    msg->linear_acceleration_covariance[8];

        return;
    }
}