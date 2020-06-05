#include<vio.hpp>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/Imu.h>
#include<cv_bridge/cv_bridge.h>
#include<geometry_msgs/Vector3.h>
#include<eigen3/Eigen/Dense>
#include<string>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<kalman.hpp>

namespace huro{
    vio_estimator::vio_estimator(const ros::NodeHandle& nh, 
            const cv::Mat& K,
            const std::string& image_topic, 
            const std::string& depth_topic,
            const std::string& imu_topic): 
            nh_(nh), it_(nh),
            K_(K){

        image_sub_ = it_.subscribe(image_topic, 2, &vio_estimator::image_update_cb, this);
        depth_sub_ = it_.subscribe(depth_topic, 2, &vio_estimator::depth_update_cb, this);
        imu_sub_ = nh_.subscribe(imu_topic, 2, &vio_estimator::imu_update_cb, this);

        vio_pub_ = nh_.advertise<geometry_msgs::Vector3>("/automi/odometry", 2);

        pos_k_1_ << 0, 0, 0;
        vel_k_1_ << 0, 0, 0;
        pos_k_ << 0, 0, 0;
        vel_k_ << 0, 0, 0;
        R = cv::Mat::eye(3, 3, CV_32FC1);
        t = cv::Mat::zeros(3, 1, CV_32FC1);
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

        // std::cout << "Step 1" << std::endl;
        im_k_1_ = im_k_;
        cv::cvtColor(ptr->image, im_k_, cv::COLOR_BGR2GRAY);
        if (im_k_.cols>0 && im_k_.rows>0 && im_k_1_.cols>0 && im_k_1_.rows>0){
            // cv::imshow("current", im_k_);
            // cv::imshow("prev", im_k_1_);
            // cv::waitKey(5);
            update();
        }
        return;
    }

    void vio_estimator::depth_update_cb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }

        if (ptr->image.rows>0 && ptr->image.cols>0){
            depth_k_1_ = depth_k_;
            depth_k_ = ptr->image;
        }
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

        // std::cout << acc_ << std::endl;
        // std::cout << acc_cov_ << std::endl;
        return;
    }

    void vio_estimator::update(){
        if (depth_k_1_.rows==0 || depth_k_.rows==0)
            return;
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat des1, des2, rmat, tvect;
        extract_features(im_k_1_, kp1, des1);
        extract_features(im_k_, kp2, des2);
        std::vector<std::vector<cv::DMatch>> matches = match_features(des1, des2);
        std::vector<cv::DMatch> filtered = filter_matches(matches);
        estimate_motion(filtered, kp1, kp2, K_, rmat, tvect, depth_k_1_, depth_k_);

        //find absolute position in ground frame. 
        cv::Mat r_inv_ = rmat.inv();
        cv::Mat r_inv, t_vect;
        tvect.convertTo(t_vect, CV_32FC1);
        r_inv_.convertTo(r_inv, CV_32FC1);
        R = R * r_inv;

        //R is the absolute rotation matrix 
        //likewise t is the absolute translation vector
        //fuse this with the acceleration obtained from IMU data. 
        //kalman filter problem 

        //fuse this rotation(r_inv) with the one obtained from IMU
        //use fused rotation to get absolute R = R*r_inv

        //fuse translation(t_vect) with IMU data to get t
        //use husky playpen world to test 

        t = t + R * t_vect;
        std::cout << R << std::endl;
        std::cout << t << std::endl;
        std::cout << tvect << std::endl << std::endl;

    }
}