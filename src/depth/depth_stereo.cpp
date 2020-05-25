#include<string>
#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include<depth_stereo.hpp>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
// #include<iostream>
#include<opencv2/ximgproc/disparity_filter.hpp>

namespace huro{
    depth_generator::depth_generator(const ros::NodeHandle& nh, 
                                     const std::string& image_left_topic, 
                                     const std::string& image_right_topic)
    : it_(nh) {
        image_left_sub_ = it_.subscribe(image_left_topic, 1, &depth_generator::left_update_callback, this);
        image_right_sub_ = it_.subscribe(image_right_topic, 1, &depth_generator::right_update_callback, this);
    }

    void depth_generator::left_update_callback(const sensor_msgs::ImageConstPtr& left){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }
        // im_left_ = ptr->image;
        cvtColor(ptr->image, im_left_, cv::COLOR_BGR2GRAY);
        if (im_left_.rows > 0 && im_left_.cols > 0)
            cv::imshow("left", im_left_);
        return;
    }

    void depth_generator::right_update_callback(const sensor_msgs::ImageConstPtr& right){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }
        // im_right_ = ptr->image;
        cvtColor(ptr->image, im_right_, cv::COLOR_BGR2GRAY);
        if (im_right_.rows > 0 && im_right_.cols > 0)
            cv::imshow("right", im_right_);
        return;
    }

    cv::Mat depth_generator::calc_depth(){
        static cv::Ptr<cv::StereoBM> left_matcher = cv::StereoBM::create(
            num_disparities_, block_size_
        );
        static cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
        static cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
        wls_filter->setLambda(lambda_);
        wls_filter->setSigmaColor(sigma_);

        cv::Mat disparity, temp_l, temp_r, temp_d;
        if (im_left_.rows > 0 && im_right_.rows>0){
            left_matcher->compute(im_left_, im_right_, temp_l);
            right_matcher->compute(im_right_, im_left_, temp_r);
            wls_filter->filter(temp_l, im_left_, temp_d, temp_r);
            
            temp_d.convertTo(disparity, CV_32F);
            // disparity = disparity/16;

            for (int i=0; i<disparity.rows; i++){
                float* di = disparity.ptr<float>(i);
                for (int j=0; j<disparity.cols; j++){
                    if (di[j] == -1 || di[j] == 0)
                        di[j] = 1e-7;
                }
            }
        }

        // cv::Mat depth = (focus_ * baseline_)/disparity;
        // cv::Mat depth = disparity;
        cv::Mat depth;
        disparity.convertTo(depth, CV_16S);
        return depth;
    }
}