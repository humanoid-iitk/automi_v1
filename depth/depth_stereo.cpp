#include<string>
#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<opencv2/calib3d.hpp>
#include<depth_stereo.hpp>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>

namespace huro{

    void depth_generator::left_update_callback(const sensor_msgs::ImageConstPtr& left){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }

        cvtColor(ptr->image, im_left_, cv::COLOR_BGR2GRAY);
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

        cvtColor(ptr->image, im_right_, cv::COLOR_BGR2GRAY);
        return;
    }

    depth_generator::depth_generator(const ros::NodeHandle& nh, 
                                     const std::string& image_left_topic="image_topic_1", 
                                     const std::string& image_right_topic="image_topic_2")
    : it_(nh) {
        image_left_sub_ = it_.subscribe(image_left_topic, 1, &depth_generator::left_update_callback, this);
        image_right_sub_ = it_.subscribe(image_right_topic, 1, &depth_generator::right_update_callback, this);
    }

    cv::Mat depth_generator::calc_depth(){
        static cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
            min_disparity_, num_disparities_, block_size_, 
            8*3*window_size_*window_size_, 32*3*window_size_*window_size_
        );

        cv::Mat disparity;
        stereo->compute(im_left_, im_right_, disparity);
        disparity.convertTo(disparity, CV_32F);
        disparity = disparity/16;
        
        cv::Mat depth = (focus_ * baseline_)/disparity;
        return depth;
    }
}