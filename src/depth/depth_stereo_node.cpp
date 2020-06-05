#include<vector>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<depth_stereo.hpp>
#include<iostream>
#include<opencv2/ximgproc/disparity_filter.hpp>

int main(int argc, char**argv){
    ros::init(argc, argv, "depth_stereo");
    ros::NodeHandle nh;

    huro::depth_generator gen(nh, "/husky/camera1/image_raw", "/husky/camera2/image_raw");

    while (ros::ok()){
        cv::Mat depth = gen.calc_depth();
        // if (depth.rows > 0 && depth.cols > 0){
        //     // cv::Mat deptht;
        //     // cv::ximgproc::getDisparityVis(depth, deptht, 4);
        //     cv::imshow("depth", depth);
        //     cv::waitKey(1);
        // }
        // std::cout << depth << std::endl;
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}