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

    huro::depth_generator gen(nh, "image1", "image2");

    while (ros::ok()){
        cv::Mat depth = cv::Mat::zeros(5, 5, CV_64FC1);      
        cv::Mat disparity = cv::Mat::zeros(5, 5, CV_64FC1); 
        gen.calc_depth(depth, disparity);

        std::cout << depth;
        // if (depth.rows > 0 && depth.cols > 0){
        //     // cv::Mat deptht;
        //     // cv::ximgproc::getDisparityVis(depth, deptht, 6);
        //     cv::imshow("depth", deptht);
        //     cv::waitKey(1);
        // }
        // std::cout << depth << std::endl;
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}
