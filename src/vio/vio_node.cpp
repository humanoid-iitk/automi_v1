#include<vio.hpp>
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "vio_node");
    ros::NodeHandle nh;
    float data[9] = {1.398, 0, 640, 0, 1.398, 480, 0, 0, 1};
    huro::vio_estimator vio(nh, cv::Mat(cv::Size(3, 3), CV_32FC1, data), "/husky/camera1/image_raw", "/automi/depth", "/imu/data");

    ros::spin();
    return 0;
}