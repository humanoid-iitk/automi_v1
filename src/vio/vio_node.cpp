#include<vio.hpp>
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "vio_node");
    ros::NodeHandle nh;
    huro::vio_estimator vio(nh, cv::Mat(), "/husky/camera1/image_raw", "/imu/data");

    ros::spin();
    return 0;
}