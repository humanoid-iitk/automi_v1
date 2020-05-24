#include<vio.hpp>
#include<ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "vio_node");
    ros::NodeHandle nh;
    huro::vio_estimator vio(nh, "/husky/camera1/image_raw", "/husky/imu");

    ros::spin();
    return 0;
}