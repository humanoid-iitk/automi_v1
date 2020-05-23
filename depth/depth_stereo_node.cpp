#include<vector>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<depth_stereo.hpp>

int main(int argc, char**argv){
    ros::init(argc, argv, "depth_node");
    ros::NodeHandle nh;

    huro::depth_generator gen(nh, "image_topic_1", "image_topic_2");
    ros::Publisher depth_pub = nh.advertise<sensor_msgs::Image>("depth", 1);
    while(ros::ok()){
        cv::Mat depth = gen.calc_depth();
        cv_bridge::CvImagePtr ptr;
        ptr->image = depth;
        depth_pub.publish(ptr->toImageMsg());
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}