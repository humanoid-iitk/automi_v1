#pragma once

#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<automi_v1/map.h>
#include<map.hpp>

namespace huro{
    class Planner{
        public:
            Planner();


        protected:
            cv::Mat frame_;
            Map::Map map_;

            virtual void image_update_cb(const sensor_msgs::ImageConstPtr& frame);
            virtual void map_update_cb(const automi_v1::map::ConstPtr& map);

            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;
            ros::Subscriber map_sub_;
            ros::Publisher move_pub_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
    };
}
