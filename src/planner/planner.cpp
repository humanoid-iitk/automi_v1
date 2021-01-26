#include<planner.hpp>
#include<geometry_msgs/Vector3.h>
#include<cv_bridge/cv_bridge.h>

namespace huro{

    Planner::Planner():
    it_(nh_),
    nh_private_(ros::NodeHandle("~"))
    {
        image_sub_ = it_.subscribe("image", 2, &Planner::image_update_cb, this);
        map_sub_ = nh_.subscribe("map", 2, &Planner::map_update_cb, this);
        move_pub_ = nh_private_.advertise<geometry_msgs::Vector3>("next_move", 2);
    }

    void Planner::image_update_cb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }

        frame_ = ptr->image;
        return;
    }

    void Planner::map_update_cb(const automi_v1::map::ConstPtr& msg){
        
        return;
    }


}