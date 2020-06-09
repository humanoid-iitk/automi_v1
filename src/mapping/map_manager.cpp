#include<map_manager.hpp>
#include<cv_bridge/cv_bridge.h>

namespace huro::Map{
    manager::manager():
        nh_private_(ros::NodeHandle("~"))
        {
            bot_sub_ = nh_.subscribe("localisation_topic", 2, &manager::bot_update_cb, this);
            map_pub_ = nh_private_.advertise<automi_v1::map>("map", 2);
        }

    void manager::bot_update_cb(const geometry_msgs::Vector3::ConstPtr& msg){
        Eigen::Vector3d pos = Eigen::Vector3d(msg->x,
                                              msg->y,
                                              msg->z);
        map_.automi.update(pos);
        map_pub_.publish(Map::mapToMsg(map_));
        return;
    }

    manager_auto::manager_auto():
        manager(),
        it_(nh_){
            image_sub_ = it_.subscribe("image", 2, &manager_auto::image_update_cb, this);
            depth_sub_ = it_.subscribe("depth", 2, &manager_auto::depth_update_cb, this);
        }

    void manager_auto::image_update_cb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }
        
        image_ = ptr->image;
        return;
    }

    void manager_auto::depth_update_cb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr ptr;
        try{
            ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch(cv_bridge::Exception e){
            ROS_ERROR("cv bridge exception: %s", e.what());    
            return;
        }

        depth_ = ptr->image;
        return;
    }

    std::vector<Container> manager_auto::sense(){
        std::vector<Container> objects;
        //algorithm goes here

        return objects;
    }

    //utility function
    void manager_auto::update(const Container& obj){
        map_.update(obj);
        return;
    }

    void manager_auto::senseAndUpdate(){
        std::vector<Container> objects = sense();
        for (auto obj: objects){
            map_.update(obj);
        }

        map_pub_.publish(Map::mapToMsg(map_));
        return;
    }

    manager_manual::manager_manual():
        manager(){
            object_sub_ = nh_.subscribe("object", 2, &manager_manual::object_update_cb, this);
        }
    
    void manager_manual::object_update_cb(const automi_v1::object::ConstPtr& msg){
        Container obj = Container::msgToObj(msg);
        map_.update(obj);

        map_pub_.publish(Map::mapToMsg(map_));
        return;
    }
}