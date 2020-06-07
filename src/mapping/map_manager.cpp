#include<map_manager.hpp>

namespace huro::Map{
    map_manager::map_manager():map_(Map()){
        bot_sub = nh_.subscribe("localisation_topic", 1, &map_manager::bot_update_cb, this);
        object_sub = nh_.subscribe("object_rec_topic", 1, &map_manager::object_update_cb, this);
        map_pub = nh_.advertise<automi_v1::map>("map_topic", 1);
    }

    void map_manager::update(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient, const TYPE type){
        Container obj = Container(pos, orient, type);
        
        map_.objects.push_back(obj);

        //publish updated map
        map_pub.publish(Map::mapToMsg(map_));
        return;
    }

    void map_manager::updateBot(const Eigen::Vector3d& pos){
        map_.automi.update(pos);
        //bot orient???
        
        //publish updated map
        map_pub.publish(Map::mapToMsg(map_));
        return;
    }

    void map_manager::bot_update_cb(const geometry_msgs::Vector3::ConstPtr& msg){
        Eigen::Vector3d pos(msg->x, msg->y, msg->z);
        updateBot(pos);
        return;
    }

    void map_manager::object_update_cb(const automi_v1::object::ConstPtr& msg){
        Eigen::Vector3d pos(msg->pos.x, msg->pos.y, msg->pos.z);
        Eigen::Vector3d orient(msg->orient.x, msg->orient.y, msg->orient.z);
        update(pos, orient, static_cast<TYPE>(msg->type));
    }
}