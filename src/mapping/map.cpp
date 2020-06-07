#include<map.hpp>
#include<vector> 
#include<eigen3/Eigen/Dense>
#include<container.hpp>
#include<automi_v1/map.h>
#include<automi_v1/object.h>

namespace huro::Map{

    automi_v1::map toMapMsg(const Map& map){
        automi_v1::map msg;
        Eigen::Vector3d bot_pos = map.botPos();
        msg.bot_pos.x = bot_pos(0);
        msg.bot_pos.y = bot_pos(1);
        msg.bot_pos.z = bot_pos(2);

        for (Container box : map.objects){
            automi_v1::object obj;
            obj.pos.x = box.pos(0);
            obj.pos.y = box.pos(1);
            obj.pos.z = box.pos(2);

            obj.type = static_cast<uint8_t>(box.type());
            // obj.dims = box->dims_;
            msg.objects.push_back(obj);
        }
        
        return msg;
    }

    Map msgToMap(const automi_v1::map::ConstPtr){
        return Map();
    }

    Map::Map():automi(Bot()){}

    map_manager::map_manager():map_(Map()){
        bot_sub = nh_.subscribe("localisation_topic", 1, &map_manager::bot_update_cb, this);
        object_sub = nh_.subscribe("object_rec_topic", 1, &map_manager::object_update_cb, this);
        map_pub = nh_.advertise<automi_v1::map>("map_topic", 1);
    }

    void map_manager::update(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient, const TYPE type){
        Container obj = Container(pos, orient, type);
        
        map_.objects.push_back(obj);

        //publish updated map
        map_pub.publish(toMapMsg(map_));
        return;
    }

    void map_manager::updateBot(const Eigen::Vector3d& pos){
        map_.automi.update(pos);
        //bot orient???
        
        //publish updated map
        map_pub.publish(toMapMsg(map_));
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


 


