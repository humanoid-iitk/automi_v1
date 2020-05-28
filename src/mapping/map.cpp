#include<map.hpp>
#include<vector> 
#include<eigen3/Eigen/Dense>
#include<container.hpp>
#include<automi_v1/map.h>
#include<automi_v1/object.h>

namespace huro{
    Map::Map():automi(Bot()){
        bot_sub = nh_.subscribe("localisation_topic", 1, &Map::bot_update_cb, this);
        object_sub = nh_.subscribe("object_rec_topic", 1, &Map::object_update_cb, this);
        map_pub = nh_.advertise<automi_v1::map>("map_topic", 1);
    }

    void Map::update(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient, const TYPE type){
        switch (type){
        case TYPE::BOX:{
            Box box(pos, orient);
            boxes.push_back(box);
        }
        break;
        case TYPE::CIRCLE:{
            Circle circle(pos, orient);
            circles.push_back(circle);
        }
            break;
        case TYPE::RECTANGLE:{
            Rectangle rectangle(pos, orient);
            rectangles.push_back(rectangle);
        }
            break;
        case TYPE::SPHERE:{
            Sphere sphere(pos, orient);
            spheres.push_back(sphere);
        }
            break;
        default:
            //Report error and exit
            return;
        }

        //publish updated map
    }

    void Map::updateBot(const Eigen::Vector3d& pos){
        automi.pos = pos;
        //bot orient???
        
        //publish updated map
    }

    void Map::bot_update_cb(const geometry_msgs::Vector3::ConstPtr& msg){

    }

    void Map::object_update_cb(const automi_v1::object::ConstPtr& msg){
        
    }
}


 


