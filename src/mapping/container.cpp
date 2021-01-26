#include<container.hpp>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<std_msgs/Header.h>

namespace huro::Map{
    Container Container::create(const Eigen::Vector3d& pos, 
                            const Eigen::Vector3d& orient,
                            const TYPE& type)
    {
        Container obj = Container(pos, orient, type);
        return obj;
    }

    Container Container::create(const Eigen::Vector3d& pos, 
                            const Eigen::Vector3d& orient,
                            const TYPE& type,
                            const std::vector<automi_v1::dimension>& dims)
    {
        Container obj = Container(pos, orient, type);
        for (auto dim: dims){
            obj.dims_[dim.key] = std::pair<float, float>(dim.value, dim.accuracy);
        }

        return obj;
    }

    automi_v1::object Container::objToMsg(const Container& obj){
        automi_v1::object msg;
        msg.header = std_msgs::Header();
        msg.occupancy_ratio = obj.occupancy_;
        msg.type = static_cast<uint8_t>(obj.type_);

        msg.pos.x = obj.pos_(0);
        msg.pos.y = obj.pos_(1);
        msg.pos.z = obj.pos_(2);

        msg.orient.x = obj.orient_(0);
        msg.orient.y = obj.orient_(1);
        msg.orient.z = obj.orient_(2);

        for (auto dim: obj.dims_){
            automi_v1::dimension d;
            d.key=dim.first;
            d.value=dim.second.first;
            d.accuracy=dim.second.second;
            msg.dims.push_back(d);
        }

        return msg;
    }

    Container Container::msgToObj(const automi_v1::object& msg){
        Eigen::Vector3d pos(msg.pos.x, msg.pos.y, msg.pos.z);
        Eigen::Vector3d orient(msg.orient.x, msg.orient.y, msg.orient.z);
        Container obj = Container::create(pos, orient, static_cast<TYPE>(msg.type), msg.dims);
        return obj;
    }

    Container Container::msgToObj(const automi_v1::object::ConstPtr& msg){
        Eigen::Vector3d pos(msg->pos.x, msg->pos.y, msg->pos.z);
        Eigen::Vector3d orient(msg->orient.x, msg->orient.y, msg->orient.z);
        Container obj = Container::create(pos, orient, static_cast<TYPE>(msg->type), msg->dims);
        return obj;
    }


}