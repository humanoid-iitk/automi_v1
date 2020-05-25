#include<cmath>
#include<eigen3/Eigen/Dense>
#include<container.hpp>

namespace huro::Map{

    Bot::Bot(const Eigen::Vector3d& pos): pos(pos){}

    template<int T, TYPE t>
    Container<T, t>::Container(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient):
        pos(pos), 
        orient(orient), 
        type(t){
            
    }
}
