#include<map.hpp>
#include<vector> 
#include<eigen3/Eigen/Dense>

namespace huro::Map{
    Map::Map():automi(Eigen::Vector3d()){
        automi.pos << 0, 0, 0;
    }
}
 


