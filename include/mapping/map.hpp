#pragma once

#include<eigen3/Eigen/Dense>
#include<container.hpp>
#include<vector>
#include<geometry_msgs/Vector3.h>
#include<automi_v1/object.h>
#include<automi_v1/map.h>

namespace huro::Map{

	class Map{
		friend class map_manager;

		public:
			Map();
			// ~Map();
			static automi_v1::map mapToMsg(const Map& map);
			static Map msgToMap(const automi_v1::map& msg);
			
			Eigen::Vector3d botPos() const {return automi.pos();} 
			void update(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient, const TYPE& type);

		protected:
			Bot automi;
			std::vector<Container> objects;
	};
	
}
