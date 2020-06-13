#pragma once

#include<eigen3/Eigen/Dense>
#include<vector>

#include<geometry_msgs/Vector3.h>
#include<automi_v1/object.h>
#include<automi_v1/map.h>

#include<container.hpp>

namespace huro::Map{

	class Map{
		friend class manager;

		public:
			Map();
			// ~Map();
			static automi_v1::map mapToMsg(const Map& map);
			static Map msgToMap(const automi_v1::map& msg);
			static Map msgToMap(const automi_v1::map::ConstPtr& msg);
			
			Eigen::Vector3d botPos() const {return automi.pos();} 
			void update(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient, const TYPE& type);
			void update(const Container& obj);

		protected:
			std::vector<Container>::iterator find_(const Eigen::Vector3d& pos, 
												   const float& tolerance=0.2);
			//range(as fraction of dimension) in direction of vector connecting the 2 centers

			std::vector<Container>::iterator find_(const Container& obj, 
												   const float& tolerance=0.2);									   	
			//or overlap area/volume permissible after appropriate expansion. IOU basically. 

			//if an object is already there, the update function will assume the two to be the same 
			//and update the Container object after applying a certain filter algorithm. 
			//else add a new container. (Heap, or sorted list, or tree?)

			Bot automi;
			std::vector<Container> objects;
	};
	
}
