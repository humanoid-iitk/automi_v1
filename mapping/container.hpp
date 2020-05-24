#pragma once

#include<eigen3/Eigen/Dense>
#include<array>
#include<utility>

namespace huro::Map{

	enum class TYPE{BOX, RECTANGLE, CIRCLE, SPHERE};
	
	class Bot{
		friend class Map;
		
		public:
			Bot(const Eigen::Vector3d& init_pos);

		protected:
			Eigen::Vector3d pos;
	};

	template<int T, TYPE type>
	class Container{
		friend class Map;

		public:	
			Container(const Eigen::Vector3d& pos, 
					  const Eigen::Vector3d& orient);

		protected:
			Eigen::Vector3d pos;
			Eigen::Vector3d orient;							//Vector in direction of central axis
			std::array<std::pair<float, float>, T> dims;	//Dimensions: value, confidence
			TYPE type;
			float occupancy_ratio;							//what ratio of container is occupied by object
	};

	typedef Container<3, TYPE::BOX> Box;				//l,b,h
	typedef Container<1, TYPE::SPHERE> Sphere;			//radius
	typedef Container<1, TYPE::CIRCLE> Circle;			//radius
	typedef Container<2, TYPE::RECTANGLE> Rectangle;	//l, b
}
