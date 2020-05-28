#pragma once

#include<eigen3/Eigen/Dense>
#include<array>
#include<utility>

namespace huro{

	enum class TYPE{BOX, RECTANGLE, CIRCLE, SPHERE};
	
	class Bot{
		friend class Map;

		public:
			Bot(const Eigen::Vector3d& pos=Eigen::Vector3d(0, 0, 0)): pos(pos){}

		protected:
			Eigen::Vector3d pos;
			//bot orient?
	};

	template<int T, TYPE t>
	class Container{
		friend class Map;

		public:	
			Container(const Eigen::Vector3d& pos, 
					  const Eigen::Vector3d& orient):
					pos(pos), 
					orient(orient), 
					type(t){}

		protected:
			Eigen::Vector3d pos;
			Eigen::Vector3d orient;							//Vector in direction of central axis
			std::array<std::pair<float, float>, T> dims;	//Dimensions: value, confidence
			TYPE type;
			float occupancy_ratio;							//what ratio of container is occupied by object
	};

	using Box = Container<3, TYPE::BOX>;				//l,b,h
	using Sphere = Container<1, TYPE::SPHERE>;			//radius
	using Circle = Container<1, TYPE::CIRCLE>;			//radius
	using Rectangle = Container<2, TYPE::RECTANGLE>;	//l, b
}
