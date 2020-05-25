#pragma once

#include<eigen3/Eigen/Dense>
#include<container.hpp>
#include<vector>

namespace huro::Map{
	class Map{
		public:
			Map();

		protected:
			Bot automi;
			std::vector<Box> boxes;
			std::vector<Rectangle> rectangles;
			std::vector<Sphere> spheres;
			std::vector<Circle> circles; 
	};

}
