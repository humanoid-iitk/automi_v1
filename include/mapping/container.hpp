#pragma once

#include<eigen3/Eigen/Dense>
#include<array>
#include<utility>
#include<vector>

namespace huro::Map{

	enum class TYPE{BOX, RECTANGLE, CIRCLE, SPHERE};
	
	class Bot{
		friend class Map;

		public:
			Bot(const Eigen::Vector3d& pos=Eigen::Vector3d(0, 0, 0)): pos_(pos){}
			void update(const Eigen::Vector3d pos) {this->pos_ = pos;}
			Eigen::Vector3d pos() const {return pos_;}

		protected:
			Eigen::Vector3d pos_;
			//bot orient?
	};

	class Container{
		friend class Map;

		public:	
			Container(const Eigen::Vector3d& pos, 
					  const Eigen::Vector3d& orient,
					  const TYPE& type):
					pos_(pos), 
					orient_(orient), 
					type_(type){}

			// static Container* create(const Eigen::Vector3d& pos, 
			// 						const Eigen::Vector3d& orient,				//tryna make it factory style, will do it later ig?
			// 						const TYPE& type,
			// 						const std::vector<float>& dims,
			// 						const std::vector<float>& dims_conf);

			virtual Eigen::Vector3d pos() const {return pos_;}
			virtual double pos(int index) const {return pos_(index);}
			virtual Eigen::Vector3d orient() const {return orient_;} 
			virtual TYPE type() const {return type_;}
			// virtual getDims() = 0;

		protected:
			Eigen::Vector3d pos_;
			Eigen::Vector3d orient_;							//Vector in direction of central axis
			TYPE type_;
			float occupancy_;									//what ratio of container is occupied by object
	};

	class Box : public Container {
		public:
			Box(const Eigen::Vector3d& pos, 
				const Eigen::Vector3d& orient):
				Container(pos, orient, TYPE::BOX){}
		
			// static Container* create(const Eigen::Vector3d& pos, 
			// 				const Eigen::Vector3d& orient,
			// 				const TYPE& type,							//factory method reacts only
			// 				const std::vector<float>& dims,
			// 				const std::vector<float>& dims_conf);

		protected:
			std::array<std::pair<float, float>, 3> dims_;		//l,b,h
	};

	class Sphere : public Container {
		public:
			Sphere(const Eigen::Vector3d& pos, 
				const Eigen::Vector3d& orient):
				Container(pos, orient, TYPE::SPHERE){}					//Do I really need to store type?

		protected:
			std::array<std::pair<float, float>, 1> dims_;		//radius
	};

	class Rectangle : public Container {
		public:
			Rectangle(const Eigen::Vector3d& pos, 
				const Eigen::Vector3d& orient):
				Container(pos, orient, TYPE::RECTANGLE){}

		protected:
			std::array<std::pair<float, float>, 2> dims_;		//l, b
	};

	class Circle : public Container {
		public:
			Circle(const Eigen::Vector3d& pos, 
				const Eigen::Vector3d& orient):
				Container(pos, orient, TYPE::CIRCLE){}

		protected:
			std::array<std::pair<float, float>, 1> dims_;		//radius
	};

}
