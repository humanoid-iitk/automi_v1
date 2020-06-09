#pragma once

#include<eigen3/Eigen/Dense>
#include<utility>
#include<vector>
#include<map>

#include<automi_v1/object.h>
#include<automi_v1/dimension.h>

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

			static Container create(const Eigen::Vector3d& pos, 
                            const Eigen::Vector3d& orient,
                            const TYPE& type);

			static Container create(const Eigen::Vector3d& pos, 
                            const Eigen::Vector3d& orient,
                            const TYPE& type,
                            const std::vector<automi_v1::dimension>& dims);

			static automi_v1::object objToMsg(const Container& obj);
			static Container msgToObj(const automi_v1::object& msg);
			static Container msgToObj(const automi_v1::object::ConstPtr& msg);
			
			virtual Eigen::Vector3d pos() const {return pos_;}
			virtual double pos(int index) const {return pos_(index);}
			virtual Eigen::Vector3d orient() const {return orient_;}
			virtual double orient(int index) const {return orient_(index);} 
			virtual TYPE type() const {return type_;}
			virtual std::map<std::string, std::pair<float, float>> getDims(){return dims_;}

		protected:
			Eigen::Vector3d pos_;
			Eigen::Vector3d orient_;							//Vector in direction of central axis
			TYPE type_;
			float occupancy_;									//what ratio of container is occupied by object
			std::map<std::string, std::pair<float, float>> dims_;
	};

	//implement factory pattern to generate containers and extend this class according to needs. 

}
