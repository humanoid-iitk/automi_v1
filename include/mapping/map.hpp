#pragma once

#include<eigen3/Eigen/Dense>
#include<container.hpp>
#include<vector>
#include<ros/ros.h>
#include<geometry_msgs/Vector3.h>
#include<automi_v1/object.h>

namespace huro{
	class Map{
		public:
			Map();
			~Map();
			void update(const Eigen::Vector3d& pos, const Eigen::Vector3d& orient, const TYPE type);
			void updateBot(const Eigen::Vector3d& pos);

		protected:
			//Map components
			Bot automi;
			std::vector<Box> boxes;
			std::vector<Rectangle> rectangles;
			std::vector<Sphere> spheres;
			std::vector<Circle> circles; 

			//ROS components
			void bot_update_cb(const geometry_msgs::Vector3::ConstPtr& msg);
			void object_update_cb(const automi_v1::object::ConstPtr& msg);

			ros::NodeHandle nh_;
			ros::Subscriber bot_sub;
			ros::Subscriber object_sub;
			ros::Publisher map_pub;
	};

}
