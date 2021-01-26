#pragma once

#include<eigen3/Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<vector>

#include<ros/ros.h>
#include<geometry_msgs/Vector3.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/Image.h>

#include<automi_v1/object.h>
#include<map.hpp>

namespace huro::Map{
	//factory style code for 2 separate classes: AUTO and MANUAL ????

    class manager{
		public:
			manager();

		protected:
			Map map_;

			void bot_update_cb(const geometry_msgs::Vector3::ConstPtr& msg);

			ros::NodeHandle nh_;
			ros::NodeHandle nh_private_;
			ros::Publisher map_pub_;
			ros::Subscriber bot_sub_;
	};

	class manager_auto : public manager{
		public:
			manager_auto();
			std::vector<Container> sense();
			void update(const Container& obj);
			void senseAndUpdate();

		protected:
			cv::Mat image_, depth_;

			void image_update_cb(const sensor_msgs::ImageConstPtr& msg);
			void depth_update_cb(const sensor_msgs::ImageConstPtr& msg);

			image_transport::ImageTransport it_;
			image_transport::Subscriber image_sub_;
			image_transport::Subscriber depth_sub_;			
	};

	class manager_manual : public manager{
		public:
			manager_manual();

		protected:
			void object_update_cb(const automi_v1::object::ConstPtr& msg);
			ros::Subscriber object_sub_;
	};
}