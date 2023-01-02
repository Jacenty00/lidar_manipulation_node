/*
 * virtual_sensor_node.hpp
 *
 *  Created on: May 13, 2013
 *      Author: jorge
 */

#ifndef YOCS_VIRTUAL_SENSOR_NODE_HPP_
#define YOCS_VIRTUAL_SENSOR_NODE_HPP_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#include <yocs_math_toolkit/geometry.hpp>
#include <ford_msgs/Clusters.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>
#include <iostream>


// #include <yocs_msgs/Wall.h>
// #include <yocs_msgs/WallList.h>
// #include <yocs_msgs/Column.h>
// #include <yocs_msgs/ColumnList.h>

namespace virtual_sensor
{

	class VirtualSensorNode
	{
	public:
		class Obstacle
		{
		public:
			Obstacle(const std::string &name, const tf::Transform &tf, double height)
				: name_(name), tf_(tf), height_(height) {}
			~Obstacle() {}
			std::string &name() { return name_; }
			double distance() { return distance_; }
			double minHeight() { return tf_.getOrigin().z(); }
			double maxHeight() { return tf_.getOrigin().z() + height_; }
			virtual bool intersects(double rx, double ry, double max_dist, double &distance) = 0;

		protected:
			std::string name_;
			tf::Transform tf_;
			double distance_;
			double height_;
		};

		class Column : public Obstacle
		{
		public:
			Column(const std::string &name, const tf::Transform &tf, double radius, double height)
				: Obstacle(name, tf, height)
			{
				//std::cout << "obstacle name: " << name ;
				radius_ = radius;
				std::cout << " radius_: " << radius_;
				distance_ = mtk::distance2D(tf.getOrigin()) - radius;
			}

			double radius() { return radius_; }
			bool intersects(double rx, double ry, double max_dist, double &distance)
			{
				double ix, iy;
				bool intersects =
					mtk::rayCircleIntersection(rx, ry, tf_.getOrigin().x(), tf_.getOrigin().y(), radius_, ix, iy, distance);
				if ((intersects == false) || (distance > max_dist))
					return false;
				else
					return true;
			}

		private:
			double radius_;
		};

		class Wall : public Obstacle
		{
		public:
			Wall(const std::string &name, const tf::Transform &tf, double length, double width, double height)
				: Obstacle(name, tf, height)
			{
				length_ = length;
				width_ = width;

				p1_.setX(-width_ / 2.0);
				p1_.setY(-length_ / 2.0);
				p1_.setZ(0.0);
				p2_.setX(+width_ / 2.0);
				p2_.setY(+length_ / 2.0);
				p2_.setZ(+height_);

				p1_ = tf_ * p1_;
				p2_ = tf_ * p2_;
				mtk:: Vector2D point = {0, 0};
				distance_ = mtk::pointSegmentDistance(0.0, 0.0, p1_.x(), p1_.y(), p2_.x(), p2_.y());
				
				
				
			}

			double length() { return length_; }
			double width() { return width_; }
		     bool intersects(double rx, double ry, double max_dist, double &distance)
			{
				double ix, iy;
				bool intersects =
					mtk::raySegmentIntersection(0.0, 0.0, rx, ry, p1_.x(), p1_.y(), p2_.x(), p2_.y(), ix, iy, distance);
				
			//	std::cout << 'kkkkkkkkkkkkkkkkk' << intersects << std::endl; 
				if ((intersects == false) || (distance > max_dist))
					return false;
				else
					return true;
			} 

		private:
			tf::Point p1_;
			tf::Point p2_;
	
			double length_;
			double width_;
		};

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		

		class Square : public Obstacle
			{
			public:
				Square(const std::string &name, const tf::Transform &tf, double length, double width, double height)
					: Obstacle(name, tf, height)
				{
					
					length_ = length;
					width_ = width;

					p1_.setX(0.0);
					p1_.setY(0.0);
					p1_.setZ(0.0);
					p2_.setX(width_ / 2.0);
					p2_.setY(0.0);
					p2_.setZ(0.0);
					p3_.setX(width_ / 2.0);
					p3_.setY(length_ / 2.0);
					p3_.setZ(0.0);
					p4_.setX(0.0);
					p4_.setY(length_ / 2.0);
					p4_.setZ(0.0);
				

					p1_ = tf_ * p1_;
					p2_ = tf_ * p2_;
					p3_ = tf_ * p3_;
					p4_ = tf_ * p4_;
					mtk:: Vector2D point = {0, 0};
					
					distance_ = mtk::shortestDistance(point, p1_.x(), p1_.y(), p2_.x(), p2_.y(),p3_.x(), p3_.y(), p4_.x(), p4_.y());
					
					//ROS_INFO("The value of x is: %d",distance_);
				}

				double length() { return length_; }
				double width() { return width_; }
				bool intersects(double rx, double ry, double max_dist, double &distance) 
				{
					double ix, iy;
					
					mtk::Ray ray ;
					mtk:: Vector2D start = {0, 0};
					mtk:: Vector2D end = {rx, ry}; 
					
					mtk::calculateRayDirection(ray, start, end);
					
					bool intersects =
					mtk::intersectRaySquare(ray, p1_.x(), p1_.y(), p2_.x(), p2_.y(),p3_.x(), p3_.y(), p4_.x(), p4_.y(),
											ix, iy, distance);
					
					
					if  ((intersects == false) || (distance > max_dist))
						return false;
					else
						return true;  
					
				} 
			
			private:
				tf::Point p1_;
				tf::Point p2_;
				tf::Point p3_;
				tf::Point p4_;
			
				double length_;
				double width_;

			
			}; 
			
 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		VirtualSensorNode();
		~VirtualSensorNode();

		bool init();
		void spin();

		// void columnPosesCB(const yocs_msgs::ColumnList::ConstPtr& msg);
		// void wallPosesCB(const yocs_msgs::WallList::ConstPtr& msg);
		void agentsPosesCB(const pedsim_msgs::AgentStates::ConstPtr &msg);
		void laserScanCB(const sensor_msgs::LaserScan::ConstPtr &msg);

	private:
		std::string global_frame_id_;
		ros::Publisher virtual_obs_pub_;
		ros::Subscriber laser_scan_sub_;
		ros::Subscriber agents_poses_sub_;
		ros::Subscriber wall_poses_sub_;
		ros::Subscriber column_poses_sub_;
		ros::Subscriber table_poses_sub_;
		sensor_msgs::LaserScan scan_;
		tf::TransformListener tf_listener_;
		std::vector<pedsim_msgs::AgentState> agents_;
		// std::vector<yocs_msgs::Column> columns_;
		// std::vector<yocs_msgs::Wall>   walls_;

		/**
		 * Add a new obstacle to obstacles list with some processing:
		 *  - remove those out of range
		 *  - short by increasing distance to the robot
		 * @param new_obs   New obstacle in robot reference system
		 * @param obstacles Current obstacles list
		 * @return True if added, false otherwise
		 */
		bool add(boost::shared_ptr<Obstacle> &new_obs, std::vector<boost::shared_ptr<Obstacle>> &obstacles);
	};

} /* namespace virtual_sensor */
#endif /* VIRTUAL_SENSOR_NODE_HPP_ */
