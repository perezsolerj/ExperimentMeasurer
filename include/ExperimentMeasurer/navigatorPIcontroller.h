/* 
 * Copyright (c) 2015 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Author:
 *     Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <hrov_control/HrovControlStdMsg.h>


using namespace std;


#define SAT	5

//Camera info callback to get size of the camera
class PoseCallback{
	public:
		double pos[3];
		double quat[4];

		//Indicates the robot desired position
		void callback(const geometry_msgs::PoseStamped& msg)
		{
			pos[0] = msg.pose.position.x;
			pos[1] = msg.pose.position.y;
			pos[2] = msg.pose.position.z;

			quat[0] = msg.pose.orientation.x;
			quat[1] = msg.pose.orientation.y;
			quat[2] = msg.pose.orientation.z;
			quat[3] = msg.pose.orientation.w;
		}
};



class NavPiController
{

	public:
		NavPiController();
		~NavPiController();
		
		bool 			enableExecution;
		bool			safetyAlarm;
		PoseCallback	pose;
		
		void GoToPose();
		
	private:
		ros::NodeHandle nh;

		ros::Publisher		pub;
		ros::Subscriber		sub_pose;
		ros::Subscriber		sub_benchInfo;
		ros::Subscriber		sub_safetyInfo;
		ros::ServiceServer	runBlackboxGotoPoseSrv;

		void safetyMeasuresCallback(const std_msgs::Bool::ConstPtr& msg);
		bool enableRunBool(hrov_control::HrovControlStdMsg::Request &req, hrov_control::HrovControlStdMsg::Response &res);

};
