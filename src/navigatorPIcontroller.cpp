/* 
 * Copyright (c) 2015 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Author:
 * 		Javier Pérez Soler
 *      Juan Carlos García
 */ 

#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/ExperimentMeasurer/navigatorPIcontroller.h"

//DEBUG Flags
#define DEBUG_FLAG	0
#define SAT	5

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigatorPIcontroller");
	NavPiController navPiControl;
	ros::spin();
//	navPiControl.GoToPose();
 
}


NavPiController::NavPiController()
{
	safetyAlarm			= false;
	enableExecution		= false;
	targetPosition		= false;
	userControlRequest	= false;
	
	//Publishers initialization
	pub_odom = nh.advertise<nav_msgs::Odometry>("dataNavigator", 1);
	
	//Subscribers initialization
	sub_odomInfo = nh.subscribe<geometry_msgs::Pose>("g500/pose", 1, &NavPiController::odomCallback, this);
	sub_safetyInfo = nh.subscribe<std_msgs::Bool>("safetyMeasures", 1, &NavPiController::safetyMeasuresCallback,this);
	sub_userControlInfo = nh.subscribe<std_msgs::Bool>("userControlRequest", 1, &NavPiController::userControlReqCallback, this);

	//Services initialization
	runBlackboxGotoPoseSrv = nh.advertiseService("runBlackboxGotoPoseSrv", &NavPiController::enableRunBool, this);
	
}


NavPiController::~NavPiController()
{
}


bool NavPiController::enableRunBool(hrov_control::HrovControlStdMsg::Request &req, hrov_control::HrovControlStdMsg::Response &res)
{
	enableExecution = req.boolValue;
	robotDesiredPosition.pose.position.x = req.robotTargetPosition.position.x;
	robotDesiredPosition.pose.position.y = req.robotTargetPosition.position.y;
	robotDesiredPosition.pose.position.z = req.robotTargetPosition.position.z;
	targetPosition = false;
	initMissionTime = ros::Time::now();
	GoToPose();

	if ((!targetPosition) and (!enableExecution))
		res.boolValue = false; //Mission finished with error
	if ((targetPosition) and (enableExecution))
		res.boolValue = true; //Mission finished successfully

	return true;
}


void NavPiController::odomCallback(const geometry_msgs::Pose::ConstPtr& odomValue)
{
	double errorDist;
	
	//Updating the current robot position & orientation
	robotCurrentPose.position    = odomValue->position;
	robotCurrentPose.orientation = odomValue->orientation;

	//Storing the last robot-target position
	robotLastPose.position = robotTargetPose.position;

	//Getting the difference between target position & current pose.
	robotTargetPose.position.x = robotDesiredPosition.pose.position.x - robotCurrentPose.position.x;
	robotTargetPose.position.y = robotDesiredPosition.pose.position.y - robotCurrentPose.position.y;
	robotTargetPose.position.z = robotDesiredPosition.pose.position.z - robotCurrentPose.position.z;

	//Getting the difference between last & current pose.
	robotErrorPose.position.x = robotTargetPose.position.x - robotLastPose.position.x;
	robotErrorPose.position.y = robotTargetPose.position.y - robotLastPose.position.y;
	robotErrorPose.position.z = robotTargetPose.position.z - robotLastPose.position.z;

	lastRobotTargetDist = currentRobotTargetDist;

	//Checking if the robot has achieved the target position
	currentRobotTargetDist = sqrt( (double)(pow(robotTargetPose.position.x, 2)) \
		+ (double)(pow(robotTargetPose.position.y, 2)) + (double)(pow(robotTargetPose.position.z, 2)) );
	cout << "currentRobotTargetDist = " << currentRobotTargetDist << endl;
	if ((currentRobotTargetDist < 0.3) and (enableExecution))
		targetPosition = true;

	//Checking if the robot is stopped
	errorDist = lastRobotTargetDist - currentRobotTargetDist;
	cout << "errorDist = " << errorDist << endl;
	currentMissionTime = ros::Time::now();
	totalMissionTime = currentMissionTime - initMissionTime;
//	if ((abs(errorDist) < 0.0001) and (abs(errorDist) > 0.0001) and (enableExecution))
	if ((abs(errorDist) < 0.000001) and (enableExecution) and (totalMissionTime.toSec() > 2.0) )
		enableExecution = false;
	cout << "enableExecution = " << enableExecution << endl;
	cout << "targetPosition = " << targetPosition << endl;

	if (DEBUG_FLAG)
	{
		cout << "enableExecution = " << enableExecution <<  ". Robot problem" << endl;
		cout << "targetPosition = " << targetPosition << ". the robot has achieved the target position" << endl;
		//cout << "Position\n" << odomValue->position << "\nOrientation\n" << odomValue->orientation << endl;
		cout << "robotErrorPose  = " << robotErrorPose.position.x << ", " << \
				robotErrorPose.position.y << ", " << robotErrorPose.position.z << endl;
		cout << "robotTargetPose = " << robotTargetPose.position.x << ", " << \
				robotTargetPose.position.y << ", " << robotTargetPose.position.z << endl;
	}
}


void NavPiController::GoToPose()
{
	double Itermx = 0; double Itermy = 0; double Itermz = 0;
	double errorx = 0; double errory = 0; double errorz = 0;
	double gain = 0.5, Igain = 0;	

	ros::Rate loop_rate(50);
	while ((enableExecution) and (!targetPosition) and (!userControlRequest))
	{
		cout << "Inside if: robotTargetPose = " << robotTargetPose.position.x << ", " <<robotTargetPose.position.y << ", "\
									<< robotTargetPose.position.z << endl;

		//Compute control law
		errorx = gain * (robotTargetPose.position.x);
		errory = gain * (robotTargetPose.position.y);
		errorz = gain * (robotTargetPose.position.z);

		Itermx += robotTargetPose.position.x;
		Itermy += robotTargetPose.position.y;
		Itermz += robotTargetPose.position.z;

		//Send message to Simulator
		nav_msgs::Odometry msg;
		msg.twist.twist.linear.x  = errory + Igain * Itermy;
		msg.twist.twist.linear.y  = -errorx - Igain * Itermx;
		msg.twist.twist.linear.z  = errorz + Igain * Itermz;
		msg.twist.twist.angular.x = 0;
		msg.twist.twist.angular.y = 0;
		msg.twist.twist.angular.z = 0;
		if (msg.twist.twist.linear.x>SAT) msg.twist.twist.linear.x=SAT; else if (msg.twist.twist.linear.x<-SAT) msg.twist.twist.linear.x=-SAT;
		if (msg.twist.twist.linear.y>SAT) msg.twist.twist.linear.y=SAT; else if (msg.twist.twist.linear.y<-SAT) msg.twist.twist.linear.y=-SAT;
		if (msg.twist.twist.linear.z>SAT) msg.twist.twist.linear.z=SAT; else if (msg.twist.twist.linear.z<-SAT) msg.twist.twist.linear.z=-SAT;
		pub_odom.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	//Send 0 velocity to the thrustersAllocator rospackage
	nav_msgs::Odometry msg;
	msg.twist.twist.linear.x  = 0;
	msg.twist.twist.linear.y  = 0;
	msg.twist.twist.linear.z  = 0;
	msg.twist.twist.angular.x = 0;
	msg.twist.twist.angular.y = 0;
	msg.twist.twist.angular.z = 0;
	pub_odom.publish(msg);

	cout << "---------------- FINISH -------------------- " << endl;
	cout << msg.twist.twist << endl;
	
}



void NavPiController::userControlReqCallback(const std_msgs::Bool::ConstPtr& msg)
{
	userControlRequest = msg->data;
	if (DEBUG_FLAG)
		cout << "userControlRequestCallback: " << userControlRequest << endl;
}


void NavPiController::safetyMeasuresCallback(const std_msgs::Bool::ConstPtr& msg)
{
	safetyAlarm = msg->data;
	if (DEBUG_FLAG)
		cout << "safetyMeasuresCallback: " << safetyAlarm << endl;
}


