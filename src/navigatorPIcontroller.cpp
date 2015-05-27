#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../include/ExperimentMeasurer/navigatorPIcontroller.h"

//DEBUG Flags
#define DEBUG_FLAG	1
#define SAT	5

using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigatorPIcontroller");
	NavPiController navPiControl;
//	ros::spin();
	navPiControl.GoToPose();
 
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
	//GoToPose();
	
	//Send 0 velocity to the thrustersAllocator rospackage
	nav_msgs::Odometry msg;
	msg.twist.twist.linear.x  = 0;
	msg.twist.twist.linear.y  = 0;
	msg.twist.twist.linear.z  = 0;
	msg.twist.twist.angular.x = 0;
	msg.twist.twist.angular.y = 0;
	msg.twist.twist.angular.z = 0;
	pub_odom.publish(msg);

	if ((!targetPosition) and (!enableExecution))
		res.boolValue = false; //Mission finished with error
	if ((targetPosition) and (!enableExecution))
		res.boolValue = true; //Mission finished successfully

	return true;
}


void NavPiController::odomCallback(const geometry_msgs::Pose::ConstPtr& odomValue)
{
	double dist;
	
	//Storing the last robot position
	robotLastPose.position = robotCurrentPose.position;
	//Updating the current robot position & orientation
	robotCurrentPose.position    = odomValue->position;
	robotCurrentPose.orientation = odomValue->orientation;
	
	//Getting the difference between last & current pose.
	robotErrorPose.position.x = robotCurrentPose.position.x - robotLastPose.position.x;
	robotErrorPose.position.y = robotCurrentPose.position.y - robotLastPose.position.y;
	robotErrorPose.position.z = robotCurrentPose.position.z - robotLastPose.position.z;

	//Checking if the difference is less than 0.01 (in each component), so the robot has a problem and it is not able to move
	//sqrt(a²+b²+c²) < 0.1 -> 0.173205081
	dist = sqrt( (double)(pow(robotErrorPose.position.x, 2)) \
		+ (double)(pow(robotErrorPose.position.y, 2)) + (double)(pow(robotErrorPose.position.z, 2)) );
	if (dist < 0.17)
		enableExecution = false;

	//Getting the difference between target position & current pose.
	robotTargetPose.position.x = robotDesiredPosition.pose.position.x - robotCurrentPose.position.x;
	robotTargetPose.position.y = robotDesiredPosition.pose.position.y - robotCurrentPose.position.y;
	robotTargetPose.position.z = robotDesiredPosition.pose.position.z - robotCurrentPose.position.z;

	//Checking if the robot has achieved the target position
	dist = sqrt( (double)(pow(robotTargetPose.position.x, 2)) \
		+ (double)(pow(robotTargetPose.position.y, 2)) + (double)(pow(robotTargetPose.position.z, 2)) );
	if (dist < 0.17)
		targetPosition = true;

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


void NavPiController::GoToPose()
{
	double Itermx = 0; double Itermy = 0; double Itermz = 0;
	double errorx = 0; double errory = 0; double errorz = 0;
	double gain = 0.5, Igain = 0;	

	ros::Rate loop_rate(50);
	while ( ros::ok() )
	{
		if ((enableExecution) and (!targetPosition) and (!userControlRequest))
		{
			cout << "robotCurrentPose = " << robotCurrentPose.position.x << ", " <<robotCurrentPose.position.y << ", "\
										<< robotCurrentPose.position.z << endl;
			cout << "robotDesiredPosition = " << robotDesiredPosition.pose.position.x << ", " << robotDesiredPosition.pose.position.y << ", "\
										<< robotDesiredPosition.pose.position.z << endl; 

			//Getting the difference between last & current pose.
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
		}
/*		else
		{
			cout << "\nsafetyAlarm: " << safetyAlarm << ", the robot is stopped..." << endl;
			cout << "enableExecution: " << enableExecution << endl;
			cout << "targetPosition: " << targetPosition << "\n" << endl;
			
		}*/
		ros::spinOnce();
		loop_rate.sleep();
	}

	
}


