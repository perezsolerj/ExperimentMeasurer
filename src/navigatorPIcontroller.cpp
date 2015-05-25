#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "../include/ExperimentMeasurer/navigatorPIcontroller.h"


using namespace std;


void enableRunBool(hrov_control::HrovControlStdMsg::Request &req, hrov_control::HrovControlStdMsg::Response &res)
{
//	navPiControlInfo.enableExecution = req.boolValue;
//	res.boolValue = safetyInfo.safetyAlarm;
}

NavPiController::NavPiController()
{
	safetyAlarm = false;
	
	sub_pose = nh.subscribe("gotopose", 1000, &PoseCallback::callback,&pose);
	
	sub_safetyInfo = nh.subscribe<std_msgs::Bool>("safetyMeasures", 1, &NavPiController::safetyMeasuresCallback,this);
	pub = nh.advertise<nav_msgs::Odometry>("dataNavigator", 1);

	//Services initialization
//	runBlackboxGotoPoseSrv = nh.advertiseService<hrov_control::HrovControlStdMsg>("runBlackboxGotoPoseSrv", enableRunBool);

}

NavPiController::~NavPiController()
{
}


void NavPiController::safetyMeasuresCallback(const std_msgs::Bool::ConstPtr& msg)
{
      safetyAlarm = msg->data;
//      cout << "Inside callback safetyAlarm: " << safetyAlarm << endl;
}

void NavPiController::GoToPose()
{
	double Itermx = 0;
	double Itermy = 0;
	double Itermz = 0;
	double gain,Igain;	

	ros::Rate loop_rate(50);
	while ( ros::ok() )
	{
//		cout << "Inside while safetyAlarm: " << safetyAlarm << endl;
//		cout << "enableExecution: " << enableExecution << endl;
		if (!safetyAlarm)
		{
			cout << "safetyAlarm: " << safetyAlarm << ", the robot is working..." << endl;

			//Compute control law
			double errorx=gain*(pose.pos[0]);
			double errory=gain*(pose.pos[1]);
			double errorz=gain*(pose.pos[2]);

			Itermx+=pose.pos[0];
			Itermy+=pose.pos[1];
			Itermz+=pose.pos[2];

			//Send message to Simulator
			nav_msgs::Odometry msg;

			msg.twist.twist.linear.x=errorx+Igain*Itermx;
			msg.twist.twist.linear.y=errory+Igain*Itermy;
			msg.twist.twist.linear.z=errorz+Igain*Itermz;
			msg.twist.twist.angular.x=0;
			msg.twist.twist.angular.y=0;
			msg.twist.twist.angular.z=0;

			if (msg.twist.twist.linear.x>SAT) msg.twist.twist.linear.x=SAT; else if (msg.twist.twist.linear.x<-SAT) msg.twist.twist.linear.x=-SAT;
			if (msg.twist.twist.linear.y>SAT) msg.twist.twist.linear.y=SAT; else if (msg.twist.twist.linear.y<-SAT) msg.twist.twist.linear.y=-SAT;
			if (msg.twist.twist.linear.z>SAT) msg.twist.twist.linear.z=SAT; else if (msg.twist.twist.linear.z<-SAT) msg.twist.twist.linear.z=-SAT;

			pub.publish(msg);
		}
		else
			std::cout << "safetyAlarm: " << safetyAlarm << ", the robot is stopped..." << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "navigatorPIcontroller");
	NavPiController navPiControl;
	navPiControl.GoToPose();
 
}

