#include <iostream>
#include <stdlib.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"

#define SAT	5

//Camera info callback to get size of the camera
class PoseCallback{
  public:
    double pos[3];
    double quat[4];
 
	//Indicates the robot desired position
    void callback(const geometry_msgs::PoseStamped& msg) {
      pos[0]=msg.pose.position.x;
      pos[1]=msg.pose.position.y;
      pos[2]=msg.pose.position.z;

      quat[0]=msg.pose.orientation.x;
      quat[1]=msg.pose.orientation.y;
      quat[2]=msg.pose.orientation.z;
      quat[3]=msg.pose.orientation.w;
    }
};


class BenchmarkInfoCallback{
  public:
    int newIteration;

    BenchmarkInfoCallback(){
      newIteration=0;
    }

    void callback(const std_msgs::String& msg) {
      if(msg.data!="")
	newIteration=1;
    }
};


class SafetyMeasuresCallback{
  public:
    bool safetyAlarm;

    SafetyMeasuresCallback(){
      safetyAlarm = false;
    }

    void callback(const std_msgs::Bool & msg) {
      safetyAlarm = msg.data;
    }
};


class NavPiController {
	
};


int main(int argc, char **argv){
  std::string pose_topic, velocity_topic, benchinfo_topic, safetyinfo_topic;
  PoseCallback pose;
  BenchmarkInfoCallback benchInfo;
  SafetyMeasuresCallback safetyInfo;
  double gain,Igain;

  ros::init(argc, argv, "navigatorPIcontroller");
  ros::NodeHandle nh;
 
  nh.param("pose", pose_topic, (std::string)"gotopose");
  nh.param("velocity", velocity_topic, (std::string)"dataNavigator");
  nh.param("benchinfo", benchinfo_topic, (std::string)"BenchmarkInfo");
  nh.param("safetyinfo", safetyinfo_topic, (std::string)"safetyMeasures");
  nh.param("gain", gain, 0.5);
  nh.param("Igain", Igain, 0.001);

  ros::Subscriber sub_pose = nh.subscribe(pose_topic, 1000, &PoseCallback::callback,&pose);
  ros::Subscriber sub_benchInfo = nh.subscribe(benchinfo_topic, 1000, &BenchmarkInfoCallback::callback,&benchInfo);
  ros::Subscriber sub_safetyInfo = nh.subscribe(safetyinfo_topic, 1000, &SafetyMeasuresCallback::callback,&safetyInfo);
  ros::Publisher pub=nh.advertise<nav_msgs::Odometry>(velocity_topic, 1);

  double Itermx=0;
  double Itermy=0;
  double Itermz=0;

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
	  if (!safetyInfo.safetyAlarm)
	  {
		std::cout << "safetyAlarm: " << safetyInfo.safetyAlarm << std::endl;

		//Check if newIteration started to restart Iterms
		if(benchInfo.newIteration){
		  Itermx=0;
		  Itermy=0;
		  Itermz=0;
		  benchInfo.newIteration=0;
		}
		
		//Compute control law
		double errorx=gain*(pose.pos[0]);
		double errory=gain*(pose.pos[1]);
		double errorz=gain*(pose.pos[2]);

		Itermx+=pose.pos[0];
		Itermy+=pose.pos[1];
		Itermz+=pose.pos[2];

		//Send message to Simulator
		//geometry_msgs::TwistStamped msg;
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
		std::cout << "safetyAlarm: " << safetyInfo.safetyAlarm << std::endl;

    ros::spinOnce();
    loop_rate.sleep();
  }

}

