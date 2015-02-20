#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>


#define POSETOPIC "g500/pose"
#define FORCETOPIC "g500/ForceSensor"

#define WAYPOINT1 (5,15,8)
#define WAYPOINT2 (15,30,10)
#define WAYPOINT3 (30,30,10)
#define WAYPOINT4 (28,10,3)
#define WAYPOINT5 (10,-5,10)

class Measurer{
  public:
    ros::Subscriber sub_pose, sub_force;
    double distance;
    double last_position[3];
    ros::Time last_collision, initial_time;
    int collisions;
    int current_waypoint;
    std::vector<tf::Vector3> waypoints;

    int started;
    
    Measurer();
    void callbackPose(const geometry_msgs::Pose& msg);
    void callbackForce(const geometry_msgs::WrenchStamped& msg);
};

void Measurer::callbackPose(const geometry_msgs::Pose& msg){

 if(started)
   distance+=sqrt(pow(last_position[0]-msg.position.x,2)+
                pow(last_position[1]-msg.position.y,2)+
                pow(last_position[2]-msg.position.z,2));
 else
   started=1;
 last_position[0]=msg.position.x;
 last_position[1]=msg.position.y;
 last_position[2]=msg.position.z;

 if(current_waypoint< waypoints.size() and waypoints[current_waypoint].distance(tf::Vector3(msg.position.x,msg.position.y,msg.position.z))<2.0){
   std::cout<<"WAYPOINT PASADO"<<std::endl;
   std::cout<<"Distance: "<<distance<<" Ncollisions: "<<collisions<<" Time: "<<(ros::Time::now()-initial_time).toSec()<<std::endl;
   current_waypoint++;
 }

//std::cout<<distance<<"  "<<msg.position.x<<std::endl;
}

void Measurer::callbackForce(const geometry_msgs::WrenchStamped& msg){


  if ((ros::Time::now()-last_collision).toSec()>=0.5){
    if(msg.wrench.force.x !=0  or msg.wrench.force.y != 0 or msg.wrench.force.z != 0 or
        msg.wrench.torque.x != 0 or msg.wrench.torque.y != 0 or msg.wrench.torque.z != 0){
      collisions++;
      last_collision=ros::Time::now();
    }
  }

  //std::cout<<collisions<<"  "<<msg.wrench.force.x<<"  "<<msg.wrench.force.y<<"  "<<msg.wrench.force.z<<"  "<<msg.wrench.torque.x<<"  "<<msg.wrench.torque.y<<"  "<<msg.wrench.torque.z<<std::endl;
}

Measurer::Measurer(){
  ros::NodeHandle nh;
  started=0;
  distance=0;
  last_collision=ros::Time::now();
  initial_time=ros::Time::now();
  collisions=0;

  sub_pose = nh.subscribe(POSETOPIC, 1000, &Measurer::callbackPose,this);
  sub_force = nh.subscribe(FORCETOPIC, 1000, &Measurer::callbackForce,this);

  waypoints.push_back(tf::Vector3 WAYPOINT1);
  waypoints.push_back(tf::Vector3 WAYPOINT2);
  waypoints.push_back(tf::Vector3 WAYPOINT3);
  waypoints.push_back(tf::Vector3 WAYPOINT4);
  waypoints.push_back(tf::Vector3 WAYPOINT5);
  current_waypoint=0;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "measurer");
  Measurer measurer;
  ros::spin();
}
