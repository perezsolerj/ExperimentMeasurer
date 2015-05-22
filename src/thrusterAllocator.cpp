//#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
#include "underwater_sensor_msgs/DVL.h"
#define SAT	5

//Twist callback to get velocity reference
class TwistCallback{
  public:
    double linear[3];
    double angular[3];

    TwistCallback(){
	linear[0]=linear[1]=linear[2]=0;
	angular[0]=angular[1]=angular[2]=0;

    }
 
    void callback(const nav_msgs::Odometry& msg) {
      linear[0]=msg.twist.twist.linear.x;
      linear[1]=msg.twist.twist.linear.y;
      linear[2]=msg.twist.twist.linear.z;

      angular[0]=msg.twist.twist.angular.x;
      angular[1]=msg.twist.twist.angular.y;
      angular[2]=msg.twist.twist.angular.z;

      //std::cout<<linear[0]<<" "<<linear[1]<<" "<<linear[2]<<std::endl;
    }
};

//DVL callback to get vehicle's velocity
class DVLCallback{
  public:
    double linear[3];

    DVLCallback(){
	linear[0]=linear[1]=linear[2]=0;
    }
 
    void callback(const underwater_sensor_msgs::DVL& msg) {
      linear[0]=msg.bi_x_axis;
      linear[1]=msg.bi_y_axis;
      linear[2]=msg.bi_z_axis;

      //std::cout<<linear[0]<<" "<<linear[1]<<" "<<linear[2]<<std::endl;
    }
};

int main(int argc, char **argv){
  std::string twist_topic, thrusters_topic, dvl_topic;
  TwistCallback twist;
  DVLCallback dvl;

  ros::init(argc, argv, "vehicleThrusterAllocator");
  ros::NodeHandle nh;
 
  nh.param("twist", twist_topic, (std::string)"/dataNavigator");
  nh.param("thrusters", thrusters_topic, (std::string)"/g500/thrusters_input");
  nh.param("dvl", dvl_topic, (std::string)"/g500/dvl");

  ros::Subscriber sub_twist = nh.subscribe(twist_topic, 1000, &TwistCallback::callback,&twist);
  ros::Subscriber sub_dvl = nh.subscribe(dvl_topic, 1000, &DVLCallback::callback,&dvl);
  ros::Publisher pub=nh.advertise<std_msgs::Float64MultiArray>(thrusters_topic, 1);

  ros::Rate loop_rate(200);

  double thrust_req[5];
  double last_thrust_req[5];
  double current_thrust_req[5];
  memset(last_thrust_req, 0, sizeof(last_thrust_req)); //clear array

  sleep(10);
  while(ros::ok())
  {
    memset(thrust_req, 0, sizeof(thrust_req)); //clear array
    //std::cout<<twist.linear[0]-dvl.linear[0]<<" "<<twist.linear[1]-dvl.linear[1]<<" "<<twist.linear[2]-dvl.linear[2]<<std::endl;
    //std::cout<<"DESIRED: "<<twist.linear[2]<<" REAL:"<<dvl.linear[2]<<" APLICADA: "<<twist.linear[2]-dvl.linear[2]<<std::endl;


/*	if (twist.angular[2] != 0)
	{
		thrust_req[0] = -twist.angular[2];
	    thrust_req[1] = twist.angular[2];
	    thrust_req[2] = 0;
	    thrust_req[3] = 0;
		thrust_req[4] = 0;
	}
	else
	{*/
//    thrust_req[0]=(-twist.linear[0]-dvl.linear[0])*5 - twist.angular[2];
//    thrust_req[1]=(-twist.linear[0]-dvl.linear[0])*5 + twist.angular[2];
/*		thrust_req[0]=(-twist.linear[0]-dvl.linear[0])*5;
		thrust_req[1]=(-twist.linear[0]-dvl.linear[0])*5;
		thrust_req[2]=-(twist.linear[2]-dvl.linear[2])*5;
		thrust_req[3]=-(twist.linear[2]-dvl.linear[2])*5;
		thrust_req[4]=(twist.linear[1]+dvl.linear[1])*6; */

/*		thrust_req[0] = -twist.linear[0] - twist.angular[2];
		thrust_req[1] = -twist.linear[0] + twist.angular[2];
		thrust_req[2] = -twist.linear[2];
		thrust_req[3] = -twist.linear[2];
		thrust_req[4] =  twist.linear[1];*/

		current_thrust_req[0] = -twist.linear[0] - twist.angular[2];
		current_thrust_req[1] = -twist.linear[0] + twist.angular[2];
		current_thrust_req[2] = -twist.linear[2];
		current_thrust_req[3] = -twist.linear[2];
		current_thrust_req[4] =  twist.linear[1];
		
		thrust_req[0] = (current_thrust_req[0] + last_thrust_req[0]) / 2;
		thrust_req[1] = (current_thrust_req[1] + last_thrust_req[1]) / 2;
		thrust_req[2] = (current_thrust_req[2] + last_thrust_req[2]) / 2;
		thrust_req[3] = (current_thrust_req[3] + last_thrust_req[3]) / 2;
		thrust_req[4] = (current_thrust_req[4] + last_thrust_req[4]) / 2;

		last_thrust_req[0] = current_thrust_req[0];
		last_thrust_req[1] = current_thrust_req[1];
		last_thrust_req[2] = current_thrust_req[2];
		last_thrust_req[3] = current_thrust_req[3];
		last_thrust_req[4] = current_thrust_req[4];

//	}
    std::cout<<thrust_req[0]<<" "<<thrust_req[1]<<" "<<thrust_req[2]<<" "<<thrust_req[3]<<" "<<thrust_req[4]<<std::endl;

    //Send message to Simulator
    std_msgs::Float64MultiArray msg;

    msg.data.push_back(thrust_req[0]);
    msg.data.push_back(thrust_req[1]);
    msg.data.push_back(thrust_req[2]);
    msg.data.push_back(thrust_req[3]);
    msg.data.push_back(thrust_req[4]);
   
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

}

