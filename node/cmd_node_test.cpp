#include "ros/ros.h"
#include "std_msgs/String.h"
#include "getch.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <math.h>

geometry_msgs::PoseStamped target_pose;
geometry_msgs::TwistStamped target_velocity;

void target_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	target_pose = *msg;
}
void target_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	target_velocity = *msg;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "formation");
    ros::NodeHandle nh;
	ros::Rate rate(50);

	ros::Publisher arm_cmd_pub = nh.advertise<std_msgs::Bool>("/formation/all_uav_arm", 5);
	ros::Publisher mode_cmd_pub = nh.advertise<std_msgs::Int32>("/formation/all_uav_mode", 5);
	ros::Publisher uav_init_pub = nh.advertise<std_msgs::Int32>("/uav_init", 10);
	ros::Publisher leader_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/leader/formation/pose", 10);
  	ros::Publisher leader_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/leader/formation/velocity", 10);

	ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target/formation/pose", 10, target_pose_cb);
  	ros::Subscriber target_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/target/mavros/local_position/velocity_local", 10, target_vel_cb);
	
	std_msgs::Bool arm_cmd;
	std_msgs::Int32 mode_cmd;
	std_msgs::Int32 uav_init;
	geometry_msgs::PoseStamped leader_pose;
	geometry_msgs::TwistStamped leader_twist;
	arm_cmd.data = false;
	mode_cmd.data = 0;

	ROS_INFO("\n(t):takeoff\n (l):land \n (a):arm/disarm \n (p):stop MAV \n (k):kill_all_drone \n (s):start_all_drone \n (i):uav_init\n");
	
	
	while (ros::ok())
	{

		//keyboard control
		int c = getch();
		if (c != EOF) 
		{
			switch (c) 
			{
				case 97:	// (a) 
					arm_cmd.data = true;
					arm_cmd_pub.publish(arm_cmd);
					break;
				case 100:	// (d)
					arm_cmd.data = false;
					arm_cmd_pub.publish(arm_cmd);
					break;
				case 116:    // (t) takeoff
					mode_cmd.data = 1;
					mode_cmd_pub.publish(mode_cmd);
					break;
				case 108:    // (l) land
					mode_cmd.data = 2;
					mode_cmd_pub.publish(mode_cmd);
					break;
				
				case 111:    // (o) uav_kill
					mode_cmd.data = 3;
					mode_cmd_pub.publish(mode_cmd);
					break;
				case 112:    // (p) stop

					break;
				
				case 105:	// (i) uav_init
					if(!arm_cmd.data)
					{
						uav_init.data = 1;
						uav_init_pub.publish(uav_init);
						ROS_INFO("All uav pose init");
					}
					else
						ROS_INFO("Can't init, all UAV arming");
					
					break;
				case 115:
					break;
			}
		}

		leader_pose = target_pose;
		leader_twist = target_velocity;
		leader_pose_pub.publish(leader_pose);
		leader_vel_pub.publish(leader_twist);

		ros::spinOnce();
		rate.sleep();
	}


  return 0;
}
