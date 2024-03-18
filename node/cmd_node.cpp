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

	ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target/mavros/local_position/pose", 10, target_pose_cb);
  	ros::Subscriber target_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/target/mavros/local_position/velocity_local", 10, target_vel_cb);
	
	std_msgs::Bool arm_cmd;
	std_msgs::Int32 mode_cmd;
	std_msgs::Int32 uav_init;
	geometry_msgs::PoseStamped leader_pose;
	geometry_msgs::TwistStamped leader_twist;
	bool tracking = false;
	bool vision_tracking = false;
	double move_step = 0.1;
	arm_cmd.data = false;
	mode_cmd.data = 0;

	ROS_INFO("\n(z):arm \n(x):disarm \n(1):takeoff \n(2):land \n(3):Offboard \n(4):Tracking/Stop Tracking \n(i):uav_init\n");
	
	leader_pose.pose.position.x = 0;
	leader_pose.pose.position.y = 0;
	leader_pose.pose.position.z = 5;
	while (ros::ok())
	{
		
		leader_twist.twist.linear.x = 0;
		leader_twist.twist.linear.y = 0;
		leader_twist.twist.linear.z = 0;
		//keyboard control
		int c = getch();
		if (c != EOF) 
		{
			switch (c) 
			{
				case 65:    // key up
                    leader_pose.pose.position.y += move_step;
					leader_twist.twist.linear.y = move_step*50;
                    break;
                case 66:    // key down
                    leader_pose.pose.position.y += -move_step;
					leader_twist.twist.linear.y = -move_step*50;
                    break;
                case 67:    // key CW(->)
                    leader_pose.pose.position.x += move_step;
					leader_twist.twist.linear.x = move_step*50;
                    break;
                case 68:    // key CCW(<-)
                    leader_pose.pose.position.x += -move_step;
					leader_twist.twist.linear.x = -move_step*50;
                    break;
				case 72: // (r)
					leader_pose.pose.position.x = 0;
					leader_pose.pose.position.y = 0;
					leader_pose.pose.position.z = 5;
					break;
				case 122:	// (z) 
					arm_cmd.data = true;
					arm_cmd_pub.publish(arm_cmd);
					ROS_INFO("Cmd arm");
					break;
				case 120:	// (x)
					arm_cmd.data = false;
					arm_cmd_pub.publish(arm_cmd);
					ROS_INFO("Cmd disarm");
					break;
				case 49:    // (1) takeoff
					mode_cmd.data = 1;
					mode_cmd_pub.publish(mode_cmd);
					ROS_INFO("Cmd takeoff");
					break;
				case 50:    // (2) land
					mode_cmd.data = 2;
					mode_cmd_pub.publish(mode_cmd);
					ROS_INFO("Cmd land");
					break;
				
				case 51:    // (3) offboard
					mode_cmd.data = 3;
					mode_cmd_pub.publish(mode_cmd);
					ROS_INFO("Cmd offboard");
					break;
				case 52:    // (4) tracking trigger
					if(!tracking)
					{
						tracking = true;
						ROS_INFO("Start tracking");
					}
					else
					{
						tracking = false;
						ROS_INFO("Stop tracking");
					}
					break;
				case 53:    // (5) tracking trigger
					if(!tracking)
						ROS_INFO("Not in tracking mode");
					else
					{
						mode_cmd.data = 5;
						mode_cmd_pub.publish(mode_cmd);
						ROS_INFO("Vision tracking switch");
					}
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

		if(tracking)
		{
			leader_pose = target_pose;
			leader_twist = target_velocity;
		}
		
		leader_pose_pub.publish(leader_pose);
		leader_vel_pub.publish(leader_twist);

		ros::spinOnce();
		rate.sleep();
	}


  return 0;
}
