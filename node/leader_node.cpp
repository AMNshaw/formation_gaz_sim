#include "ros/ros.h"
#include "std_msgs/String.h"
#include "getch.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <math.h>

#define LEADER_INIT_X 0.0f
#define LEADER_INIT_Y 0.0f
#define TAKEOFF_HEIGHT 3.0f
#define LAND_SPEED 0.8f
#define CONTROL_HZ 100.0f

float trajectory_t; 

geometry_msgs::PoseStamped leader_pose;
geometry_msgs::TwistStamped leader_vel;
geometry_msgs::PoseStamped target_pose;
geometry_msgs::TwistStamped target_vel;
geometry_msgs::PoseStamped desired_pose;
std_msgs::Int32 init_drone;


int track_count = 0;
float sum_x = 0;
float sum_y = 0;

enum {
	DISARM,
	HOVERING,
    TAKEOFF,
    LAND,
    ATTACK,
}LeaderMode;

int leader_mode;
int kill_all_drone = 0;
int start_all_drone = 0;
int takeoff_all_drone = 0;
int init_all_drone = 0;

void start_takeoff(){
	if(leader_mode == TAKEOFF || leader_pose.pose.position.z>0.1 ){
		ROS_WARN("leader already takeoff");
	}
	else{
		leader_mode = TAKEOFF;
		ROS_INFO("leader start takeoff");
	}
}

void start_land(){
	if(leader_mode == LAND || leader_pose.pose.position.z <= 0.01 ){
		ROS_WARN("leader already landing or it's on the land");
	}
	else{
		leader_mode = LAND;
		ROS_INFO("leader start landing");
	}
}

void start_attack(){
	if(leader_mode == HOVERING){
    	leader_mode = ATTACK;
		ROS_INFO("leader start surrounding");
		leader_pose.pose.position.x = target_pose.pose.position.x;
        leader_pose.pose.position.y = target_pose.pose.position.y;
	}
	else{
		ROS_WARN("leader can not start surrounding");
	}
}

void leader_stop(){
	if(leader_mode != HOVERING){
    	leader_mode = HOVERING;
		ROS_INFO("leader stop");
	}
	else{
		ROS_WARN("leader already hovering");
	}
}

void all_drone_pose_init(){
	if(leader_mode == LAND || leader_mode == DISARM){
		ROS_INFO("Initializing all drones...");
		init_all_drone = 1;
	}
	else{
		ROS_WARN("Drones not on land");
	}
}

void velocity_ctrl(geometry_msgs::PoseStamped pose_d, geometry_msgs::TwistStamped *self_vel)
{
	float kp = 1.5;
	float err_x = pose_d.pose.position.x - leader_pose.pose.position.x;
	float err_y = pose_d.pose.position.y - leader_pose.pose.position.y;
	float err_z = pose_d.pose.position.z - leader_pose.pose.position.z;
	self_vel->twist.linear.x = kp*err_x;
	self_vel->twist.linear.y = kp*err_y;
	self_vel->twist.linear.z = kp*err_z;
}

void velocity_ctrl(geometry_msgs::PoseStamped pose_d, geometry_msgs::TwistStamped vel_d, geometry_msgs::TwistStamped *self_vel)
{
	float kp = 1.5;
	float err_x = pose_d.pose.position.x - leader_pose.pose.position.x;
	float err_y = pose_d.pose.position.y - leader_pose.pose.position.y;
	float err_z = pose_d.pose.position.z - leader_pose.pose.position.z;
	self_vel->twist.linear.x = kp*err_x + vel_d.twist.linear.x;
	self_vel->twist.linear.y = kp*err_y + vel_d.twist.linear.y;
	self_vel->twist.linear.z = 0;
	if(pose_d.pose.position.z >= 0.5)
		self_vel->twist.linear.z = kp*err_z + vel_d.twist.linear.z;
}

void leader_pose_generate(geometry_msgs::PoseStamped *leader_pose)
{
	float dt = 1/CONTROL_HZ;

	if(leader_mode == TAKEOFF)
	{
		desired_pose.pose.position.x = 0;
		desired_pose.pose.position.y = 0;
		desired_pose.pose.position.z = TAKEOFF_HEIGHT;
		velocity_ctrl(desired_pose, &leader_vel);
		float vel_norm = sqrt(pow(leader_vel.twist.linear.x, 2) + pow(leader_vel.twist.linear.y, 2) + pow(leader_vel.twist.linear.z, 2));
		if(vel_norm < 0.05 && leader_pose->pose.position.z > 0.5)
			leader_mode = HOVERING;
	}
	if(leader_mode == LAND)
	{
		desired_pose.pose.position.z = 0;
		velocity_ctrl(desired_pose, &leader_vel);
		leader_vel.twist.linear.x = 0;
		leader_vel.twist.linear.y = 0;
	}
	if(leader_mode == HOVERING)
	{
		leader_vel.twist.linear.x = 0;
		leader_vel.twist.linear.y = 0;
		leader_vel.twist.linear.z = 0;
	}
	if(leader_mode == ATTACK)
	{
		velocity_ctrl(target_pose, target_vel, &leader_vel);
	}

	leader_pose->header.stamp = ros::Time::now();
	leader_pose->pose.position.x = leader_pose->pose.position.x + leader_vel.twist.linear.x*dt;
	leader_pose->pose.position.y = leader_pose->pose.position.y + leader_vel.twist.linear.y*dt;
	leader_pose->pose.position.z = leader_pose->pose.position.z + leader_vel.twist.linear.z*dt;

}

void target_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	target_pose = *msg;
}
void target_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	target_vel = *msg;
}

int main(int argc, char **argv)
{
  leader_mode = DISARM;
  ros::init(argc, argv, "leader_pose_publisher");

  ros::NodeHandle nh;

  ros::Publisher leader_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/leader_pose", 10);
  ros::Publisher leader_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/leader_vel", 10);
  ros::Publisher uav_killer_pub = nh.advertise<std_msgs::Int32>("/uav_kill", 10);
  ros::Publisher uav_start_pub = nh.advertise<std_msgs::Int32>("/uav_start", 10);
  ros::Publisher uav_init_pub = nh.advertise<std_msgs::Int32>("/uav_init", 10);
  ros::Publisher uav_takeoff_pub = nh.advertise<std_msgs::Int32>("/uav_takeoff", 10);


  ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/target/mavros/local_position/pose_initialized", 10, target_pose_cb);
  ros::Subscriber target_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/target/mavros/local_position/velocity_local", 10, target_vel_cb);

  ros::Rate loop_rate(CONTROL_HZ);
  
  leader_pose.header.stamp = ros::Time::now();
  leader_pose.header.frame_id = "map";
  leader_pose.pose.position.x = LEADER_INIT_X;
  leader_pose.pose.position.y = LEADER_INIT_Y;
  leader_pose.pose.position.z = 0.0;
  leader_pose.pose.orientation.x = 0.0;
  leader_pose.pose.orientation.y = 0.0;
  leader_pose.pose.orientation.z = 0.0;
  leader_pose.pose.orientation.w = 1.0;
  ROS_INFO("(t):takeoff\n (l):land\n (a):attack\n (p):stop MAV\n (k):kill_all_drone\n (s):start_all_drone\n (i):uav_init\n");
  while (ros::ok())
  {
        //keyboard control
  		init_all_drone = 0;
  		kill_all_drone = 0;
  		start_all_drone = 0;
  		takeoff_all_drone = 0;
        int c = getch();
        //ROS_INFO("C: %d",c);
        if (c != EOF) {
            switch (c) {
                case 116:    // (t) takeoff
                    start_takeoff();
                    ROS_INFO("takeoff all drone");
                    takeoff_all_drone = 1;
                    break;
                case 108:    // (l) land
                    start_land();
                    break;
                case 97:    // (a) land
                    start_attack();
                    break;
                case 112:    // (p) stop
                	leader_stop();
                	break;
                case 107:    // (k) uav_kill
					kill_all_drone = 1;
					ROS_WARN("kill all drone");
                    break;
                case 105:	// (i) uav_init
                	all_drone_pose_init();
                	init_drone.data = init_all_drone;
                	uav_init_pub.publish(init_drone);
                	break;
                case 115:
                	start_all_drone = 1;
					ROS_INFO("start all drone");
					break;
			}
        }
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
	leader_pose_generate(&leader_pose);
	
	std_msgs::Int32 kill_msg;
	kill_msg.data=kill_all_drone;
	uav_killer_pub.publish(kill_msg);
	std_msgs::Int32 start_msg;
	start_msg.data=start_all_drone;
	std_msgs::Int32 takeoff_msg;
	takeoff_msg.data = takeoff_all_drone;
	uav_start_pub.publish(start_msg);
    leader_pose_pub.publish(leader_pose);
    leader_vel_pub.publish(leader_vel);
    uav_takeoff_pub.publish(takeoff_msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
