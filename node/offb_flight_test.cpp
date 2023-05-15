#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <queue>
#define gravity 9.806

using namespace std;

bool desired_input_init = false;
bool pose_init = false;

double KPx=1, KPy=1, KPz=1.2;
double KPyaw = 1;

double roll = 0, pitch = 0, yaw = 0;

geometry_msgs::PoseStamped desired_pose;
double desired_yaw = 0;
int kill_all_drone = 0;
int start_all_drone = 0;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped host_mocap;
geometry_msgs::PoseStamped initial_pose;

geometry_msgs::TwistStamped desired_vel_raw;
geometry_msgs::TwistStamped desired_vel; 

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void host_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    if(pose_init == false){
        pose_init = true;
    }
    host_mocap = *msg;

    //transfer quartenion to roll, pitch, yaw
    tf::Quaternion Q(
        host_mocap.pose.orientation.x,
        host_mocap.pose.orientation.y,
        host_mocap.pose.orientation.z,
        host_mocap.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}

void desired_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    //store odometry into global variable
    desired_pose = *msg;
    
    if(desired_input_init == false){
        desired_input_init = true;
    }
    tf::Quaternion Q(
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z,
    msg->pose.orientation.w);
    desired_yaw = tf::getYaw(Q);
}

void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    //store odometry into global variable
    desired_vel_raw = *msg;
    
    if(desired_input_init == false){
        desired_input_init = true;
    }
}

void start_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    start_all_drone = msg.data;
}
void kill_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    kill_all_drone = msg.data;
}

int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "velocity_cbf");
    ros::NodeHandle nh,private_nh("~");

    string use_input_s;
    if(private_nh.getParam("use_input", use_input_s) == false) {
       ROS_FATAL("No use_input is assigned.");
       //exit(0);
       use_input_s = "position";
    }   
    std::cout<< use_input_s << "\n";
    //    subscriber    //
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, host_pose_cb);

    ros::Subscriber desired_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("desired_pose", 10, desired_pose_cb);
    ros::Subscriber desired_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("desired_velocity_raw", 10, desired_vel_cb);
    
    ros::Subscriber uav_start_sub = nh.subscribe<std_msgs::Int32>("/uav_start", 10, start_cb);
    ros::Subscriber uav_killer_sub = nh.subscribe<std_msgs::Int32>("/uav_kill", 10, kill_cb);
    // publisher
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 2);
    // service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(100);


    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!pose_init)) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose and desired input init ,%d",pose_init);
    }
    ROS_INFO("pose initialized");
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");

    ROS_INFO("Wait for UAV all start signal");
    while (ros::ok()) {
        if(start_all_drone == 1){
            break;
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for UAV all start signal");
    }
    ROS_INFO("get UAV all start signal");

    
    //send a few velocity setpoints before starting
    for(int i = 0; ros::ok() && i < 20; i++){
        local_vel_pub.publish(desired_vel);
        rate.sleep();
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    
        desired_vel.twist.linear.z = 0.2;

        local_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}