#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <queue>


class Mav
{
private:
    bool pose_init;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;

    ros::Subscriber current_pose_sub;
    ros::Subscriber mav_state_sub;

public:
    Mav();
    Mav(ros::NodeHandle &nh);
    void mav_state_cb(const mavros_msgs::State::ConstPtr& msg);
    void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped getCurrentPose();
    mavros_msgs::State getCurrentState();
    bool getPoseInit();
};

Mav::Mav(ros::NodeHandle &nh)
{
    pose_init = false;
    mav_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Mav::mav_state_cb, this);
    current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Mav::current_pose_cb, this);
}

void Mav::mav_state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
    current_state = *msg;
}

void Mav::current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{
    if(pose_init == false)
        pose_init = true;
    current_pose = *msg;
}

geometry_msgs::PoseStamped Mav::getCurrentPose(){return current_pose;}
bool Mav::getPoseInit(){return pose_init;}
mavros_msgs::State Mav::getCurrentState(){return current_state;}

void bound_yaw(double* yaw){
        if(*yaw>M_PI)
            *yaw = *yaw - 2*M_PI;
        else if(*yaw<-M_PI)
            *yaw = *yaw + 2*M_PI;
}

int main(int argc, char **argv)
{
    //  ROS_initialize  //
    ros::init(argc, argv, "keyboard_ctrl");
    ros::NodeHandle nh;

    // object
    Mav mav(nh);

    // publisher
    ros::Publisher desired_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");


    ros::Rate rate(100);
    
    // cmd_msgs

    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!mav.getPoseInit())) 
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose init %d",mav.getPoseInit());
    }
    ROS_INFO("pose initialized");
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !mav.getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    mavros_msgs::CommandTOL land_request;
    

    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        ROS_INFO("Offboard enabled");
    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        ROS_INFO("Vehicle armed");

    geometry_msgs::PoseStamped desired_pose;
    desired_pose.pose.position.x = 0;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        desired_pose_pub.publish(desired_pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Vehicle is ready to start");
    
    float move_step = 0.1;
    double desired_yaw = 0;
    bool trajectory = false;
    double trajectory_time = 0;
    double current_x = 0;
    double current_y = 0;
    while (ros::ok()) 
    {
        
        if (mav.getCurrentState().mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {

            if (!mav.getCurrentState().armed &&
                    (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        
        //keyboard control
        int c = getch();
        //ROS_INFO("C: %d",c);
        //update desired pose
        if (c != 0) 
        {
            switch (c) {
                case 65:    // key up
                    desired_pose.pose.position.z += move_step;
                    break;
                case 66:    // key down
                    desired_pose.pose.position.z += -move_step;
                    break;
                case 67:    // key CW(->)
                    desired_yaw -= 0.03;
                    bound_yaw(&desired_yaw); 
                    break;
                case 68:    // key CCW(<-)
                    desired_yaw += 0.03;
                    bound_yaw(&desired_yaw); 
                    break;
                case 119:    // key foward(w)
                    desired_pose.pose.position.y += move_step;
                    break;
                case 115:    // key back(s)
                    desired_pose.pose.position.y -= move_step;
                    break;
                case 97:    // key left(a)
                    desired_pose.pose.position.x -= move_step;
                    break;
                case 100:    // key right(d)
                    desired_pose.pose.position.x += move_step;
                    break;
                case 108:    // key land(l)
                    desired_pose.pose.position.z = 0;
                    break;
                case 101:    // key trajectory_CCW(e)
                    trajectory = true;
                    trajectory_time = 0;
                    current_x = mav.getCurrentPose().pose.position.x;
                    current_y = mav.getCurrentPose().pose.position.y;
                    break;
                case 112:    // key stop trajectory(p)
                    trajectory = false;
                    break;  
                case 111:
                    desired_pose.pose.position.x = 0;
                    desired_pose.pose.position.y = 0;
                    desired_pose.pose.position.z = 3;
                    break;
                case 107:   // key kill(k)
                    return 0;
            }
            ROS_INFO("setpoint: %.2f, %.2f, %.2f", desired_pose.pose.position.x, desired_pose.pose.position.y, desired_pose.pose.position.z);
        }
        
        if(trajectory)
        {
            trajectory_time += 0.01;
            desired_pose.pose.position.x = current_x + 2.5*cos(trajectory_time);
            desired_pose.pose.position.y = current_y + 2.5*sin(trajectory_time);
        }

        desired_pose_pub.publish(desired_pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}