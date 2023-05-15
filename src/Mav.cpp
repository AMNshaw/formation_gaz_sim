#include "Mav.h"

int MAV::self_id = 0;

MAV::MAV(ros::NodeHandle &nh)
{
    pose_init = false;
    mav_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MAV::mav_state_cb, this);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &MAV::pose_cb, this);
    roll = pitch = yaw = 0;
}

MAV::MAV(ros::NodeHandle &nh, string pose_topic, int ID)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 10, &MAV::pose_cb, this);
    mav_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MAV::mav_state_cb, this);
    id = ID;
    pose_init = false;
    roll = pitch = yaw = 0;
}

MAV::MAV(ros::NodeHandle &nh, string pose_topic, string vel_topic, int ID)
{
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 10, &MAV::pose_cb, this);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(vel_topic, 10, &MAV::vel_cb, this);
    mav_state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MAV::mav_state_cb, this);
    id = ID;
    pose_init = false;
    vel_init = false;
    roll = pitch = yaw = 0;
}

void MAV::mav_state_cb(const mavros_msgs::State::ConstPtr& msg) 
{
    state = *msg;
}

void MAV::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(!pose_init)
        pose_init = true;
    pose_current = *msg;

    tf::Quaternion Q(
        pose_current.pose.orientation.x,
        pose_current.pose.orientation.y,
        pose_current.pose.orientation.z,
        pose_current.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}

void MAV::vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if(!vel_init)
        vel_init = true;
    vel_current = *msg;
}

geometry_msgs::PoseStamped MAV::getPose(){return pose_current;}
geometry_msgs::TwistStamped MAV::getVel(){return vel_current;}
bool MAV::getPoseInit(){return pose_init;}
bool MAV::getVelInit(){return vel_init;}
mavros_msgs::State MAV::getCurrentState(){return state;}
double MAV::getYaw(){return yaw;}