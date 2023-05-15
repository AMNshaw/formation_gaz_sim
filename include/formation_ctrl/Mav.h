#ifndef MAV_H
#define MAV_H
#pragma once

#include <cmath>
#include <cstdio>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>

using namespace std;

class MAV
{
private:
    bool pose_init;
    bool vel_init;
    int id;
    double roll;
    double pitch;
    double yaw;

    mavros_msgs::State state;
    geometry_msgs::PoseStamped pose_current;
    geometry_msgs::TwistStamped vel_current;

    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub;
    ros::Subscriber mav_state_sub;

public:
    MAV();
    MAV(ros::NodeHandle &nh);
    MAV(ros::NodeHandle &nh, string pose_topic, int ID);
    MAV(ros::NodeHandle &nh, string pose_topic, string vel_topic, int ID);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void mav_state_cb(const mavros_msgs::State::ConstPtr& msg);
    geometry_msgs::PoseStamped getPose();
    geometry_msgs::TwistStamped getVel();
    mavros_msgs::State getCurrentState();
    bool getPoseInit();
    bool getVelInit();
    double getYaw();

    static int self_id;
};

#endif