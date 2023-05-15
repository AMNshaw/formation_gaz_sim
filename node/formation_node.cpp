#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cstdio>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include "getch.h"
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <queue>
#include <Mav.h>

using namespace std;

bool init = false;
bool start = false;

geometry_msgs::TwistStamped leader_vel;

void leader_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    leader_vel = *msg;
}

void laplacian_remap(XmlRpc::XmlRpcValue laplacian_param, bool laplacian_map[][5])
{
    int k = 0;
    for(int i = 0; i < 5; i++)
    {
        for(int j = 0; j < 5; j++)
        {
    	    ROS_ASSERT(laplacian_param[k].getType() == XmlRpc::XmlRpcValue::TypeInt);
            int a = laplacian_param[k];
    	    laplacian_map[i][j] = a!=0;
    	    k++;
        }
    }
}

geometry_msgs::TwistStamped vel_limit(geometry_msgs::TwistStamped desired_vel, float limit)
{
    float vel_norm = sqrt(pow(desired_vel.twist.linear.x, 2) + pow(desired_vel.twist.linear.y, 2) + pow(desired_vel.twist.linear.z, 2));
    if(vel_norm > limit)
    {
        desired_vel.twist.linear.x *= limit/vel_norm;
        desired_vel.twist.linear.y *= limit/vel_norm;
        desired_vel.twist.linear.z *= limit/vel_norm;
    }

    return desired_vel;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;

    ros::param::get("mav_id", MAV::self_id);
    int uav_num = 4;
    //Subscriber
    MAV mav[4] = {MAV(nh, "/leader_pose", 0),
                  MAV(nh, "/iris1/mavros/local_position/pose_initialized", 1),
                  MAV(nh, "/iris2/mavros/local_position/pose_initialized", 2),
                  MAV(nh, "/iris3/mavros/local_position/pose_initialized", 3)};

    ros::Subscriber leader_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/leader_vel", 10, leader_vel_cb);
    //Publisher    
    ros::Publisher desired_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("desired_velocity_raw", 100);

    bool laplacian_map[5][5] = {1, 0, 0, 0, 0,
                                1, 1, 0, 0, 0,
                                1, 1, 1, 0, 0,
                                1, 1, 1, 1, 0,
                                1, 1, 1, 1, 0};
    

    double d = 5.0;
    double leader_uav_vector_x[5] = {0, 0,  sqrt(3)*d/2, -sqrt(3)*d/2, 0};  //active: mav2, mav4 
    double leader_uav_vector_y[5] = {0, d, -d/2,  -d/2, 0};  //vector y from leader to uav
    double relative_map_x[5][5];
    double relative_map_y[5][5];
    for(int i = 0 ; i<5; i++)
    {
        for(int j = 0 ; j<5 ; j++){
            relative_map_x[i][j] = leader_uav_vector_x[i] - leader_uav_vector_x[j];
            relative_map_y[i][j] = leader_uav_vector_y[i] - leader_uav_vector_y[j];
        }
    }

    ros::Rate rate(100);

    geometry_msgs::TwistStamped desired_vel;

    while (ros::ok()) {
        desired_vel.twist.linear.x = 0;
        desired_vel.twist.linear.y = 0;
        desired_vel.twist.linear.z = 0;
        for(int i =0 ;i<uav_num;i++)
        {
            if(laplacian_map[MAV::self_id][i] == 1)
            {
                desired_vel.twist.linear.x += (mav[i].getPose().pose.position.x
                                                - mav[MAV::self_id].getPose().pose.position.x
                                                + relative_map_x[MAV::self_id][i]);
                desired_vel.twist.linear.y += (mav[i].getPose().pose.position.y
                                                - mav[MAV::self_id].getPose().pose.position.y
                                                + relative_map_y[MAV::self_id][i]);
                desired_vel.twist.linear.z += (mav[i].getPose().pose.position.z
                                                - mav[MAV::self_id].getPose().pose.position.z);
            }
        }
        desired_vel.twist.linear.x += leader_vel.twist.linear.x;
        desired_vel.twist.linear.y += leader_vel.twist.linear.y;
        desired_vel.twist.linear.z += leader_vel.twist.linear.z;

        desired_vel = vel_limit(desired_vel, 5);
        desired_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}



