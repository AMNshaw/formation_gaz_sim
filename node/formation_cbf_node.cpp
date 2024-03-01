#include <cmath>

#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include "state_estimation/Mav.h"
#include "state_estimation/CommonFuncs.h"
#include "Formation.h"
#include "Velocity_cbf.h"

class CMD
{
    private:
        bool arm_cmd;
        int mode_cmd;
        int ID;
        int curr_mode;

        ros::NodeHandle nh;
        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    public:
        CMD(ros::NodeHandle &nh_, int id)
        {
            nh = nh_;
            ID = id;
        }
        ~CMD(){}
        ros::Subscriber arm_cmd_sub = nh.subscribe<std_msgs::Bool>("/formation/all_uav_arm", 5, &CMD::arm_cmd_cb, this);
        ros::Subscriber mode_cmd_sub = nh.subscribe<std_msgs::Int32>("/formation/all_uav_mode", 5, &CMD::mode_cmd_cb, this);
        void arm_cmd_cb(const std_msgs::Bool::ConstPtr& msg)
        {
            arm_cmd = msg->data;
            setArm(arm_cmd);
        }
        void mode_cmd_cb(const std_msgs::Int32::ConstPtr& msg)
        {
            mode_cmd = msg->data;
            setMode(mode_cmd);
        }
        void setArm(bool arm_CMD)
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = arm_CMD;
            if( arming_client.call(arm_cmd) && arm_cmd.response.success) 
                ROS_INFO("UAV_%i armed switch successfully", ID);
            else
                ROS_INFO("UAV_%i failed to arm", ID);
        }
        void setMode(int mode_CMD)
        {
            mavros_msgs::SetMode offb_set_mode;
            
            // mavros_msgs::CommandTOL srv_takeoff;
            // srv_takeoff.request.altitude = 10;
            // srv_takeoff.request.longitude = 8.5455935;
            // srv_takeoff.request.latitude = 47.3977484;
            switch (mode_CMD)
            {
            case 0: 
                offb_set_mode.request.custom_mode = "STABILIZED";
                break;
            case 1: 
                offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
                //offb_set_mode.request.custom_mode = "OFFBOARD";
                // if(takeoff_client.call(srv_takeoff))
                // {
                //     ROS_INFO("srv_takeoff send success %d", srv_takeoff.response.success);
                //     sleep(10);
                // }
                // else
                // {
                //     ROS_ERROR("Takeoff failed");
                // }
                break;
            case 2: 
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                break;
            case 3:
                offb_set_mode.request.custom_mode = "OFFBOARD";
                break;
            default:
                break;
            }
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent && curr_mode!=mode_CMD)
            {
                ROS_INFO("UAV_%i mode switched to %s", ID, offb_set_mode.request.custom_mode.c_str());
                curr_mode = mode_CMD;
                if(offb_set_mode.request.custom_mode == "AUTO.TAKEOFF")
                    sleep(5);
            }
            else
                ROS_INFO("UAV_%i failed to switch mode", ID);
        }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;

    ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    

    MAV mavs[]={MAV(nh, "iris", 0),
                MAV(nh, "iris", 1),
                MAV(nh, "iris", 2),
                MAV(nh, "iris", 3)};
    int mavNum = sizeof(mavs)/sizeof(mavs[0]);
    int ID;
    std::vector<MAV_eigen> Mavs_eigen(mavNum);
    ros::param::get("mav_id", ID);
    Formation::ID = Velocity_cbf::ID = ID;
    
    CMD cmd(nh, ID);
    Formation formation(mavNum);
    Velocity_cbf CBF(mavNum);
    float d = 4.0;
    float gamma = 1.2;
    float safe_Distance = 2;

    std::vector<std::vector<bool>> laplacianMap{{1, 0, 0, 0},
                                                {1, 1, 1, 1},
                                                {1, 1, 1, 1},
                                                {1, 1, 1, 1}};
    std::vector<std::vector<float>> formationMap{{0, 0},
                                                {0, d},
                                                {float(sqrt(3))*d/2, -d/2},
                                                {float(-sqrt(3))*d/2, -d/2}};
    formation.setLaplacianMap(laplacianMap);
    formation.setFormationMap(formationMap);

    CBF.setCBFparams(gamma, safe_Distance);


    ros::Rate rate(30);
    while (ros::ok() && !mavs[ID].getState().connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("UAV_%i waiting for FCU", ID);
    }
    while (ros::ok() && !mavs[ID].pose_init)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("UAV_%i waiting for position estimation", ID);
    }

    // cmd.setArm(true);
    // cmd.setMode(1);
    // cmd.setMode(2);

    Eigen::Vector3f formation_vel;
    Eigen::Vector3f final_vel;
    geometry_msgs::TwistStamped vel_msg;
    while (ros::ok()) 
    {
        bool armed = mavs[ID].getState().armed;
        std::cout << "UAV_" << ID << " Armed: " << armed << "\n";
        std::cout << "UAV_" << ID << " mode: " << mavs[ID].getState().mode << "\n\n";


        for(int i=0; i<mavNum; i++)
            Mavs_eigen[i] = mavMsg2Eigen(mavs[i]);
        formation.setCurr_Pose_Vel(Mavs_eigen);
        CBF.setMavsPosition(Mavs_eigen);
        formation_vel = formation.computeDesiredVelocity();

        if((ros::Time::now() - mavs[Formation::ID].getPose().header.stamp)<ros::Duration(0.1))
        {
            if(CBF.computeCBF(formation_vel))
                final_vel = formation_vel;
            else
                final_vel = CBF.getOptimizedVel();
        }
        else
            final_vel = formation_vel;

        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = final_vel(0);
        vel_msg.twist.linear.y = final_vel(1);
        vel_msg.twist.linear.z = final_vel(2);


        vel_cmd_pub.publish(vel_msg);

        rate.sleep();
        ros::spinOnce();
    }

}


