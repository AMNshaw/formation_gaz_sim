#include <cmath>

#include <ros/ros.h>
#include "ros/param.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/Mavlink.h>

#include "state_estimation/Mav.h"
#include "Formation.h"
#include "Velocity_cbf.h"
#include <state_estimation/Plot.h>

class CMD
{
    private:
        bool arm_cmd;
        int mode_cmd;
        int ID;
        int curr_mode;
        string offb_curr_mode;

        ros::NodeHandle nh;
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient takeoff_client;
    public:
        bool vision_tracking = false;

        CMD(ros::NodeHandle &nh_, int id)
        {
            nh = nh_;
            ID = id;
            offb_curr_mode = "";

            arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
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
            switch (mode_CMD)
            {
            case 0: 
                offb_set_mode.request.custom_mode = "STABILIZED";
                break;
            case 1: 
                offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";
                break;
            case 2: 
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                break;
            case 3:
                offb_set_mode.request.custom_mode = "OFFBOARD";
                break;
            case 5:
                if(!vision_tracking)
                {
                    ROS_INFO("Tracking by vision");
                    vision_tracking = true;
                }
                else
                {
                    ROS_INFO("Stop vision tracking");
                    vision_tracking = false;
                }
                break;
            
            default:
                break;
            }
            if(offb_set_mode.request.custom_mode != offb_curr_mode)
            {
                if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("UAV_%i mode switched to %s", ID, offb_set_mode.request.custom_mode.c_str());
                    offb_curr_mode = offb_set_mode.request.custom_mode;
                    if(offb_set_mode.request.custom_mode == "AUTO.TAKEOFF")
                        sleep(5);
                }
                else
                    ROS_INFO("UAV_%i failed to switch offb_mode", ID);
            }
        }
        int mode_CMD(){return mode_cmd;}
};

class Target_EST_FeedBack
{
private:
    ros::Subscriber target_est_sub;
    ros::Subscriber isTargetEst_sub1;
    ros::Subscriber isTargetEst_sub2;
    ros::Subscriber isTargetEst_sub3;
    geometry_msgs::Pose target_est_pose;
    geometry_msgs::Twist target_est_twist;
    
public:
    bool estimating[3];
    Target_EST_FeedBack(ros::NodeHandle& nh_)
    {   
        target_est_sub = nh_.subscribe<state_estimation::Plot>("THEIF/Plot", 10, &Target_EST_FeedBack::est_state_cb, this);
        isTargetEst_sub1 = nh_.subscribe<std_msgs::Bool>("/uav1/THEIF/isTargetEst", 10, &Target_EST_FeedBack::isTargetEst_cb1, this);
        isTargetEst_sub2 = nh_.subscribe<std_msgs::Bool>("/uav2/THEIF/isTargetEst", 10, &Target_EST_FeedBack::isTargetEst_cb2, this);
        isTargetEst_sub3 = nh_.subscribe<std_msgs::Bool>("/uav3/THEIF/isTargetEst", 10, &Target_EST_FeedBack::isTargetEst_cb3, this);
    }
    void est_state_cb(const state_estimation::Plot::ConstPtr& msg)
    {
        target_est_pose = msg->est_pose;
        target_est_twist = msg->est_twist;
    }
    void isTargetEst_cb1(const std_msgs::Bool::ConstPtr& msg)
    {
        estimating[0] = msg->data;
    }
    void isTargetEst_cb2(const std_msgs::Bool::ConstPtr& msg)
    {
        estimating[1] = msg->data;
    }
    void isTargetEst_cb3(const std_msgs::Bool::ConstPtr& msg)
    {
        estimating[2] = msg->data;
    }
    geometry_msgs::Pose getPose(){return target_est_pose;}
    geometry_msgs::Twist getTwist(){return target_est_twist;}
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation");
    ros::NodeHandle nh;

    ros::Publisher vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    MAV mavs[]={MAV(nh, "target", 0),
                MAV(nh, "uav", 1),
                MAV(nh, "uav", 2),
                MAV(nh, "uav", 3)};
    int mavNum = sizeof(mavs)/sizeof(mavs[0]);
    int ID;
    std::vector<MAV_eigen> Mavs_eigen(mavNum);
    ros::param::get("mav_id", ID);
    Formation::ID = Velocity_cbf::ID = ID;
    
    CMD cmd(nh, ID);
    Target_EST_FeedBack target(nh);
    Formation formation(mavNum);
    Velocity_cbf CBF(mavNum);
    double d = 4.0;
    double gamma = 1.2;
    double safe_Distance = 3;

    std::vector<std::vector<bool>> laplacianMap{{1, 0, 0, 0},
                                                {1, 1, 1, 1},
                                                {1, 1, 1, 1},
                                                {1, 1, 1, 1}};
    std::vector<std::vector<double>> formationMap{{0, 0},
                                                {0, d},
                                                {double(sqrt(3))*d/2, -d/2},
                                                {double(-sqrt(3))*d/2, -d/2}};
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
    
    Eigen::Vector3d formation_vel;
    Eigen::VectorXd final_vel;
    double yaw_vel;
    geometry_msgs::TwistStamped vel_msg;

    double dt = 0.001;
	double last_t = ros::Time::now().toSec();

    while (ros::ok()) 
    {
        bool armed = mavs[ID].getState().armed;
        std::cout << "[UAV_" << ID << "]:\n";
        std::cout << "Armed: " << armed << "\n";
        std::cout << "Mode: " << mavs[ID].getState().mode << "\n";

        if(cmd.vision_tracking)
        {
            if(target.estimating[0] || target.estimating[1] || target.estimating[2])
            {
                std::cout << "Vision tracking mode\n\n";
                mavs[0].setPose(target.getPose());
                mavs[0].setTwist(target.getTwist());
            }
            else
            {
                ROS_INFO("Target lost, unable to track");
                cmd.vision_tracking = false;
                geometry_msgs::Twist stop;
                stop.linear.x = 0;
                stop.linear.y = 0;
                stop.linear.z = 0;
                mavs[0].setTwist(stop);
            }
                
        }
        for(int i=0; i<mavNum; i++)
            Mavs_eigen[i] = mavMsg2Eigen(mavs[i]);
        formation.setCurr_Pose_Vel(Mavs_eigen);
        CBF.setMavsPosition(Mavs_eigen);
        formation_vel = formation.computeDesiredLVelocity(dt);
        yaw_vel = formation.computeDesiredYawVelocity();
        Eigen::Vector3d ang_vel = Eigen::Vector3d::Zero();
        ang_vel(2) = yaw_vel;

        final_vel = formation_vel;
        if(cmd.mode_CMD() == 5)
        {
            
            if((ros::Time::now() - mavs[Formation::ID].getPose().header.stamp)<ros::Duration(0.1))
            {
                if(CBF.CBF_init(6))
                {
                    if(CBF.computeCBF(formation_vel, ang_vel))
                    {
                        final_vel = CBF.getOptimizedVel();
                    }
                    else
                        final_vel = formation_vel;
                }
            }                
        }
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = final_vel(0);
        vel_msg.twist.linear.y = final_vel(1);
        vel_msg.twist.linear.z = final_vel(2);
        vel_msg.twist.angular.z = yaw_vel;
        if(final_vel.size() > 3)
        {
            vel_msg.twist.angular.x = final_vel(3);
            vel_msg.twist.angular.y = final_vel(4);
            vel_msg.twist.angular.z = final_vel(5);
        }
        

        vel_cmd_pub.publish(vel_msg);

        dt = ros::Time::now().toSec() - last_t;
    	last_t = ros::Time::now().toSec();

        rate.sleep();
        ros::spinOnce();
    }

}


