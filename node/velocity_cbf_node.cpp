#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <queue>
#include "Mav.h"
#define gravity 9.806

using namespace std;

bool desired_input_init = false;

//set control P-gain
double KPx=1, KPy=1, KPz=1.2;
double KPyaw = 1;

double roll = 0, pitch = 0, yaw = 0;

// var for desired_pose
geometry_msgs::PoseStamped desired_pose;
double desired_yaw = 0;
int kill_all_drone = 0;
int start_all_drone = 0;
int takeoff_all_drone = 0;
// var for desired_velocity
geometry_msgs::TwistStamped desired_vel_raw;
geometry_msgs::TwistStamped desired_vel;        //output

// var for obstacle
//sgeometry_msgs::PoseStamped obstacle_pose;

struct CBF_param
{
    float obs_safeDistance;
    float obs_gamma;
    float mav_safeDistance;
    float mav_gamma;
}cbf_param;


void desired_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    if(desired_input_init == false)
        desired_input_init = true;
    desired_vel_raw = *msg;
}


int velocity_cbf(geometry_msgs::TwistStamped desired_vel_raw,geometry_msgs::TwistStamped* desired_vel, MAV mav[])
{
        int cbf_num = 0;
        for(int i = 0; i < 5; i++)
            if(mav[i].getPoseInit() == true)
                cbf_num++;


        Eigen::SparseMatrix<double> hessian_Matrix;
        Eigen::VectorXd gradient;
        Eigen::SparseMatrix<double> linearMatrix;
        Eigen::VectorXd lowerBound;
        Eigen::VectorXd upperBound;
        hessian_Matrix.resize(2,2);
        hessian_Matrix.insert(0,0) = 1;
        hessian_Matrix.insert(1,0) = 0;
        hessian_Matrix.insert(0,1) = 0;
        hessian_Matrix.insert(1,1) = 1;

        gradient.resize(2);
        gradient << - desired_vel_raw.twist.linear.x , - desired_vel_raw.twist.linear.y;
       
        
       ////////////////////// cbf constraints ///////////////////////// 
        upperBound.resize(cbf_num-1);
        lowerBound.resize(cbf_num-1);
        linearMatrix.resize(cbf_num-1,2);

        int j = 0;
        for(int i = 0; i < cbf_num; i++)
        {
            if(i != MAV::self_id && mav[i].getPoseInit() == true)
            {
                linearMatrix.insert(j,0) = 2*(mav[i].getPose().pose.position.x - mav[MAV::self_id].getPose().pose.position.x );
                linearMatrix.insert(j,1) = 2*(mav[i].getPose().pose.position.y - mav[MAV::self_id].getPose().pose.position.y );
                upperBound(j) = cbf_param.mav_gamma*(
                    pow((mav[i].getPose().pose.position.x - mav[MAV::self_id].getPose().pose.position.x ),2)+
                    pow((mav[i].getPose().pose.position.y - mav[MAV::self_id].getPose().pose.position.y ),2)-
                    pow(cbf_param.mav_safeDistance,2));
                /*
                if(mav[i].getVelInit() == true)
                    upperBound(j)+= -2*mav[i].getVel().twist.linear.x*(mav[i].getPose().pose.position.x - mav[MAV::self_id].getPose().pose.position.x)
                                  - 2*mav[i].getVel().twist.linear.y*(mav[i].getPose().pose.position.y - mav[MAV::self_id].getPose().pose.position.y);
                
                */
                lowerBound(j) = -OsqpEigen::INFTY;

                j++;
            }   
        }

        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);

        solver.data()->setNumberOfVariables(2);
        solver.data()->setNumberOfConstraints(cbf_num-1);

        if(!solver.data()->setHessianMatrix(hessian_Matrix)) return 1;
        if(!solver.data()->setGradient(gradient)) return 1;
        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
        if(!solver.data()->setLowerBound(lowerBound)) return 1;
        if(!solver.data()->setUpperBound(upperBound)) return 1;
        if(!solver.initSolver()) return 1;
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        Eigen::VectorXd QPSolution;
        QPSolution = solver.getSolution();
        *desired_vel = desired_vel_raw;
        desired_vel->twist.linear.x = QPSolution(0);
        desired_vel->twist.linear.y = QPSolution(1);

        return 0;
}

void start_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    start_all_drone = msg.data;
}

void takeoff_cb(const std_msgs::Int32 msg){
    //store odometry into global variable
    takeoff_all_drone = msg.data;
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
    
    //    subscriber    //
    ros::Subscriber desired_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("desired_velocity_raw", 10, desired_vel_cb);
    
    ros::Subscriber uav_start_sub = nh.subscribe<std_msgs::Int32>("/uav_start", 10, start_cb);
    ros::Subscriber uav_killer_sub = nh.subscribe<std_msgs::Int32>("/uav_kill", 10, kill_cb);
    ros::Subscriber uav_takeoff_sub = nh.subscribe<std_msgs::Int32>("/uav_takeoff", 10, takeoff_cb);
    // publisher
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 2);
    // service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::Rate rate(100);

    ros::param::get("mav_id", MAV::self_id);
    ros::param::get("obs_gamma", cbf_param.obs_gamma);
    ros::param::get("obs_safeDistance", cbf_param.obs_safeDistance);
    ros::param::get("mav_gamma", cbf_param.mav_gamma);
    ros::param::get("mav_safeDistance", cbf_param.mav_safeDistance);


    MAV mav[4] = {MAV(nh, "/target/mavros/local_position/pose_initialized", "/target/mavros/local_position/velocity_local", 0),
                  MAV(nh, "/iris_1/mavros/local_position/pose_initialized", 1),
                  MAV(nh, "/iris_2/mavros/local_position/pose_initialized", 2),
                  MAV(nh, "/iris_3/mavros/local_position/pose_initialized", 3)};

    ROS_INFO("Wait for pose and desired input init");
    while (ros::ok() && (!desired_input_init || !mav[MAV::self_id].getPoseInit())) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for pose and desired input init %d,%d",desired_input_init,mav[MAV::self_id].getPoseInit());
        ROS_INFO("\nself_id: %d", MAV::self_id);
    }
    ROS_INFO("pose initialized");
    
    ROS_INFO("Wait for FCU connection");
    while (ros::ok() && !mav[MAV::self_id].getCurrentState().connected) {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for FCU");
    }
    ROS_INFO("FCU connected");

    for(int i = 0; ros::ok() && i < 20; i++){
        local_vel_pub.publish(desired_vel);
        rate.sleep();
    }
    
    while (ros::ok()) {
        if(start_all_drone == 1){
            break;
        }
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("Wait for UAV all start signal");
    }
    ROS_INFO("get UAV all start signal");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled");
    }

    if( arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }

    while (ros::ok()) {
        if (mav[MAV::self_id].getCurrentState().mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0))) 
        {
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent) 
            {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request = ros::Time::now();
        }
    
        //keyboard control
        if(kill_all_drone == 1){
            ROS_WARN("velocity_cbf_kill!");
            offb_set_mode.request.custom_mode = "STABILIZED";
            set_mode_client.call(offb_set_mode);
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
        }

        if(( ros::Time::now() - mav[0].getPose().header.stamp)<ros::Duration(0.1))
        {
            if(velocity_cbf( desired_vel_raw , &desired_vel, mav)!=0)
                desired_vel = desired_vel_raw;
        }
        else
            desired_vel = desired_vel_raw;
        
        local_vel_pub.publish(desired_vel);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



