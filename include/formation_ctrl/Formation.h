#ifndef FORMATION_H
#define FORMATION_H
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "state_estimation/Mav.h"
#include <tf/tf.h>

class Formation
{
private:
    int mav_num;
    std::vector<std::vector<double>> formationMap;
    std::vector<std::vector<Eigen::Vector3d>> relative_Map;
    std::vector<std::vector<bool>> laplacianMap;
    std::vector<MAV_eigen> Mavs_eigen;
    Eigen::Vector3d desiredVel;
    Eigen::Vector3d virtualAcc;
    double desired_yaw;
    double yaw_vel;

    double roll;
    double pitch;
    double yaw;

    
public:
    Formation(int mavNum);
    ~Formation();
    void setLaplacianMap(std::vector<std::vector<bool>> LM);
    void setFormationMap(std::vector<std::vector<double>> FM);
    void setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen);
    Eigen::Vector3d computeDesiredLVelocity(double dt);
    double computeDesiredYawVelocity();

    static int ID;
};



#endif