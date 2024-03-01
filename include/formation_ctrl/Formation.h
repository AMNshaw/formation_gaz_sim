#ifndef FORMATION_H
#define FORMATION_H
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "state_estimation/Mav.h"

class Formation
{
private:
    int mav_num;
    std::vector<std::vector<float>> formationMap;
    std::vector<std::vector<Eigen::Vector3f>> relative_Map;
    std::vector<std::vector<bool>> laplacianMap;
    std::vector<MAV_eigen> Mavs_eigen;
    Eigen::Vector3f desiredVel;

    
public:
    Formation(int mavNum);
    ~Formation();
    void setLaplacianMap(std::vector<std::vector<bool>> LM);
    void setFormationMap(std::vector<std::vector<float>> FM);
    void setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen);
    Eigen::Vector3f computeDesiredVelocity();

    static int ID;
};



#endif