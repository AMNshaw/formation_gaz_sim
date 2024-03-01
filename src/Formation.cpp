#include "Formation.h"
int Formation::ID = 0;

Formation::Formation(int mavNum)
{
    mav_num = mavNum;

    formationMap.resize(mavNum);
    for(auto& xy : formationMap)
        xy.resize(2);
    
    relative_Map.resize(mavNum);
    for(auto& j : relative_Map)
        j.resize(mavNum);
    
    laplacianMap.resize(mavNum);
    for(auto& j : relative_Map)
        j.resize(mavNum);
}
Formation::~Formation(){}

void Formation::setLaplacianMap(std::vector<std::vector<bool>> LM){ laplacianMap = LM;}
void Formation::setFormationMap(std::vector<std::vector<float>> FM)
{
    formationMap = FM;
    for(int i=0; i<mav_num; i++)
    {
        for(int j=0; j<mav_num; j++)
        {
            relative_Map[i][j](0) = formationMap[i][0] - formationMap[j][0];
            relative_Map[i][j](1) = formationMap[i][1] - formationMap[j][1];
        }
    }
}
void Formation::setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen){ Mavs_eigen = mavs_eigen;}
Eigen::Vector3f Formation::computeDesiredVelocity()
{
    desiredVel.setZero();
    for(int i=0; i<mav_num; i++)
    {
        if(laplacianMap[ID][i])
        {
            desiredVel(0) += (Mavs_eigen[i].r(0) - Mavs_eigen[ID].r(0) + relative_Map[ID][i](0));
            desiredVel(1) += (Mavs_eigen[i].r(1) - Mavs_eigen[ID].r(1) + relative_Map[ID][i](1));
            desiredVel(2) += (Mavs_eigen[i].r(2) - Mavs_eigen[ID].r(2));
        }
        
    }
    desiredVel += Mavs_eigen[0].v;
    return desiredVel;
}