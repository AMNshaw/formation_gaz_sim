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

    desired_yaw = M_PI_2;
}
Formation::~Formation(){}

void Formation::setLaplacianMap(std::vector<std::vector<bool>> LM){ laplacianMap = LM;}
void Formation::setFormationMap(std::vector<std::vector<double>> FM)
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
void Formation::setCurr_Pose_Vel(std::vector<MAV_eigen> mavs_eigen)
{
    Mavs_eigen = mavs_eigen;
    Mavs_eigen[ID].q;
    tf::Quaternion Q(
    Mavs_eigen[ID].q.x(),
    Mavs_eigen[ID].q.y(),
    Mavs_eigen[ID].q.z(),
    Mavs_eigen[ID].q.w());
    yaw = tf::getYaw(Q);
}

Eigen::Vector3d Formation::computeDesiredLVelocity(double dt)
{
    desiredVel.setZero();
    desiredVel += virtualAcc*dt;
    virtualAcc.setZero();
    for(int i=1; i<mav_num; i++)
    {
        if(laplacianMap[ID][i])
        {
            desiredVel(0) += 0.3*(Mavs_eigen[i].r(0) - Mavs_eigen[ID].r(0) + relative_Map[ID][i](0));
            desiredVel(1) += 0.3*(Mavs_eigen[i].r(1) - Mavs_eigen[ID].r(1) + relative_Map[ID][i](1));
            //desiredVel(2) += (Mavs_eigen[i].r(2) - Mavs_eigen[ID].r(2));
        
            virtualAcc += (Mavs_eigen[i].v - Mavs_eigen[ID].v);
        }
    }
    desiredVel(0) += (Mavs_eigen[0].r(0) - Mavs_eigen[ID].r(0) + relative_Map[ID][0](0));
    desiredVel(1) += (Mavs_eigen[0].r(1) - Mavs_eigen[ID].r(1) + relative_Map[ID][0](1));
    desiredVel(2) += (Mavs_eigen[0].r(2) - Mavs_eigen[ID].r(2));
    desiredVel(2) += 0.5;

    virtualAcc += 20*(Mavs_eigen[0].v - Mavs_eigen[ID].v);
    // desiredVel(0) += Mavs_eigen[0].v(0);
    // desiredVel(1) += Mavs_eigen[0].v(1);
    
    return desiredVel;
}

double Formation::computeDesiredYawVelocity()
{
    desired_yaw = atan2(Mavs_eigen[0].r(1) - Mavs_eigen[ID].r(1), Mavs_eigen[0].r(0) - Mavs_eigen[ID].r(0));
    
    double error_yaw = desired_yaw - yaw;
    if(error_yaw>M_PI)
        error_yaw = error_yaw - 2*M_PI;
    else if(error_yaw<-M_PI)
        error_yaw = error_yaw + 2*M_PI;

    yaw_vel = error_yaw;

    return yaw_vel;
}