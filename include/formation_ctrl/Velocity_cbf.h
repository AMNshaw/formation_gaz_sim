#ifndef VELOCITY_CBF_H
#define VELOCITY_CBF_H
#pragma once

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include "state_estimation/Mav.h"
#include "state_estimation/Camera.h"

class Velocity_cbf
{
private:
    int mav_num;
    Eigen::VectorXd cbf_optimized_vel;
    std::vector<MAV_eigen> Mavs_eigen;
    std::vector<Eigen::Vector3d> Mavs_pose;

    double safe_Distance;
    double track_Distance;
    double gamma;
    double kappa_HF;
    double kappa_VF;

    Camera cam;

    int varNum;
    int constraintsNum;

    Eigen::MatrixXd P;
    Eigen::VectorXd q;
    Eigen::VectorXd A_HF;
    double B_HF_u;
    double B_HF_l;
    Eigen::VectorXd A_VF;
    double B_VF_u;
    double B_VF_l;
    Eigen::VectorXd A_tT_tCA;
    double B_tT_u;
    double B_tCA_l;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd QPSolution;
    Eigen::SparseMatrix<double> hessian_Matrix;
    Eigen::SparseMatrix<double> constraintA;
    
    OsqpEigen::Solver solver;
public:
    Velocity_cbf(int mavNum);
    ~Velocity_cbf();
    void setCBFparams(double G, double SD);
    void setMavsPosition(std::vector<MAV_eigen> Mavs_eigen);
    bool CBF_init(int varNum, int constraintsNum);
    bool computeCBF(Eigen::Vector3d formation_vel);
    bool computeCBF(Eigen::Vector3d formation_vel, Eigen::Vector3d angular_vel);
    Eigen::VectorXd getOptimizedVel();

    Eigen::Matrix3d skew(Eigen::Vector3d v);

    static int ID;
};
#endif