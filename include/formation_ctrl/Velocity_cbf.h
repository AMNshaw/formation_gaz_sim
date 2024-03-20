#ifndef VELOCITY_CBF_H
#define VELOCITY_CBF_H
#pragma once
#include <vector>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include "state_estimation/Mav.h"
#include "state_estimation/Camera.h"

class Velocity_cbf
{
private:
    int mav_num;
    int neighbor_num;
    Eigen::VectorXd cbf_optimized_vel;
    std::vector<MAV_eigen> Mavs_eigen;

    double neighbor_CD;
    double neighbor_SD;
    double target_TD; 
    double target_SD;

    double gamma;
    double kappa_HF;
    double kappa_VF;

    Camera cam;

    int varNum;
    int constraintsNum;

    Eigen::MatrixXd P;
    Eigen::VectorXd q;
    Eigen::MatrixXd A;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;
    Eigen::SparseMatrix<double> A_sparse;
    Eigen::SparseMatrix<double> hessian_Matrix;

    Eigen::VectorXd A_HF;
    double a;
    Eigen::VectorXd A_VF;
    double b;
    Eigen::VectorXd A_tS_tCA;
    Eigen::MatrixXd A_uS_uCA;

    double B_HF_u;
    double B_VF_u;
    double B_tS_u;
    Eigen::VectorXd B_uS_u;

    double B_HF_l;
    double B_VF_l;
    double B_tCA_l;
    Eigen::VectorXd B_uCA_l;

    Eigen::VectorXd QPSolution;
    
    OsqpEigen::Solver solver;
public:
    Velocity_cbf(int mavNum);
    ~Velocity_cbf();
    void setCBFparams(double G, double SD);
    void setMavsPosition(std::vector<MAV_eigen> Mavs_eigen);
    bool CBF_init(int varNum);
    bool computeCBF(Eigen::Vector3d formation_vel);
    bool computeCBF(Eigen::Vector3d formation_vel, Eigen::Vector3d angular_vel);
    Eigen::VectorXd getOptimizedVel();

    Eigen::Matrix3d skew(Eigen::Vector3d v);

    static int ID;
};
#endif