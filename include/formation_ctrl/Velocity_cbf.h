#ifndef VELOCITY_CBF_H
#define VELOCITY_CBF_H
#pragma once

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include "state_estimation/Mav.h"

class Velocity_cbf
{
private:
    int mav_num;
    Eigen::Vector3f cbf_optimized_vel;
    std::vector<Eigen::Vector3f> Mavs_pose;

    float safe_Distance;
    float gamma;

    // Eigen::SparseMatrix<double> hessian_Matrix;
    // Eigen::SparseMatrix<double> linearMatrix;
    // Eigen::VectorXd gradient;
    // Eigen::VectorXd lowerBound;
    // Eigen::VectorXd upperBound;
    // Eigen::VectorXd QPSolution;

    // OsqpEigen::Solver solver;
public:
    Velocity_cbf(int mavNum);
    ~Velocity_cbf();
    void setCBFparams(float G, float SD);
    void setMavsPosition(std::vector<MAV_eigen> Mavs_eigen);
    bool computeCBF(Eigen::Vector3f formation_vel);
    Eigen::Vector3f getOptimizedVel();

    static int ID;
};
#endif