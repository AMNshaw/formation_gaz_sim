#include "Velocity_cbf.h"

int Velocity_cbf::ID = 0;

Velocity_cbf::Velocity_cbf(int mavNum)
{
    mav_num = mavNum;
    Mavs_pose.resize(mav_num);
    
    gamma = 1.2;
    safe_Distance = 2.0;

    // hessian_Matrix.resize(2,2);
    // gradient.resize(2);

    // hessian_Matrix.insert(0,0) = 1;
    // hessian_Matrix.insert(1,0) = 0;
    // hessian_Matrix.insert(0,1) = 0;
    // hessian_Matrix.insert(1,1) = 1;
    
    // upperBound.resize(mav_num-1);
    // lowerBound.resize(mav_num-1);
    // linearMatrix.resize(mav_num-1,2);
}

Velocity_cbf::~Velocity_cbf(){}

void Velocity_cbf::setCBFparams(float G, float SD)
{
    gamma = G;
    safe_Distance = SD;
}

void Velocity_cbf::setMavsPosition(std::vector<MAV_eigen> Mavs_eigen)
{
    for(int i=0; i<mav_num; i++)
        Mavs_pose[i] = Mavs_eigen[i].r;
}

Eigen::Vector3f Velocity_cbf::getOptimizedVel(){return cbf_optimized_vel;}

bool Velocity_cbf::computeCBF(Eigen::Vector3f formation_vel)
{
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


    gradient << - formation_vel(0) , - formation_vel(1);
    ////////////////////// cbf constraints /////////////////////////
    upperBound.resize(mav_num-1);
    lowerBound.resize(mav_num-1);
    linearMatrix.resize(mav_num-1,2);

    int j = 0;
    for(int i = 0; i < mav_num; i++)
    {
        if(i != ID)
        {
            linearMatrix.insert(j,0) = 2*(Mavs_pose[i](0) - Mavs_pose[ID](0));
            linearMatrix.insert(j,1) = 2*(Mavs_pose[i](1) - Mavs_pose[ID](1));
            upperBound(j) = gamma*(
                pow((Mavs_pose[i](0) - Mavs_pose[ID](0)),2)+
                pow((Mavs_pose[i](1) - Mavs_pose[ID](1)),2)-
                pow(safe_Distance,2));
            lowerBound(j) = -OsqpEigen::INFTY;
            j++;
        }   
    }
    

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(mav_num-1);
    
    if(!solver.data()->setHessianMatrix(hessian_Matrix)) return true;
    if(!solver.data()->setGradient(gradient)) return true;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return true;
    if(!solver.data()->setLowerBound(lowerBound)) return true;
    if(!solver.data()->setUpperBound(upperBound)) return true;
    if(!solver.initSolver()) return true;
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return true;
    
    Eigen::VectorXd QPSolution;
    QPSolution = solver.getSolution();
    cbf_optimized_vel = formation_vel;
    cbf_optimized_vel(0) = QPSolution(0);
    cbf_optimized_vel(1) = QPSolution(1);

    return 0;
}