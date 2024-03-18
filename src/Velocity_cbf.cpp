#include "Velocity_cbf.h"

int Velocity_cbf::ID = 0;

Velocity_cbf::Velocity_cbf(int mavNum)
{
    mav_num = mavNum;
    Mavs_pose.resize(mav_num);
    
    gamma = 1.2;
    kappa_HF = 3;
    kappa_VF = 2;

    safe_Distance = 2.0;
    track_Distance = 4.0;
}

Velocity_cbf::~Velocity_cbf(){}

void Velocity_cbf::setCBFparams(double G, double SD)
{
    gamma = G;
    safe_Distance = SD;
}

void Velocity_cbf::setMavsPosition(std::vector<MAV_eigen> mavs_eigen)
{
    Mavs_eigen = mavs_eigen;
    Mavs_eigen[ID].r_c = Mavs_eigen[ID].r + Mavs_eigen[ID].R_w2b.inverse()*cam.t_B2C();
}

Eigen::VectorXd Velocity_cbf::getOptimizedVel(){return cbf_optimized_vel;}

bool Velocity_cbf::CBF_init(int var_Num, int constraints_Num)
{
    varNum = var_Num;
    constraintsNum = constraints_Num;

    P.setIdentity(varNum, varNum);
    q.resize(varNum);
    A_HF.resize(varNum); A_HF.setZero();
    A_VF.resize(varNum); A_VF.setZero();
    A_tT_tCA.resize(varNum); A_tT_tCA.setZero();

    solver.clearSolver();
    solver.data()->setNumberOfVariables(varNum);
    solver.data()->setNumberOfConstraints(constraintsNum);
	solver.settings()->setWarmStart(true);
	solver.settings()->setMaxIteration(25);
	solver.settings()->setRelativeTolerance(1e-3);
    solver.settings()->setVerbosity(false); // Set to true if you want to see OSQP's output
    
    return true;
}

bool Velocity_cbf::computeCBF(Eigen::Vector3d formation_vel, Eigen::Vector3d angular_vel)
{
    P.setIdentity(varNum, varNum);
    hessian_Matrix = P.sparseView();

    q << -2*formation_vel, -2*angular_vel;

    Eigen::MatrixXd R_W2C = cam.R_B2C()*Mavs_eigen[ID].R_w2b;
    Eigen::Vector3d r_tc_C = R_W2C*(Mavs_eigen[0].r - Mavs_eigen[ID].r_c);
    Eigen::MatrixXd r_x = skew(r_tc_C);
    double r_tc_C_xz = sqrt(r_tc_C(0)*r_tc_C(0) + r_tc_C(2)*r_tc_C(2));
    double max_FOV_H = atan2(cam.lx(), 2*cam.fx());
    double max_FOV_V = atan2(cam.ly(), 2*cam.fy());
    double err_FOV_H = max_FOV_H/3 - atan2(abs(r_tc_C(0)), r_tc_C(2));
    double err_FOV_V = max_FOV_V - atan2(abs(r_tc_C(1)), r_tc_C_xz);

    double a = -1/(1 + pow(abs(r_tc_C(0))/r_tc_C(2), 2));
    A_HF.segment(0, 3) = a*(-r_tc_C(0)/(abs(r_tc_C(0))*r_tc_C(2))*R_W2C.row(0)
                        + a*abs(r_tc_C(0))/pow(r_tc_C(2), 2)*R_W2C.row(2));
    A_HF.segment(3, 3) = a*(-r_tc_C(0)/(abs(r_tc_C(0))*r_tc_C(2))*r_x.row(0)
                        + a*abs(r_tc_C(0))/pow(r_tc_C(2), 2)*r_x.row(2));
    B_HF_l = -kappa_HF*(err_FOV_H);
            // - a*(-r_tc_C(0)/(abs(r_tc_C(0))*r_tc_C(2))*R_W2C.row(0) 
            // + abs(r_tc_C(0))/pow(r_tc_C(2), 2)*R_W2C.row(2))*Mavs_eigen[0].v;
    
    double b = -1/(1 + pow(abs(r_tc_C(1))/r_tc_C_xz, 2));
    A_VF.segment(0, 3) = b*((r_tc_C(0)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(0)
                        + (r_tc_C(2)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(2)
                        - (r_tc_C(1)/(abs(r_tc_C(1))*r_tc_C_xz))*R_W2C.row(1));
    A_VF.segment(3, 3) = b*((r_tc_C(0)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*r_x.row(0)
                        + (r_tc_C(2)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*r_x.row(2)
                        - (r_tc_C(1)/(abs(r_tc_C(1))*r_tc_C_xz))*r_x.row(1));
    B_VF_l = -kappa_VF*(err_FOV_V);
            // + b*((r_tc_C(0)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(0)
            // + (r_tc_C(2)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(2)
            // - (r_tc_C(1)/(abs(r_tc_C(1))*r_tc_C_xz))*R_W2C.row(1)) * Mavs_eigen[0].v;
    
    A_tT_tCA.segment(0, 3) = 2*(Mavs_eigen[0].r - Mavs_eigen[ID].r);

    B_tT_u = gamma * (pow((Mavs_eigen[0].r - Mavs_eigen[ID].r).norm(), 2) - pow(safe_Distance,2));
            // + 2*(Mavs_eigen[0].r - Mavs_eigen[ID].r).dot(Mavs_eigen[0].v);
    B_tCA_l = -gamma * (pow(track_Distance,2) - pow((Mavs_eigen[0].r - Mavs_eigen[ID].r).norm(), 2));
            // + 2*(Mavs_eigen[0].r - Mavs_eigen[ID].r).dot(Mavs_eigen[0].v);

    Eigen::MatrixXd A(constraintsNum, varNum);
    Eigen::VectorXd B_ub(constraintsNum), B_lb(constraintsNum);
    A.row(0) = A_HF;
    A.row(1) = A_VF;
    A.row(2) = A_tT_tCA;
    B_ub << OsqpEigen::INFTY, OsqpEigen::INFTY, B_tT_u;
    B_lb << B_HF_l, B_VF_l, B_tCA_l;
    constraintA = A.sparseView();

    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();
    if(!solver.data()->setHessianMatrix(hessian_Matrix)) return false;
    if(!solver.data()->setGradient(q)) return false;
    if(!solver.data()->setLinearConstraintsMatrix(constraintA)) return false;
    if(!solver.data()->setUpperBound(B_ub)) return false;
    if(!solver.data()->setLowerBound(B_lb)) return false;
    
    
    if(!solver.initSolver()) return false;
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
    
    cbf_optimized_vel = solver.getSolution();
    cbf_optimized_vel.segment(3, 3) = -cam.R_B2C().inverse()*cbf_optimized_vel.segment(3, 3);
    return true;
}

Eigen::Matrix3d Velocity_cbf::skew(Eigen::Vector3d v)
{
	Eigen::Matrix3d vec_skew;
	vec_skew << 0, -v(2), v(1),
				v(2), 0, -v(0),
				-v(1), v(0), 0;
	return vec_skew;
}
// bool Velocity_cbf::computeCBF(Eigen::Vector3d formation_vel)
// {
//     hessian_Matrix.resize(2,2);
//     hessian_Matrix.insert(0,0) = 1;
//     hessian_Matrix.insert(1,0) = 0;
//     hessian_Matrix.insert(0,1) = 0;
//     hessian_Matrix.insert(1,1) = 1;

//     gradient.resize(2);


//     gradient << - formation_vel(0) , - formation_vel(1);
//     ////////////////////// cbf constraints /////////////////////////
//     upperBound.resize(mav_num-1);
//     lowerBound.resize(mav_num-1);
//     linearMatrix.resize(mav_num-1,2);

//     int j = 0;
//     for(int i = 0; i < mav_num; i++)
//     {
//         if(i != ID)
//         {
//             linearMatrix.insert(j,0) = 2*(Mavs_pose[i](0) - Mavs_pose[ID](0));
//             linearMatrix.insert(j,1) = 2*(Mavs_pose[i](1) - Mavs_pose[ID](1));
//             upperBound(j) = gamma*(
//                 pow((Mavs_pose[i](0) - Mavs_pose[ID](0)),2)+
//                 pow((Mavs_pose[i](1) - Mavs_pose[ID](1)),2)-
//                 pow(safe_Distance,2));
//             lowerBound(j) = -OsqpEigen::INFTY;
//             j++;
//         }   
//     }
    

//     OsqpEigen::Solver solver;
//     solver.settings()->setWarmStart(true);
//     solver.settings()->setVerbosity(false);

//     solver.data()->setNumberOfVariables(2);
//     solver.data()->setNumberOfConstraints(mav_num-1);
    
//     if(!solver.data()->setHessianMatrix(hessian_Matrix)) return false;
//     if(!solver.data()->setGradient(gradient)) return false;
//     if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return false;
//     if(!solver.data()->setLowerBound(lowerBound)) return false;
//     if(!solver.data()->setUpperBound(upperBound)) return false;
//     if(!solver.initSolver()) return false;
//     if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
    
//     Eigen::VectorXd QPSolution;
//     QPSolution = solver.getSolution();
//     cbf_optimized_vel = formation_vel;
//     cbf_optimized_vel(0) = QPSolution(0);
//     cbf_optimized_vel(1) = QPSolution(1);

//     return true;
// }