#include "Velocity_cbf.h"

int Velocity_cbf::ID = 0;

Velocity_cbf::Velocity_cbf(int mavNum)
{
    mav_num = mavNum;
    neighbor_num = mav_num - 2;
    
    gamma = 3;

    kappa_HF = 4;
    kappa_VF = 1;

    target_TD = 4;
    target_SD = 2;
    
    neighbor_CD = 8;
    neighbor_SD = 1.5;
}

Velocity_cbf::~Velocity_cbf(){}

void Velocity_cbf::setCBFparams(double G, double SD)
{
    gamma = G;
    target_SD = SD;
}

void Velocity_cbf::setMavsPosition(std::vector<MAV_eigen> mavs_eigen)
{
    Mavs_eigen = mavs_eigen;
    Mavs_eigen[ID].r_c = Mavs_eigen[ID].r + Mavs_eigen[ID].R_w2b.inverse()*cam.t_B2C();
}

Eigen::VectorXd Velocity_cbf::getOptimizedVel(){return cbf_optimized_vel;}

bool Velocity_cbf::CBF_init(int var_Num)
{
    varNum = var_Num;

    P.setIdentity(varNum, varNum);
    q.resize(varNum);

    A_HF.setZero(varNum);
    A_VF.setZero(varNum);
    A_tS_tCA.setZero(varNum);
    A_uS_uCA.setZero(neighbor_num, varNum);
    B_uS_u.setZero(neighbor_num);
    B_uCA_l.setZero(neighbor_num);

    constraintsNum = 3 + A_uS_uCA.rows();

    A.setZero(constraintsNum, varNum);
    lowerBound.setZero(constraintsNum);
    upperBound.setZero(constraintsNum);

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

    int j=0;
    for(int i=1; i<mav_num; i++)
    {
        if(i != ID)
        {
            A_uS_uCA.block(j, 0, 1, 3) = 2*(Mavs_eigen[i].r - Mavs_eigen[ID].r).transpose();
            B_uS_u(j) = gamma * (pow((Mavs_eigen[i].r - Mavs_eigen[ID].r).norm(), 2) - pow(neighbor_SD, 2))
                        + 2*(Mavs_eigen[i].r - Mavs_eigen[ID].r).dot(Mavs_eigen[i].v);
            B_uCA_l(j) = -gamma * (pow(neighbor_CD, 2) - pow((Mavs_eigen[j].r - Mavs_eigen[ID].r).norm(), 2))
                        + 2*(Mavs_eigen[i].r - Mavs_eigen[ID].r).dot(Mavs_eigen[i].v);
            j++;
        }
    }

    A_tS_tCA.segment(0, 3) = 2*(Mavs_eigen[0].r - Mavs_eigen[ID].r);
    B_tS_u = gamma * (pow((Mavs_eigen[0].r - Mavs_eigen[ID].r).norm(), 2) - pow(target_SD,2));
            // + 2*(Mavs_eigen[0].r - Mavs_eigen[ID].r).dot(Mavs_eigen[0].v);
    B_tCA_l = -gamma * (pow(target_TD,2) - pow((Mavs_eigen[0].r - Mavs_eigen[ID].r).norm(), 2));
            //+ 2*(Mavs_eigen[0].r - Mavs_eigen[ID].r).dot(Mavs_eigen[0].v);

    Eigen::MatrixXd R_W2C = cam.R_B2C()*Mavs_eigen[ID].R_w2b;
    Eigen::Vector3d r_tc_C = R_W2C*(Mavs_eigen[0].r - Mavs_eigen[ID].r_c);
    Eigen::MatrixXd r_x = skew(r_tc_C);
    double r_tc_C_xz = sqrt(r_tc_C(0)*r_tc_C(0) + r_tc_C(2)*r_tc_C(2));
    double err_FOV_H = atan2(cam.lx(), 2*cam.fx())*1/8 - atan2(abs(r_tc_C(0)), r_tc_C(2));
    double err_FOV_V = atan2(cam.ly(), 2*cam.fy())*1/2 - atan2(abs(r_tc_C(1)), r_tc_C_xz);

    a = -1/(1 + pow(abs(r_tc_C(0))/r_tc_C(2), 2));
    A_HF.segment(0, 3) = a*(-r_tc_C(0)/(abs(r_tc_C(0))*r_tc_C(2))*R_W2C.row(0)
                        + abs(r_tc_C(0))/pow(r_tc_C(2), 2)*R_W2C.row(2));
    A_HF.segment(3, 3) = a*(-r_tc_C(0)/(abs(r_tc_C(0))*r_tc_C(2))*r_x.row(0)
                        + abs(r_tc_C(0))/pow(r_tc_C(2), 2)*r_x.row(2));
    A_HF.segment(0, 3).setZero();
    B_HF_u = OsqpEigen::INFTY;
    B_HF_l = -kappa_HF*(err_FOV_H);
            // - a*(-r_tc_C(0)/(abs(r_tc_C(0))*r_tc_C(2))*R_W2C.row(0)
            // + abs(r_tc_C(0))/pow(r_tc_C(2), 2)*R_W2C.row(2))*Mavs_eigen[0].v;
    
    b = -1/(1 + pow(abs(r_tc_C(1))/r_tc_C_xz, 2));
    A_VF.segment(0, 3) = b*((r_tc_C(0)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(0)
                        + (r_tc_C(2)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(2)
                        - (r_tc_C(1)/(abs(r_tc_C(1))*r_tc_C_xz))*R_W2C.row(1));
    A_VF.segment(3, 3) = b*((r_tc_C(0)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*r_x.row(0)
                        + (r_tc_C(2)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*r_x.row(2)
                        - (r_tc_C(1)/(abs(r_tc_C(1))*r_tc_C_xz))*r_x.row(1));
    B_VF_u = OsqpEigen::INFTY;
    B_VF_l = -kappa_VF*(err_FOV_V);
            // - b*((r_tc_C(0)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(0)
            // + (r_tc_C(2)*abs(r_tc_C(1))/pow(r_tc_C_xz, 3))*R_W2C.row(2)
            // - (r_tc_C(1)/(abs(r_tc_C(1))*r_tc_C_xz))*R_W2C.row(1))* Mavs_eigen[0].v;

    A.block(0, 0, neighbor_num, varNum) = A_uS_uCA;
    A.block(neighbor_num, 0, 1, varNum) = A_tS_tCA.transpose();
    A.block(neighbor_num + 1, 0, 1, varNum) = A_HF.transpose();
    A.block(neighbor_num + 2, 0, 1, varNum) = A_VF.transpose();
    A_sparse = A.sparseView();

    upperBound.segment(0, neighbor_num) = B_uS_u;
    upperBound(neighbor_num) = B_tS_u;
    upperBound(neighbor_num+1) = B_HF_u;
    upperBound(neighbor_num+2) = B_VF_u;

    lowerBound.segment(0, neighbor_num) = B_uCA_l;
    lowerBound(neighbor_num) = B_tCA_l;
    lowerBound(neighbor_num+1) = B_HF_l;
    lowerBound(neighbor_num+2) = B_VF_l;
    
    solver.data()->clearHessianMatrix();
    solver.data()->clearLinearConstraintsMatrix();
    if(!solver.data()->setHessianMatrix(hessian_Matrix)) return false;
    if(!solver.data()->setGradient(q)) return false;
    if(!solver.data()->setLinearConstraintsMatrix(A_sparse)) return false;
    if(!solver.data()->setUpperBound(upperBound)) return false;
    if(!solver.data()->setLowerBound(lowerBound)) return false;
    
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