#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<objective_func.hpp>

double time_step;
Eigen::DiagonalMatrix<double, 3> P(1, 1, 2);
Eigen::DiagonalMatrix<double, 3> Q(1, 2, 2);
double a1,a2,a3;

double calc_potential(const std::vector<float>& histogram , const Eigen::VectorXd& u , const Eigen::VectorXd& x0)
{
    return histogram[u[2]*time_step+x0[2]];
}

double calc_effort(const Eigen::VectorXd& u)
{
    return u.transpose()*P*u;
}

double ref_error(const Eigen::VectorXd& uref , const Eigen::VectorXd& u)
{
    Eigen::VectorXd uerr= uref-u;
    return uerr.transpose()*Q*uerr;
}

double loss_func(const std::vector<float>& histogram ,const Eigen::VectorXd& x0,const Eigen::VectorXd& uref,const Eigen::VectorXd& u)
{
    double loss= a1*calc_potential(histogram , u,x0) + a2*calc_effort(u) + a3*ref_error(uref,u);
    return loss;
}