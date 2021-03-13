#pragma once

#include <eigen3/Eigen/Dense>
#include<vector>

double calc_potential(std::vector<float>& histogram , const Eigen::VectorXd& uref , const Eigen::VectorXd& x0);

double calc_effort(const Eigen::VectorXd& u);

double ref_error(const Eigen::VectorXd& uref , const Eigen::VectorXd& u);

double loss_func(std::vector<float>& histogram , const Eigen::VectorXd& x0,const Eigen::VectorXd& uref,const Eigen::VectorXd& u);



