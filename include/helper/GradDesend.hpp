// #pragma once

// #include<vector>
// #include<functional>

// //namespace huro{
//     //template<class T>
//         class Optmizer{
//             public:
//                 Optmizer(); //constructor
            
//                 //for cost function
//                 void set_cost_func(std::function<double(std::vector<double>*)> costfunction); 
//                 //to capital X
//                 void X(const std::vector<double> cap_X);
//                 //for step_size
//                 void set_alpha(double alpha);
//                 //for max_iterration
//                 void max_iterr(int max_iterration);
//                 //to set threshold
//                 void gradient_thresh(double gradThresh);
//                 //for optimizing
//                 bool optmise(std::vector<double> *func , double *funcvalue);
            
//             private:
//                 //to estimate gradient
//                 double ComputeGradient(int Dim);
//                 //to compute gradient vector
//                 std::vector<double> ComputeGradientVector();
//                 //to compute gradient magnitude
//                 double GradientMagnitude(std::vector<double> gradientvector);

//                 int dim;
//                 double c_alpha, h;
//                 int setmaxiterr;
//                 double setgradthresh;
//                 std::vector<double> StartPoint;
//                 std::vector<double> CurrentPoint;
//                 std::function<double(std::vector<double>*)> c_costfunc;
//             };
// //}


#include<vector>
#include<functional>
#include<eigen3/Eigen/Core>

class GradDesent {
public:
    GradDesent(); //constructor
    GradDesent(std::function<double(Eigen::VectorXd X)> costfunction, const Eigen::VectorXd cap_X); //constructor

    void set_cost_func(std::function<double(Eigen::VectorXd X)> costfunction);    //for cost function

    void set_init(const Eigen::VectorXd cap_X); // set the initial point

    Eigen::VectorXd Gradient(Eigen::VectorXd point, Eigen::VectorXd direction); //compute gradient
    Eigen::VectorXd Gradient(Eigen::VectorXd point); //compute gradient

    //for step_size
    void set_alpha(double alpha);

    //for max_iterration
    void max_iterr(int max_iterration);

    //to set threshold
    void gradient_thresh(double gradThresh);

    //to se momentum
    void set_momentum(double momentum);
    
    //for optimizing
    void optmise();

    
    int dim;
    double c_alpha, h, momentum;
    int setmaxiterr;
    double setgradthresh;
    Eigen::VectorXd StartPoint;
    Eigen::VectorXd CurrentPoint;
    Eigen::VectorXd PrvGrad;
    std::function<double(Eigen::VectorXd)> Costfunc;
};