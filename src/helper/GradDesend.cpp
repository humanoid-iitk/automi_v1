// #include"gradient.hpp"
// #include<iostream>
// #include<math.h>
// #include<fstream>
// #include<vector>

// //namespace huro{
//     //template <typename T>
//         Optmizer::Optmizer(){
//             //default values
//             dim = 0;
//             c_alpha = 0.0;
//             setmaxiterr = 1;
//             h = 0.001;
//             setgradthresh = 1e-09;
//         }

//         Optmizer::~Optmizer(){

//         }

//         void Optmizer::set_cost_func(std::function<double(std::vector<double>*)> costfunction){
//             c_costfunc = costfunction;
//         }

//         void Optmizer::X(const std::vector<double> cap_X){
//             StartPoint = cap_X;
//             dim = StartPoint.size();
//         }

//         void Optmizer::set_alpha(double alpha){
//             c_alpha = alpha;
//         }

//         void Optmizer::max_iterr(int max_iterration){
//             setmaxiterr = max_iterration;
//         }

//         void Optmizer::gradient_thresh(double gradThresh){
//             setgradthresh = gradThresh;
//         }

//         bool Optmizer::optmise(std::vector<double> *func , double *funcvalue){
//             CurrentPoint = StartPoint;
//             int count = 0;
//             double gradientMagnitude = 1.0;
//             while((count < setmaxiterr) && (gradientMagnitude > setgradthresh)){
//                 std::vector<double> gradientVector = ComputeGradientVector();
//                 gradientMagnitude = GradientMagnitude(gradientVector);

//                 std::vector<double> newPoint = CurrentPoint;
//                 for(int i = 0; i < dim; i++){
//                     newPoint[i] = newPoint[i] - (c_alpha * gradientVector[i]);
//                 }
//                 CurrentPoint = newPoint;
//                 count++;
//             }

//             *func = CurrentPoint;
//             *funcvalue = c_costfunc(&CurrentPoint);
//             return 0;
//         }
// //vec sample for vector
//         double Optmizer::ComputeGradient(int Dim){
//             std::vector<double> newpoint = CurrentPoint;
//             newpoint[Dim] = newpoint[Dim] + h;

//             double funcvalat_Xn = c_costfunc(&CurrentPoint);
//             double funcvalat_Xn_plus_one = c_costfunc(&newpoint);

//             return (funcvalat_Xn_plus_one - funcvalat_Xn) / h;
//         }

//         std::vector<double> Optmizer::ComputeGradientVector(){
//             std::vector<double> gradientVec = CurrentPoint;
//             for(int i = 0; i < dim; i++){
//                 gradientVec[i] = ComputeGradient(i);
//             }
//             return gradientVec;
//         }

//         double Optmizer::GradientMagnitude(std::vector<double> gradientvector){
//             double grad_mag = 0.0;
//             for(int i = 0; i < dim; i++){
//                 grad_mag = gradientvector[i]*gradientvector[i];
//             }
//         return sqrt(grad_mag);
//         }
// //}

#include"GradDesend.hpp"
#include<iostream>

GradDesent::GradDesent() {
    //default values
    dim = 0;
    c_alpha = 0.1;
    setmaxiterr = 100;
    h = 0.001;
    setgradthresh = 1e-03;
    momentum = 0;
}

GradDesent::GradDesent(std::function<double(Eigen::VectorXd X)> costfunction, const Eigen::VectorXd cap_X) {
    //default values
    dim = 0;
    c_alpha = 0.1;
    setmaxiterr = 100;
    h = 0.001;
    setgradthresh = 1e-03;
    set_cost_func(costfunction);
    set_init(cap_X);
    momentum = 0;
}

void GradDesent::set_cost_func(std::function<double(Eigen::VectorXd X)> costfunction) {
    Costfunc = costfunction;
}

void GradDesent::set_init(const Eigen::VectorXd cap_X) {
    StartPoint = cap_X;
    dim = StartPoint.size();
}

Eigen::VectorXd GradDesent::Gradient(Eigen::VectorXd point, Eigen::VectorXd direction) {
    //return direction;
    auto unit = direction;
    unit.normalize();
    return unit*(   Costfunc( point+(unit * h) )  -  Costfunc( point - (unit * h) )   )/(2*h);
}

Eigen::VectorXd GradDesent::Gradient(Eigen::VectorXd point) {
    //return direction;
    Eigen::VectorXd unit = Eigen::VectorXd::Random(dim);
    unit.normalize();
    return unit * (Costfunc(point + (unit * h)) - Costfunc(point - (unit * h))) / (2 * h);
}

void GradDesent::set_alpha(double alpha) {
    c_alpha = alpha;
}

void GradDesent::max_iterr(int max_iterration) {
    setmaxiterr = max_iterration;
}

void GradDesent::gradient_thresh(double gradThresh) {
    setgradthresh = gradThresh;
}

void GradDesent::set_momentum(double mome) {
    momentum = mome;
}

void GradDesent::optmise() {
    PrvGrad = Eigen::VectorXd::Zero(dim);
    CurrentPoint = StartPoint;
    int count = 0;
    double gradientMagnitude = 1.0;
    while ((count < setmaxiterr) && (gradientMagnitude > setgradthresh)) {
        Eigen::VectorXd gradientVector = Gradient(CurrentPoint);
        gradientVector = gradientVector * (1 - momentum) + PrvGrad * (momentum);
        gradientMagnitude = gradientVector.norm();
        //std::cout << "ittration = " << count << "   grad_Mag = " << gradientMagnitude << "   cost = " << Costfunc(CurrentPoint)   << "   point = " << std::endl << CurrentPoint << std::endl << "Gradient = " << std::endl << gradientVector << std::endl;
        std::cout << "ittration =" << count << "   cost = " << Costfunc(CurrentPoint) << std::endl;
        std::cout << "gradientMag... = " << gradientMagnitude << std::endl;
        std::cout << "Currentpoint... = " << CurrentPoint.transpose() << std::endl;
        std::cout << "gradientVec... = " << gradientVector.transpose() << std::endl;

        CurrentPoint = CurrentPoint - (gradientVector * c_alpha);

        //std::vector<double> newPoint = CurrentPoint;
        //for (int i = 0; i < dim; i++) {
        //    newPoint[i] = newPoint[i] - (c_alpha * gradientVector[i]);
        //}
        //CurrentPoint = newPoint;
        count++;
    }

    //*func = CurrentPoint;
    //*funcvalue = c_costfunc(&CurrentPoint);
}