#include"gradient.hpp"
#include<iostream>
#include<math.h>
#include<fstream>
#include<vector>

//namespace huro{
    //template <typename T>
        Optmizer::Optmizer(){
            //default values
            dim = 0;
            c_alpha = 0.0;
            setmaxiterr = 1;
            h = 0.001;
            setgradthresh = 1e-09;
        }

        Optmizer::~Optmizer(){

        }

        void Optmizer::set_cost_func(std::function<double(std::vector<double>*)> costfunction){
            c_costfunc = costfunction;
        }

        void Optmizer::X(const std::vector<double> cap_X){
            StartPoint = cap_X;
            dim = StartPoint.size();
        }

        void Optmizer::set_alpha(double alpha){
            c_alpha = alpha;
        }

        void Optmizer::max_iterr(int max_iterration){
            setmaxiterr = max_iterration;
        }

        void Optmizer::gradient_thresh(double gradThresh){
            setgradthresh = gradThresh;
        }

        bool Optmizer::optmise(std::vector<double> *func , double *funcvalue){
            CurrentPoint = StartPoint;
            int count = 0;
            double gradientMagnitude = 1.0;
            while((count < setmaxiterr) && (gradientMagnitude > setgradthresh)){
                std::vector<double> gradientVector = ComputeGradientVector();
                gradientMagnitude = GradientMagnitude(gradientVector);

                std::vector<double> newPoint = CurrentPoint;
                for(int i = 0; i < dim; i++){
                    newPoint[i] = newPoint[i] - (c_alpha * gradientVector[i]);
                }
                CurrentPoint = newPoint;
                count++;
            }

            *func = CurrentPoint;
            *funcvalue = c_costfunc(&CurrentPoint);
            return 0;
        }
//vec sample for vector
        double Optmizer::ComputeGradient(int Dim){
            std::vector<double> newpoint = CurrentPoint;
            newpoint[Dim] = newpoint[Dim] + h;

            double funcvalat_Xn = c_costfunc(&CurrentPoint);
            double funcvalat_Xn_plus_one = c_costfunc(&newpoint);

            return (funcvalat_Xn_plus_one - funcvalat_Xn) / h;
        }

        std::vector<double> Optmizer::ComputeGradientVector(){
            std::vector<double> gradientVec = CurrentPoint;
            for(int i = 0; i < dim; i++){
                gradientVec[i] = ComputeGradient(i);
            }
            return gradientVec;
        }

        double Optmizer::GradientMagnitude(std::vector<double> gradientvector){
            double grad_mag = 0.0;
            for(int i = 0; i < dim; i++){
                grad_mag = gradientvector[i]*gradientvector[i];
            }
        return sqrt(grad_mag);
        }
//}
