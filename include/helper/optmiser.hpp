#pragma once

#include<vector>
#include<functional>

//namespace huro{
    //template<class T>
        class Optmizer{
            public:
                Optmizer(); //constructor
            
                //for cost function
                void set_cost_func(std::function<double(std::vector<double>*)> costfunction); 
                //to capital X
                void X(const std::vector<double> cap_X);
                //for step_size
                void set_alpha(double alpha);
                //for max_iterration
                void max_iterr(int max_iterration);
                //to set threshold
                void gradient_thresh(double gradThresh);
                //for optimizing
                bool optmise(std::vector<double> *func , double *funcvalue);
            
            private:
                //to estimate gradient
                double ComputeGradient(int Dim);
                //to compute gradient vector
                std::vector<double> ComputeGradientVector();
                //to compute gradient magnitude
                double GradientMagnitude(std::vector<double> gradientvector);

                int dim;
                double c_alpha, h;
                int setmaxiterr;
                double setgradthresh;
                std::vector<double> StartPoint;
                std::vector<double> CurrentPoint;
                std::function<double(std::vector<double>*)> c_costfunc;
            };
//}