#include<iostream>
#include"GradDesend.hpp"
#include<functional>

//define cost function
double cost_function(std::vector<double> *func){
    double x = func->at(0);
    double y = func->at(1);
    double theta = func->at(2);

    return ((x*x) + (y*y) + (theta*theta));
}

int main(int argc, char**argv){
    std::function<double(std::vector<double>*)> ptr_func{cost_function};

    Optmizer optmiser;

    optmiser.set_cost_func(ptr_func);
    std::vector<double> startPoint = {1,1,1};
    optmiser.X(startPoint);
    optmiser.set_alpha(0.01);
    optmiser.max_iterr(50);

    double funcvalue;
    std::vector<double> func;
    optmiser.optmise(&func, &funcvalue);
    
    std::cout << "location :\n" << func[0] << "," << func[1] << "," << func[2] << std::endl;
    std::cout << "optmised_value :\n" << funcvalue <<std::endl;

    return 0;
}

