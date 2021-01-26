// int argMax = std::distance(x.begin(), std::max_element(x.begin(), x.end()));
# pragma once

#include <math.h>
#include <vector>
#include<opencv2/ximgproc/disparity_filter.hpp>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include <numeric>
#include <iostream>
// #include <matplotlibcpp.h>
using namespace std;
// namespace plt = matplotlibcpp;
#define PI 3.14159265

// constants
float Gamma =0.5;   //for attractive field to g (Had to change from gamma as that's a predefined var in mathcalls.h)
const float LC = 1; // Least count of angle for the sensor
const float MAX_DIST = 2; // Threshold Distance for the algo
const float WIDTH = 0.1;//Width of the robot

//sample input array containing distance values at 
vector<float> input[80];


//sample output array for storing potential field values
vector<float> field[80];

//will need more work
float index_to_angle(int i){
    return (PI*i)/180;
}

float angle_to_index(float a){
    return (a*180)/PI;
}

class obstacle
{
private:
    float d;    //average distance to each obstacle 
    float phi;  //the angle occupied by it
    // float sigma;    //half of the angle occupied by the obstacle considering for size of bot
    float theta;    //center angle for obstacle
    float A ;

public:
    obstacle(float d_in, float phi_in,float theta_in);  //constructor
    void increase_width(float w); // 
    void compute_field(vector<float>& field);   //compute and add field to the total field
    float get_theta(){ return theta;}
    float get_dist(){return d;}
    float get_phi(){ return phi;}
    bool inVic(){return (d<=0.1);};
};

std::vector<float> show_histogram(const std::string& name, const cv::Mat& image);
