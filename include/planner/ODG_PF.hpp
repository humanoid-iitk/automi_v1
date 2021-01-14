// int argMax = std::distance(x.begin(), std::max_element(x.begin(), x.end()));
# pragma once

#include <math.h>
#include <vector>
#include <iostream>
// #include <matplotlibcpp.h>
using namespace std;
// namespace plt = matplotlibcpp;
#define PI 3.14159265

using namespace std;

// constants
float Gamma =0.5;   //for attractive field to g (Had to change from gamma as that's a predefined var in mathcalls.h)
const float LC = 1; // Least count of angle for the sensor
const float MAX_DIST = 2; // Threshold Distance for the algo
const float WIDTH = 0;//Width of the robot

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

void draw_circle(float cx, float cy, float rad){
    vector<float> x;
    vector<float> y;
    for(int i =0;i<360;i++){
        float a = index_to_angle(i);
        x.push_back(cx+rad*cos(a));
        y.push_back(cy+rad*sin(a));
    }
    // plt::plot(x, y);
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
    bool inVic();
};

obstacle::obstacle(float d_in, float phi_in,float theta_in)
{
    d = d_in;
    phi = phi_in;
    theta = theta_in;
    A = (MAX_DIST-d)*exp(0.5);
}

void obstacle::increase_width(float w){
    float phi_ = index_to_angle(phi);
    phi_ = 2*atan((tan(phi_/2)+ (w/2) )/d );
    phi_ = angle_to_index(phi_);
    this->phi = phi_;
}

void obstacle::compute_field(vector<float> &field)
{
    float theta = index_to_angle(this->theta);
    float ph = index_to_angle(this->phi);
    for(int i=0;i < field.size();i++){
        float angle = index_to_angle(i);
        double temp = A*exp(-1*((theta-angle)*(theta-angle))/(2*ph*ph));
        field[i] += temp;   
    }
}

bool obstacle::inVic(){
    return (d<=0.1);
}

float get_goal_angle(float x, float y, float goal_x, float goal_y){
    float a = atan((goal_y-y)/(goal_x-x));
    if(goal_y-y<0 && goal_x-x>0){
        a = 2*PI + a;
    }
    else if(goal_y-y<0 && goal_x-x<0){
        a = PI + a;
    }
    else if(goal_y-y>0 && goal_x-x<0){
        a = PI+ a;
    }
    return a; 
}

void goal_field(vector<float> &field, float goal_angle){
    for (int i = 0; i < field.size(); i++)
    {
        float angle = index_to_angle(i);
        auto temp = (Gamma)*abs(angle - goal_angle);
        field[i] += temp;
    }
    
}

class rect{
    float x1, x2, x3, x4;
    float y1, y2, y3, y4;
 public:
    rect(float x1_, float y1_, float x3_, float y3_){   
        x1 = x1_;
        x2 = x3_;
        x3 = x3_;
        x4 = x1_;
        y1 = y1_;
        y2 = y1_;
        y3 = y3_;
        y4 = y3_;
    }
    bool on_line(float x, float y);
    float get_min_dist(float x, float y, float theta);
    void draw();

};

bool rect::on_line(float x, float y){
    float cx = (x1+x2)/2;
    float cy = (y1+y4)/2;
    float rad = abs(x1-x2)/2;
    if((x-cx)*(x-cx)+ (y-cy)*(y-cy) <= rad*rad)
        return true;
    
    return false;
}

float rect::get_min_dist(float x, float y, float theta){
    float d = 0;
    if(this->on_line(x, y)){
        float cx = (this->x1+this->x2)/2;
        float cy = (this->y1+this->y4)/2;
        float atoc = get_goal_angle(x, y,cx, cy);
        return abs(theta-atoc);
    }
    for(d = 0; d < MAX_DIST; d+=0.001){
        float xi, yi;
        xi = x + d*cos(theta); 
        yi = y + d*sin(theta);

        if(this->on_line(xi, yi))
            break;

    }
    return d;
}

void rect::draw(){
    draw_circle((x1+x2)/2, (y1+y3)/2, abs(x2-x1)/2);
}