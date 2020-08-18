# pragma once

#include "ODG-PF.h"
#include <iostream>
#include <bits/stl_algo.h>
#include <bits/stdc++.h>
#include <numeric>
#include <cassert>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;




vector<obstacle> get_obstacles(vector<float> polar_dat){
    vector<obstacle> obs;
    int flag = 0; // represents whether an obs is being read
    float theta1, theta2, angular_width, mean_angle, dist, d;
    polar_dat[polar_dat.size()-1] = MAX_DIST;
    for(int i=0;i<polar_dat.size();i++){
        dist = polar_dat[i];

        if((flag == 0) and (dist<MAX_DIST)){// Start a new  obstacle
            flag = 1;
            theta1 = i;
        }

        else if((flag == 1) and (dist>=MAX_DIST)){
            flag = 0;
            theta2 = i-1;
            angular_width = theta2-theta1;
            mean_angle = (theta2+theta1)/2;
            d = accumulate(&polar_dat[theta1], &polar_dat[theta2], 0.0f) / angular_width;
            assert(angular_width >= 0);
            obs.push_back(obstacle(d, angular_width, mean_angle));
            theta1 = theta2;
        }         
    }
    
    return obs;
}

vector<obstacle> process_obs(vector<obstacle> &obs){
    vector<obstacle> obs_  = obs;
    for (int i = 0; i<obs_.size();i++){
        obs_[i].increase_width(WIDTH);
    }
    return obs_;
}

int get_best_header(vector<float> potential){
    int header = std::distance(potential.begin(), std::min_element(potential.begin(), potential.end()));
    return header;
}

vector<float> polar_data(360); // vector of size 360 : length of free path along each dir
vector<float> potential(360); // Final potential at each angle

float get_header_rad(vector<float> polardat, float goal_angle, bool showPlot){
    float header;
    
    auto obstacles = get_obstacles(polardat);
    // obstacles = process_obs(obstacles);
    // For debugging
    if(showPlot){
        for(auto a:obstacles){
            cout << "Obstacle is " << a.get_dist()<< "m away at angle "<<a.get_theta()<<" with width "<<a.get_phi()<<" \n";
        }
        cout << "Num obstacles = " << obstacles.size() << endl;
    }
    
    vector<float> potfield(360, 0);
    for(int i=0;i<obstacles.size();i++){
        obstacles[i].compute_field(potfield);
    }
    goal_field(potfield, goal_angle);
    header = get_best_header(potfield);

    return index_to_angle(header);
}