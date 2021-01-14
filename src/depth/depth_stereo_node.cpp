#include<vector>
#include<cmath>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<depth_stereo.hpp>
#include<ODG_PF.hpp>
#include<iostream>
#include<opencv2/ximgproc/disparity_filter.hpp>
# pragma once

#include <bits/stl_algo.h>
#include <bits/stdc++.h>
#include <numeric>
#include <cassert>
using namespace std;
#define PI 3.141592654


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

vector<float> polar_data(80); // vector of size 360 : length of free path along each dir
vector<float> potential(80); // Final potential at each angle

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
    
    vector<float> potfield(80, 0);
    for(int i=0;i<obstacles.size();i++){
        obstacles[i].compute_field(potfield);
    }
    goal_field(potfield, goal_angle);
    header = get_best_header(potfield);

    return index_to_angle(header);
}


std::vector<float> show_histogram(const std::string& name, const cv::Mat& image)
{
    std::vector<float> v;
    int hist_h = 480, hist_w = 640;
    double focus = 381.362467;
    int cx_ = hist_w/2;
    int cy_ = hist_h/2;

    cv::Mat dist(image.rows, image.cols, CV_32F, cv::Scalar(0, 0, 0));
    cv::Mat angle(image.rows, image.cols, CV_32F, cv::Scalar(0, 0, 0));

    for(int iterr=0; iterr<image.rows; iterr++)
    {
        float dist_sum = 0;
        int count = 1;
        for(int iterc=0; iterc<image.cols; iterc++)
        {
            float X = (iterc - cx_)*image.at<float>(iterr, iterc)/focus;
            float Y = 0;
            float Z = image.at<float>(iterr, iterc);
            dist.at<float>(iterr, iterc) = cv::sqrt(X*X + Y*Y + Z*Z);
            angle.at<float>(iterr, iterc) = cvRound(atan(X/Z)*180/PI);
            int anglenow = angle.at<float>(iterr, iterc);
            int anglebef = angle.at<float>(iterr, iterc-1);
            
            if(iterc==0)
            {
                dist_sum = dist.at<float>(iterr, iterc);
                continue;
            }
            if(anglenow == anglebef)
            {
                dist_sum += dist.at<float>(iterr, iterc);
                count++;
            }
            else
            {
                dist_sum = dist_sum/count;
                v.push_back(dist_sum);
                dist_sum = dist.at<float>(iterr, iterc);
                count=1;
            }

        }
    }

    float min_el = *std::min_element(v.begin()+5, v.end());
    float max_el = *std::max_element(v.begin()+5, v.end());

    for (int i=0; i<v.size(); ++i)
    {
        if(v[i]>max_el)
            v[i] = min_el;
    } 
    // std::cout << *std::max_element(v.begin(), v.end()) << " ";

    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0,0,0));
    int bin_w = hist_w/v.size();
    for(int bin=0; bin<v.size(); bin++)
    {
        cv::line(histImage, 
                    cv::Point(bin_w*bin, cvRound(v[bin]*100)),
                    cv::Point(bin_w*bin, hist_h),
                    cv::Scalar(255, 255, 255));
    }

    cv::imshow(name, histImage);

    return v;
}

int main(int argc, char**argv){
    ros::init(argc, argv, "depth_stereo");
    ros::NodeHandle nh;

    huro::depth_generator gen(nh, "image1", "image2");

    while (ros::ok()){
        cv::Mat depth;      
        cv::Mat disparity;
        gen.calc_depth(depth, disparity);
        
        disparity = disparity*20;


        if (depth.rows > 0 && depth.cols > 0)
        {
            cv::Mat disparityvis;
            disparity.convertTo(disparity, CV_16S);
            cv::ximgproc::getDisparityVis(disparity, disparityvis, 6);
            cv::imshow("disparity", disparityvis);
            
            std::vector<float> dist;
            dist = show_histogram("hist", depth.row(depth.rows/2));

            float fin_ang = get_header_rad(dist, 0.698132, true);
            std::cout << fin_ang*180/PI <<std::endl;
            cv::waitKey(1);
        }
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}

