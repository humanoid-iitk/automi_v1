// #include<vector>
#include<cmath>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
// #include<cv_bridge/cv_bridge.h>
// #include<opencv2/opencv.hpp>
// #include<opencv2/features2d.hpp>
#include<depth_stereo.hpp>
#include<ODG_PF.hpp>
// #include<iostream>
// #include<opencv2/ximgproc/disparity_filter.hpp>
// # pragma once

// #include <bits/stl_algo.h>
// #include <bits/stdc++.h>
// #include <numeric>
// #include <cassert>
// using namespace std;
// #define PI 3.1415926540

int main(int argc, char**argv){
    ros::init(argc, argv, "depth_stereo");
    ros::NodeHandle nh;

    huro::depth_generator gen(nh, "image1", "image2");
    int t = 0;
    while (ros::ok()){
        t += 1;
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
            if (t%30 == 0)
            {   
                for(int i=0; i<dist.size(); ++i)
                    std::cout << dist[i] << ' '; 
                std::cout << endl;
            }
            
            // float fin_ang = get_header_rad(dist, 0.698132, true);
            // std::cout << fin_ang*180/PI <<std::endl;
            cv::waitKey(1);
        }
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}

