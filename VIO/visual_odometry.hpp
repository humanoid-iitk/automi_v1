#pragma once

#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/calib3d.hpp>
#include<vector>

namespace huro{
    void extract_features(cv::InputArray image, std::vector<cv::KeyPoint>& kp, cv::Mat& des);

    std::vector<std::vector<cv::DMatch>> match_features(const cv::InputArray des1, const cv::InputArray des2);
    
    std::vector<cv::DMatch> filter_matches(const std::vector< std::vector<cv::DMatch>> matches, float threshold = 0.6);
    
    void estimate_motion(const std::vector<cv::DMatch>& matches, 
                         const std::vector<cv::KeyPoint>& kp1, 
                         const std::vector<cv::KeyPoint>& kp2,
                         const cv::InputArray K,                    //camera matrix
                         cv::Mat& rmat, cv::Mat&tvect);
}