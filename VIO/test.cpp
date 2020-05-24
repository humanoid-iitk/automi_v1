#include<visual_odometry.hpp>
#include<opencv2/opencv.hpp>
#include<vector>
#include<opencv2/features2d.hpp>
#include<iostream>

int main(){
    cv::Mat im, des1, des2, output;
    cv::namedWindow("input");
    cv::namedWindow("output");
    im = cv::imread("../test.jpeg", 0);
    std::vector<cv::KeyPoint> kp1, kp2;
    huro::extract_features(im, kp1, des1);
    huro::extract_features(im, kp2, des2);
    std::vector<std::vector<cv::DMatch>> matches = huro::match_features(des1, des2);
    std::vector<cv::DMatch> filtered = huro::filter_matches(matches);
    //cv::drawKeypoints(im, kp1, output);
    cv::Mat rmat, tvect;
    float data[9] = {0.8, 0, (float)(im.cols)/2, 0, 0.8, (float)(im.rows)/2, 0, 0, 1};
    cv::Mat K = cv::Mat(cv::Size(3,3), CV_32F, data);
    huro::estimate_motion(filtered, kp1, kp2, K, rmat, tvect);
    std::cout << rmat << std::endl;
    std::cout << tvect << std::endl;
    cv::drawMatches(im, kp1, im, kp2, filtered, output);
    cv::imshow("input", im);
    cv::imshow("output", output);
    cv::waitKey(0);
    return 0;
}