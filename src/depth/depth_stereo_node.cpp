#include<vector>
#include<cmath>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<depth_stereo.hpp>
#include<iostream>
#include<opencv2/ximgproc/disparity_filter.hpp>
#define PI 3.141592654

void show_histogram(const std::string& name, const cv::Mat& image)
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

            show_histogram("hist", depth.row(depth.rows/2));
            cv::waitKey(1);
        }
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}

