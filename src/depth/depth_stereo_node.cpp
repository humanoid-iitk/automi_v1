#include<vector>
#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<depth_stereo.hpp>
#include<iostream>
#include<opencv2/ximgproc/disparity_filter.hpp>

void show_histogram(std::string const& name, cv::Mat const& image)
{
    int bins = 640;
    int histSize[] = {bins};

    float range[] = {0, 256};
    const float* histRange = {range};
    bool uniform=true, accumulate=false;
    cv::Mat hist =  image;
    
    int hist_h = 480, hist_w = 640;
    int bin_w = cvRound(hist_w/bins);
    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0,0,0));

    //cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_L2, -1, cv::Mat());
    hist = 256 - hist;
    double max_val;
    cv::minMaxLoc(hist, 0, &max_val);
    //std::cout << cv::sum(hist) <<std::endl;
    for(int b=0; b<bins; b+=2)
    {
        float binval = hist.at<short>(1, b);
        //int height = cvRound(binval);
        int height = binval;
        std::cout << binval << std::endl;
        //std::cout << height << std::endl;
        cv::line(histImage, 
                    cv::Point(bin_w*b, hist_h-height),
                    cv::Point(bin_w*b, hist_h),
                    cv::Scalar(255, 255, 255));
    }

    cv::imshow(name, histImage);

/*
    // Set histogram bins count
    int bins = 480;
    int histSize[] = {bins};
    // Set ranges for histogram bins
    float lranges[] = {0, 256};
    const float* ranges[] = {lranges};
    // create matrix for histogram
    cv::Mat hist;
    int channels[] = {0};

    // create matrix for histogram visualization
    int const hist_height = 640;
    cv::Mat3b hist_image = cv::Mat3b::zeros(hist_height, bins);

    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);

    double max_val;
    minMaxLoc(hist, 0, &max_val);

    // visualize each bin
    for(int b = 0; b < bins; b++) {
        float const binVal = hist.at<float>(b);
        int   const height = cvRound(binVal*hist_height/max_val);
        cv::line
            ( hist_image
            , cv::Point(b, hist_height-height), cv::Point(b, hist_height)
            , cv::Scalar::all(255)
            );
    }
    cv::imshow(name, hist_image);*/
}

int main(int argc, char**argv){
    ros::init(argc, argv, "depth_stereo");
    ros::NodeHandle nh;

    huro::depth_generator gen(nh, "image1", "image2");

    while (ros::ok()){
        cv::Mat depth;      
        cv::Mat disparity;
        gen.calc_depth(depth, disparity);
        //cv::imshow("depth original", disparity);
        disparity = disparity*1000;
        disparity.convertTo(depth, CV_16S);

        if (depth.rows > 0 && depth.cols > 0)
         {
            cv::Mat deptht;
            cv::ximgproc::getDisparityVis(depth, deptht, 6);
            
            cv::imshow("depth", depth);
            //cv::imshow("disparity", disparity);
            double min, max;
            depth = depth/100;
            cv::minMaxLoc(depth, &min, &max);
            //std::cout << min << " "<< max;
            //std::cout << std::endl;
            show_histogram("hist", depth.row(depth.rows/2));
            //std::cout << depth.row(depth.rows/2).cols << std::endl;
            cv::waitKey(1);
        }
        //std::cout << depth << std::endl;
        ros::spinOnce();
    }

    ros::spin();
    return 0;
}

