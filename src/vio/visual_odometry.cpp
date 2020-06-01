#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/calib3d.hpp>
#include<vector>
#include<string>

namespace huro{
    const int cx=320;
    const int cy=240;
    const float f = 1.398;
    void extract_features(cv::InputArray image, std::vector<cv::KeyPoint>& kp, cv::Mat& des){
        static cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> kp_temp;
        cv::Mat des_temp;
        orb->detectAndCompute(image, cv::Mat(), kp_temp, des_temp);
        kp = kp_temp;
        des = des_temp;
        return;
    }

    std::vector<std::vector<cv::DMatch>> match_features(const cv::InputArray des1, const cv::InputArray des2){
        static cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
        std::vector<std::vector< cv::DMatch >> matches;
        matcher->knnMatch(des1, des2, matches, 2);
        return matches;
    }

    std::vector<cv::DMatch> filter_matches(const std::vector< std::vector<cv::DMatch>> matches, float threshold = 0.6){
        std::vector<cv::DMatch> filtered_match;
        
        for (std::vector<cv::DMatch>match_pair: matches){
            cv::DMatch m = match_pair[0];
            cv::DMatch n = match_pair[1];
            if (m.distance < threshold*n.distance){
                filtered_match.push_back(m);
            }
        }
        return filtered_match;
    }

    std::vector<cv::Point3f> getObjectPoints(const std::vector<cv::Point2i>& image_points, const cv::Mat& depth){
        std::vector<cv::Point3f> obj_points;

        for (cv::Point2i pt: image_points){
            float Z = (depth.ptr<float>(pt.x))[pt.y];
            cv::Point3f pt_3(
                (pt.x-cx)*Z/f,
                (pt.y-cy)*Z/f, 
                Z
            );
            obj_points.push_back(pt_3);
        }

        return obj_points;
    }

    void estimate_motion(const std::vector<cv::DMatch>& matches, 
                         const std::vector<cv::KeyPoint>& kp1, 
                         const std::vector<cv::KeyPoint>& kp2,
                         const cv::InputArray K,                        //camera matrix
                         cv::Mat& rmat, cv::Mat&tvect,
                         const cv::Mat& depth1,
                         const cv::Mat& depth2){          
        
        std::vector<cv::Point2i> image1_points, image2_points;
        
        for (cv::DMatch match: matches){
            int train_idx = match.trainIdx;
            int query_idx = match.queryIdx;
            
            cv::Point2i p1 = kp1[query_idx].pt;
            image1_points.push_back(p1);
            cv::Point2i p2 = kp2[train_idx].pt;
            image2_points.push_back(p2);
        }

        cv::Mat rvect1, rvect2, tvect1, tvect2;
        std::vector<cv::Point3f> object_points1 = getObjectPoints(image1_points, depth1);
        std::vector<cv::Point3f> object_points2 = getObjectPoints(image2_points, depth2);
        cv::solvePnPRansac(object_points1, image1_points, K, cv::Mat(), rvect1, tvect1);
        cv::solvePnPRansac(object_points2, image2_points, K, cv::Mat(), rvect2, tvect2);

        cv::Mat RPNP1, RPNP2;
        cv::Rodrigues(rvect1, RPNP1);
        cv::Rodrigues(rvect2, RPNP2);

        rmat = RPNP1.inv() * RPNP2;
        tvect = RPNP2 * tvect2 - RPNP1 * tvect1;  
        // cv::Mat emat = cv::findEssentialMat(image1_points, image2_points, K);
        // cv::Mat rmat_temp, tvect_temp;
        // cv::recoverPose(emat, image1_points, image2_points, K, rmat_temp, tvect_temp);
        // rmat = rmat_temp;
        // tvect = tvect_temp;
        return;
    }    

}