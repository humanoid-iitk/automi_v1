#include<ODG_PF.hpp>

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
    
    for (int i=0; i<v.size()-5; ++i)
    {
        if(v[i]>max_el)
            v[i] = v[i+5];
    } 
    
    // std::cout << *std::max_element(v.begin(), v.end()) << " ";

    cv::Mat histImage2(hist_h, hist_w, CV_8UC1, cv::Scalar(0,0,0));
    int bin_w2 = hist_w/v.size();
    for(int bin=0; bin<v.size(); bin++)
    {
        cv::line(histImage2, 
                    cv::Point(bin_w2*bin, cvRound(v[bin]*100)),
                    cv::Point(bin_w2*bin, hist_h),
                    cv::Scalar(255, 255, 255));
    }

    cv::imshow("dist", histImage2);

    auto obstacles = get_obstacles(v);
    obstacles = process_obs(obstacles);

    std::vector<float> potfield(80, 0);
    for(int i=0;i<obstacles.size();i++){
        obstacles[i].compute_field(potfield);
    }

    // std::cout<< "v :"<< v.size() << endl;
    cv::Mat histImage(hist_h, hist_w, CV_8UC1, cv::Scalar(0,0,0));
    int bin_w = hist_w/potfield.size();
    for(int bin=0; bin<potfield.size(); bin++)
    {
        cv::line(histImage, 
                    cv::Point(bin_w*bin, cvRound(potfield[bin]*100)),
                    cv::Point(bin_w*bin, hist_h),
                    cv::Scalar(255, 255, 255));
    }

    cv::imshow(name, histImage);
    return v;
}