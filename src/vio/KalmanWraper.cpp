# include <iostream>
# include <Eigen/Geometry>
# include "kalman/kalman.cpp"
#include <iostream>
#include <Eigen/Dense>
// #include "kalman.hpp"

using namespace Eigen;
using namespace std;

class translation{
     public:
          int n = 6; // Number of states
          int m = 3; // Number of measurements
          int c = 3; // Number of control inputs

          Eigen::MatrixXd A; // System dynamics matrix
          Eigen::MatrixXd B; // Input control matrix
          Eigen::MatrixXd C; // Output matrix
          Eigen::MatrixXd Q; // Process noise covariance
          Eigen::MatrixXd R; // Measurement noise covariance
          Eigen::MatrixXd P; // Estimate error covariance

          translation(){
               int n = 6; // Number of states
               int m = 3; // Number of measurements
               int c = 3; // Number of control inputs

               Eigen::MatrixXd A_(n, n); // System dynamics matrix
               Eigen::MatrixXd B_(n, c); // Input control matrix
               Eigen::MatrixXd C_(m, n); // Output matrix
               Eigen::MatrixXd Q_(n, n); // Process noise covariance
               Eigen::MatrixXd R_(m, m); // Measurement noise covariance
               Eigen::MatrixXd P_(n, n); // Estimate error covariance

               double dt = 1.0/30;

               A_ << 1, 0, 0,dt, 0, 0,
                    0, 1, 0, 0,dt, 0,
                    0, 0, 1, 0, 0,dt,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 1;

               B_ << 0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    dt, 0, 0,
                    0,dt, 0,
                    0, 0,dt;

               C_ << 1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0;

               Q_ << 0.05, 0, 0, 0, 0, 0,
                    0, 0.05, 0, 0, 0, 0,
                    0, 0, 0.05, 0, 0, 0,
                    0, 0, 0, 0.05, 0, 0,
                    0, 0, 0, 0, 0.05, 0,
                    0, 0, 0, 0, 0, 0.05;

               //get from imu
               R_ << 0.05, 0, 0,
                    0, 0.05, 0,
                    0, 0, 0.05;

               P_ = Q;

               this->A = A_;
               this->B = B_;
               this->C = C_;
               this->Q = Q_;
               this->R = R_;
               this->P = P_;


          }
};


class rotation{
     public:
          int n = 3; // Number of states
          int m = 3; // Number of measurements
          int c = 3; // Number of control inputs

          Eigen::MatrixXd A; // System dynamics matrix
          Eigen::MatrixXd B; // Input control matrix
          Eigen::MatrixXd C; // Output matrix
          Eigen::MatrixXd Q; // Process noise covariance
          Eigen::MatrixXd R; // Measurement noise covariance
          Eigen::MatrixXd P; // Estimate error covariance

          rotation(){
               int n = 3; // Number of states
               int m = 3; // Number of measurements
               int c = 3; // Number of control inputs

               Eigen::MatrixXd A_(n, n); // System dynamics matrix
               Eigen::MatrixXd B_(n, c); // Input control matrix
               Eigen::MatrixXd C_(m, n); // Output matrix
               Eigen::MatrixXd Q_(n, n); // Process noise covariance
               Eigen::MatrixXd R_(m, m); // Measurement noise covariance
               Eigen::MatrixXd P_(n, n); // Estimate error covariance

               double dt = 1.0/30;

               A_ << 0, 0, 0,
                     0, 0, 0,
                     0, 0, 0;

               B_ << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;

               C_ << 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1;

               Q_ << 0.05, 0, 0,
                     0, 0.05, 0,
                     0, 0, 0.05;
                    
               //get from imu
               R_ << 0.05, 0, 0,
                    0, 0.05, 0,
                    0, 0, 0.05;

               P_ = Q;

               this->A = A_;
               this->B = B_;
               this->C = C_;
               this->Q = Q_;
               this->R = R_;
               this->P = P_;


          }

          // Vector3f quat2ypr(Quaterniond Q){
          //      return Q.normalized().toRotationMatrix().eulerAngles(0, 1, 2);
          // }
};

Matrix3d quat2mat(const Eigen::Quaterniond& q)
{
//     std::cout << "R=" << std::endl << q.normalized().toRotationMatrix() << std::endl;

    return q.normalized().toRotationMatrix();
}


auto mat2euler(Matrix3d mat){
     return mat.eulerAngles(0,1,2);
}


// int main(){

//      translation t = translation();
//      KalmanFilter kf_translation(t.A, t.B, t.C, t.Q, t.R, t.P);
//      kf_translation.init();


//      rotation r = rotation();
//      KalmanFilter kf_rotation(r.A, r.B, r.C, r.Q, r.R, r.P);
//      kf_rotation.init();

//      auto angle = M_PI / 4;
//      auto sinA = std::sin(angle / 2);
//      auto cosA = std::cos(angle / 2);

//      Eigen::Quaterniond q;
//      q.x() = 0 * sinA;
//      q.y() = 1 * sinA;
//      q.z() = 0 * sinA;
//      q.w() = cosA;  

//      // cout << "w:" << q.w() << " x:" <<q.x() << " y:" << q.y() << " " << q.z() << " " << endl;
//      // auto mat = mat2euler(outputAsMatrix(q));
     
//      // cout << mat <<endl;
    



//     return 0;

// }
