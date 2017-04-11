#include "kalman_filter.h"
#include <iostream>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;


}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
 // std::cout<<"initial estimate"<<x_<<"\n";
  //new estimate
  x_ = x_ + (K * y);
  //std::cout<<"z_pred"<<z_pred<<"\n";
 // std::cout<<"z"<<z<<"\n";
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    float pho = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    float theta =  atan2(x_[1],x_[0]);
    while(theta < -M_PI) {
        theta += 2 * M_PI;
    }
    while(theta > M_PI) {
        theta -= 2 * M_PI;
    }
   // std::cout<<"theta"<<theta<<"\n";
    float pho_hat = (x_[0]*x_[2] + x_[1]*x_[3]) / pho;
    VectorXd z_pred = VectorXd(3);
    z_pred << pho, theta, pho_hat;

    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
