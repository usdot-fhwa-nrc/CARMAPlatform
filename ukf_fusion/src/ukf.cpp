#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <vector>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    //is_initialized_ = false;
    // initial state vector
    x_ = VectorXd(9);
    // initial covariance matrix
    P_ = MatrixXd(9, 9);
    // State dimension
    n_x_ = x_.size();
    // Augmented state dimension
    n_aug_ = n_x_ + 2; // Create 2 * n_aug_ + 1 sigma points
    // Number of sigma points
    n_sig_ = 2 * n_aug_ + 1;
    // Set the predicted sigma points matrix dimentions
    Xsig_pred_ = MatrixXd(n_x_, n_sig_);
    // Sigma point spreading parameter
    lambda_ = 3 - n_aug_;
    // Weights of sigma points
    weights_ = VectorXd(n_sig_);
       }

UKF::~UKF() {}

/**
 *  Angle normalization to [-Pi, Pi]
 */
void UKF::NormAng(double *ang) {
    while (*ang > M_PI) *ang -= 2. * M_PI;
    while (*ang < -M_PI) *ang += 2. * M_PI;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    double delta_t2 = delta_t*delta_t;
    // Augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    // Augmented state covarience matrix
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    // Sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
    // Fill the matrices
    x_aug.fill(0.0);
    x_aug.head(n_x_) = x_;
    P_aug.fill(0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(9,9) = 1;
    P_aug(10,10) =1;
    // Square root of P matrix
    MatrixXd L = P_aug.llt().matrixL();
    // Create sigma points
    Xsig_aug.col(0) = x_aug;
    double sqrt_lambda_n_aug = sqrt(lambda_+n_aug_); // Save some computations
    VectorXd sqrt_lambda_n_aug_L;
    for(int i = 0; i < n_aug_; i++) {
        sqrt_lambda_n_aug_L = sqrt_lambda_n_aug * L.col(i);
        Xsig_aug.col(i+1)        = x_aug + sqrt_lambda_n_aug_L;
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt_lambda_n_aug_L;
    }

    // Predict sigma points
   
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }
    // Predicted state mean
    x_ = Xsig_pred_ * weights_; // vectorised sum
    // Predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization
        NormAng(&(x_diff(3)));
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
 * Updates the state and the state covariance matrix using a IMU measurement.
 *
 */
void UKF::UpdateIMU(VectorXd raw_imu) {
    // Set measurement dimension
    int n_z = 3;
    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    // Transform sigma points into measurement space
    UpdateUKF(raw_imu, Zsig, n_z);
}


/**
 * Updates the state and the state covariance matrix using a GPS measurement.
 *
 */
void UKF::UpdateGPS(VectorXd raw_gps) {
    // Set measurement dimension
    int n_z = 3;
    // Create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    // Transform sigma points into measurement space
    UpdateUKF(raw_gps, Zsig, n_z);
}

// Universal update function
void UKF::UpdateUKF(VectorXd raw_data, MatrixXd Zsig, int n_z){
    // Mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred  = Zsig * weights_;
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {
        // Residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // Angle normalization
        NormAng(&(z_diff(1)));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }
    // Add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z, n_z);

    S = S + R;

    // Create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    // Calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // State difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // Angle normalization
        NormAng(&(x_diff(3)));
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    // Measurements
    VectorXd z = raw_data;
    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();
    // Residual
    VectorXd z_diff = z - z_pred;
    // Update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

}
