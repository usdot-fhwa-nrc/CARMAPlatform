#ifndef UKF_H
#define UKF_H


#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
  //  bool is_initialized_;

    ///* state vector:
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Number of sigma points
    int n_sig_;

    ///* Sigma point spreading parameter
    double lambda_;

    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     *  Angle normalization to [-Pi, Pi]
     */
    void NormAng(double *ang);
    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a IMU measurement
     *
     */
    void UpdateIMU(VectorXd raw_imu);

    /**
     * Updates the state and the state covariance matrix using a GPS measurement
     *
     */
    void UpdateGPS(VectorXd raw_gps);
    /**
     * Updates the state and the state covariance matrix of the UKF
     *
     */
    void UpdateUKF(VectorXd raw, MatrixXd Zsig, int n_z);
};

#endif /* UKF_H */