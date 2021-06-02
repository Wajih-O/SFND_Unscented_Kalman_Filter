#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "ukf.h"
#include "Eigen/Dense"


using Eigen::MatrixXd;
using Eigen::VectorXd;


void normalize_angle_dim(VectorXd &input, int pos) {
    if (pos < input.size()){
      auto abs_pi_nbr = floor(abs(input(pos))/M_PI);
      if ( input(pos) > 0) {
        input(pos) -= abs_pi_nbr*M_PI;
      } else {
        input(pos) += abs_pi_nbr*M_PI;
      }
    }
}


std::string gen_nis_output_filename(std::string prefix, std::string sensor) {
    std::stringstream fileName;
    fileName << prefix << "_"<< sensor << ".csv";
    std::cout << fileName.str() << std::endl;
    return fileName.str();
}


void save_data(std::string output_path, std::vector<double> data) {
  std::ofstream output_file;
  // std::cout << "saving data into: " << output_path << std::endl;
  output_file.open(output_path);
  for (auto value: data) {
    output_file << value << std::endl;
  }
  output_file.close();
}


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  iter = 0; // an iteration limit to controle and debug iteration (to be removed)

  n_x_ = 5;
  n_aug_ = n_x_ + 2;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 10;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  /**
   * Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  lambda_ = 3 - n_aug_;
  is_initialized_ = false;

  P_ =  .1 * MatrixXd::Identity(n_x_, n_x_);

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  // Initialize weights  of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(.5/ (lambda_ + n_aug_));
  weights_(0) = lambda_/(lambda_ + n_aug_);

}

UKF::~UKF() {}

// void UKF::ProcessLidarMeasurement()

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // Lidar (Laser measurement)

  switch (meas_package.sensor_type_) {

    case (MeasurementPackage::SensorType::LASER) : {

      if (!is_initialized_) {
        std::cout << "Kalman Filter Initialization (from Lidar)" << std::endl;

        // set the state with the initial location and zero velocity
        x_ << meas_package.raw_measurements_[0],
              meas_package.raw_measurements_[1],
              0,
              0,
              0;

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
      }
      if (use_laser_){
        // std::cout << "Predict/update Lidar" << std::endl;
        // compute the time elapsed between the current and previous measurements
        // dt - expressed in seconds
        float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
        time_us_ = meas_package.timestamp_;
        Prediction(dt);
        UpdateLidar(meas_package);
      }
    }
    break;

  case (MeasurementPackage::SensorType::RADAR) :
  {
      if (!is_initialized_) {
        std::cout << "Kalman Filter Initialization (from RADAR measurement)" << std::endl;
        // Set the state with the initial location and zero velocity (From radar)
        x_ << meas_package.raw_measurements_[0]*cos(meas_package.raw_measurements_[1]),
              meas_package.raw_measurements_[0]*sin(meas_package.raw_measurements_[1]),
              meas_package.raw_measurements_[2],
              0,
              0;
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
      }
      else {
        if(use_radar_) {
          // std::cout << "Predict/update RADAR" << std::endl;

          // compute the time elapsed between the current and previous measurements
          // dt - expressed in seconds
          float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
          time_us_ = meas_package.timestamp_;
          Prediction(dt);
          UpdateRadar(meas_package);
        }
    }

  }
  break;
  }
}


void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; ++i) {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd const &Xsig_aug, double delta_t) {

  // create matrix with predicted sigma points as columns
  Xsig_pred_.fill(0.0);

  for (int col_idx=0; col_idx<Xsig_aug.cols(); col_idx++ ) {

    VectorXd col = Xsig_aug.col(col_idx);
    VectorXd output_col = Xsig_aug.col(col_idx).head(n_x_);

    output_col(0) += 0.5 * pow(delta_t, 2)*cos(col(3))*col(5);
    output_col(1) += 0.5 * pow(delta_t, 2)*sin(col(3))*col(5);
    output_col(2) += delta_t * col(5);
    output_col(3) += 0.5 * pow(delta_t, 2)*col(6);
    output_col(4) += delta_t * col(6);

    if (col(4) == 0) {
        // phi dot is zero (avoid division by zero)
        output_col[0] += delta_t*cos(col[3])*col[2];
        output_col[1] += delta_t*sin(col[3])*col[2];

    } else {
        double tmp = col(2)/col(4);
        output_col[0] += tmp * (sin(col(3) + (delta_t*col[4])) - sin(col(3)));
        output_col[1] += tmp * (-cos(col(3) + (delta_t*col[4])) + cos(col(3)));
    }
    output_col(3) += delta_t*col(4);

    Xsig_pred_.col(col_idx) = output_col.head(n_x_);

  }

}

void UKF::Prediction(double delta_t) {
  /**
   * Modify the state vector, x_. Predict sigma points, the state,
   * and the state covariance matrix.
   */

  // create sigma point matrix
  MatrixXd Xsig_aug;
  AugmentedSigmaPoints(&Xsig_aug);

  Xsig_pred_.fill(0.0);
  SigmaPointPrediction(Xsig_aug, delta_t);
  // std::cout << "Predicted Sigma points" << std::endl << Xsig_pred_;

  // state mean, and the state covariance matrix.
  // std::cout << "weights: " << weights_ << std::endl;

  // Predict state mean
  x_.fill(0.0);
  for (int col_idx=0; col_idx<Xsig_pred_.cols(); col_idx++ ) {
    x_ +=weights_(col_idx) * Xsig_pred_.col(col_idx);
  }

  // Predict state covariance matrix
  P_.fill(0.0);
  VectorXd centred = VectorXd(n_x_);
  for (int col_idx=0; col_idx<Xsig_pred_.cols(); col_idx++ ) {
        centred = (Xsig_pred_.col(col_idx) - x_);
        // check phi to stay within the -pi, pi
        normalize_angle_dim(centred, 3);
        P_ +=  weights_(col_idx)*centred*centred.transpose() ;
  }

  // std::cout << "updated x:" << std::endl << x_;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * Use lidar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  MatrixXd H_ = MatrixXd(2, 5);
  H_ << 1, 0, 0, 0, 0 ,
        0, 1, 0, 0, 0;
  // std::cout << H_.size() << std::endl;
  VectorXd z_pred = H_ * x_;
  VectorXd y = meas_package.raw_measurements_ - z_pred;
  MatrixXd Ht = H_.transpose();
  // measurement covariance
  auto R_ = MatrixXd(2, 2);
  R_ << pow(std_laspx_, 2), 0,
        0, pow(std_laspy_, 2);

  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

  // calculate and collect the lidar NIS
  lidarNISHist.push_back(y.transpose()*Si*y);

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * Use radar data to update the belief
   * about the object's position. Modify the state vector, x_, and
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

 // protect running with other SensorTypes than RADAR
 if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR) {
  // Predict Radar Measurement

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(meas_package.raw_measurements_.size(), 2 * n_aug_ + 1);
  VectorXd centred = VectorXd(meas_package.raw_measurements_.size());

  // transform sigma points into measurement space
  for (int col_idx=0; col_idx<Xsig_pred_.cols(); col_idx++ ) {
    VectorXd col = Xsig_pred_.col(col_idx);
    VectorXd z_col = VectorXd(3);

    z_col(0) = sqrt(pow(col(0), 2) + pow(col(1), 2)); // r
    z_col(1) = atan2(col(1), col(0)); // phi
    // Ensure phi between -pi and pi
    normalize_angle_dim(z_col, 1);
    z_col(2) = col(2)*(col(0) * cos(col(3)) + (col(1) * sin(col(3)))) / z_col(0); // r_dot
    Zsig.col(col_idx) = z_col;
  }

  // Predict state mean
  VectorXd z_pred = VectorXd(meas_package.raw_measurements_.size());
  z_pred.fill(0.0);

  for (int col_idx=0; col_idx<Zsig.cols(); col_idx++ ) {
         z_pred += Zsig.col(col_idx) * weights_(col_idx);
  }

  // Calculate innovation covariance matrix S
  MatrixXd S = MatrixXd(meas_package.raw_measurements_.size(), meas_package.raw_measurements_.size());
  S.fill(0.0);
  for (int col_idx=0; col_idx<Zsig.cols(); col_idx++ ) {
        centred = (Zsig.col(col_idx) - z_pred);
        normalize_angle_dim(centred, 1);
        S +=  weights_(col_idx)*centred*centred.transpose() ;
  }
  // S = S + R
  S(0, 0) += pow(std_radr_, 2);
  S(1, 1) += pow(std_radphi_, 2);
  S(2, 2) += pow(std_radrd_, 2);

  // Update state

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, meas_package.raw_measurements_.size());
  Tc.fill(0.0);

  // calculate cross correlation matrix
  VectorXd centred_x = VectorXd(n_x_);
  VectorXd centred_z = VectorXd(meas_package.raw_measurements_.size());

  for (int col_idx=0; col_idx<Xsig_pred_.cols(); col_idx++ ) {
        centred_x = (Xsig_pred_.col(col_idx) - x_);
        centred_z = (Zsig.col(col_idx) - z_pred);
        normalize_angle_dim(centred_z, 1);
        normalize_angle_dim(centred_x, 3);
        // Sanity check (test) for angle normalization !
        for (auto item : std::vector<std::pair<double, std::string>>{{centred_z(1), "z"}, {centred_x(3), "x"}}) {
          if (abs(item.first) >  M_PI)  {
            std::cout << " diverged angle! -> "  << item.second << std::endl;
            exit(1);
          }
        }
        Tc +=  weights_(col_idx)*centred_x*centred_z.transpose();
  }

  // calculate Kalman gain K;
  MatrixXd Si = S.inverse();
  MatrixXd K = Tc*Si;

  // update state mean and covariance matrix
  VectorXd z_res = (meas_package.raw_measurements_ - z_pred);
  // norm_angle(z_res, 1);
  x_ = x_ + K*z_res;
  P_ = P_ - K*S*K.transpose();

  // calculate and collect the radar NIS in radar NIS history
  radarNISHist.push_back(z_res.transpose() * S.inverse() * z_res);
 } else {
   std::cerr << "UpdateRadar called for a non RADR measurement" << std::endl;
 }

}


// Saving NIS data to file
  void UKF::saveNIS(std::string prefix) {
    // Save Radar NIS
    save_data(gen_nis_output_filename(prefix, "radar"), radarNISHist);
    // Save Lidar NIS
    save_data(gen_nis_output_filename(prefix, "lidar"), lidarNISHist);


  }