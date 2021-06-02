#ifndef UKF_H
#define UKF_H

#include <vector>
#include <string>
#include "Eigen/Dense"
#include "measurement_package.h"


void normalize_angle_dim(Eigen::VectorXd &input, int dim);

// A helper to generate output filename for nis
std::string gen_nis_output_filename(std::string prefix, std::string sensor);

// A helper to save the nis data into a file name
void save_data(std::string output_path, std::vector<double> data);


class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);
  void AugmentedSigmaPoints(Eigen::MatrixXd* Xsig_out);
  void SigmaPointPrediction(Eigen::MatrixXd const &Xsig_aug, double delta_t);
  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  // Saving NIS data to file
  void saveNIS(std::string prefix);


  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;
  long previous_timestamp_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us (previous/last timestamp)
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  int iter;

  std::vector<double> lidarNISHist;
  std::vector<double> radarNISHist;
};

#endif  // UKF_H