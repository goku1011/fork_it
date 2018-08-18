// POINTS INTERPOLATION AND UPDATE WITH NDT POSE LKF3
#include "integrated_localizer.h"
#include "unsupported/Eigen/MatrixFunctions"
//#include <polyx_nodea/CorrectedIMU.h>
#include <integrated_localizer/CorrectedIMU.h>

// The state vector is a 16*1 vector // 3 - x,y,z // 3 - vx, vy, vz // 4 - qx, qy, qz, qw // 3 - a_bias // 3 - w_bias
//



// --------------------------------------------------------------------------------------------------------
// Initializes main localizer with parameters
IntegratedLocalizer::IntegratedLocalizer(
    const std::vector<double>& init_state_cov,
    const std::vector<double>& state_noise,
    const std::vector<double>& gps_noise,
    const std::vector<double>& lidar_noise,
    double alpha,
    double beta,
    double kappa,
    float attenuation_coefficient,
    int is_rad_per_sec,
    const std::vector<double>& lidar_to_imu,
    const std::vector<double>& camera_to_imu,
    double TAI_offset) {
  set_cov(init_state_cov, state_noise, gps_noise, lidar_noise);
  alpha_ = alpha;
  beta_ = beta;
  kappa_ = kappa;
  attenuation_coefficient_ = attenuation_coefficient;
  is_rad_per_sec_ = is_rad_per_sec;
  TAI_offset_ = TAI_offset;
  gravity_ << 0.0, 0.0, -9.81;
  prev_lidar_pose_timestamp_ = 0.0;
  prev_gps_pose_timestamp_ = 0.0;
  prev_gnss_x = 0.0;
  prev_gnss_y = 0.0;
  ros::Time prev_imu_timestamp = ros::Time::now();
  std::cout<< prev_imu_timestamp<<std::endl;
  compute_weights();
  set_transform_lidar_imu(lidar_to_imu);
  set_transform_camera_imu(camera_to_imu);


// The H_ matrix is defined in such a way
// H_ = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
//       0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0
//       0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0
//       0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0
//       0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0
//       0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0
//       0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]


  H_ = Eigen::Matrix<double, 7, 16>::Zero();
  H_(0,0) = 1;
  H_(1,1) = 1;
  H_(2,2) = 1;
  H_(3,6) = 1;
  H_(4,7) = 1;
  H_(5,8) = 1;
  H_(6,9) = 1;

  A_matrix_ = Eigen::Matrix<double, 16, 16>::Zero();
  for (int i = 0; i < 16; i++){
    A_matrix_(i, i) = 1;
  }
}

// --------------------------------------------------------------------------------------------------------
// Sets transformation parameters between lidar and imu
void IntegratedLocalizer::set_transform_lidar_imu(
    const std::vector<double>& lidar_to_imu) {
  Eigen::Matrix3d lidar_to_imu_ori;
  lidar_to_imu_ori(0, 0) = lidar_to_imu[0];
  lidar_to_imu_ori(0, 1) = lidar_to_imu[1];
  lidar_to_imu_ori(0, 2) = lidar_to_imu[2];
  lidar_to_imu_ori(1, 0) = lidar_to_imu[4];
  lidar_to_imu_ori(1, 1) = lidar_to_imu[5];
  lidar_to_imu_ori(1, 2) = lidar_to_imu[6];
  lidar_to_imu_ori(2, 0) = lidar_to_imu[8];
  lidar_to_imu_ori(2, 1) = lidar_to_imu[9];
  lidar_to_imu_ori(2, 2) = lidar_to_imu[10];

  Eigen::Vector3d lidar_to_imu_pos;
  lidar_to_imu_pos(0) = lidar_to_imu[3];
  lidar_to_imu_pos(1) = lidar_to_imu[7];
  lidar_to_imu_pos(2) = lidar_to_imu[11];

  lidar_to_imu_.ori = lidar_to_imu_ori;
  lidar_to_imu_.pos = lidar_to_imu_pos;

  imu_to_lidar_.ori = lidar_to_imu_ori.transpose();
  imu_to_lidar_.pos = - lidar_to_imu_ori.transpose() * lidar_to_imu_pos;
}

// --------------------------------------------------------------------------------------------------------
// Sets transformation parameters between camera and imu
void IntegratedLocalizer::set_transform_camera_imu(
    const std::vector<double>& camera_to_imu) {
  Eigen::Matrix3d camera_to_imu_ori;
  camera_to_imu_ori(0, 0) = camera_to_imu[0];
  camera_to_imu_ori(0, 1) = camera_to_imu[1];
  camera_to_imu_ori(0, 2) = camera_to_imu[2];
  camera_to_imu_ori(1, 0) = camera_to_imu[4];
  camera_to_imu_ori(1, 1) = camera_to_imu[5];
  camera_to_imu_ori(1, 2) = camera_to_imu[6];
  camera_to_imu_ori(2, 0) = camera_to_imu[8];
  camera_to_imu_ori(2, 1) = camera_to_imu[9];
  camera_to_imu_ori(2, 2) = camera_to_imu[10];

  Eigen::Vector3d camera_to_imu_pos;
  camera_to_imu_pos(0) = camera_to_imu[3];
  camera_to_imu_pos(1) = camera_to_imu[7];
  camera_to_imu_pos(2) = camera_to_imu[11];

  camera_to_imu_.ori = camera_to_imu_ori;
  camera_to_imu_.pos = camera_to_imu_pos;

  imu_to_camera_.ori = camera_to_imu_ori.transpose();
  imu_to_camera_.pos = - camera_to_imu_ori.transpose() * camera_to_imu_pos;
}

// --------------------------------------------------------------------------------------------------------

// Computes UKF weights
void IntegratedLocalizer::compute_weights() {
  lambda_ = alpha_ * alpha_ * (15 + kappa_) - 15;
  mean_weights_.push_back(lambda_ / (15 + lambda_));
  cov_weights_.push_back(lambda_ / (15 + lambda_) + (1 - alpha_ * alpha_ + beta_));

  for (int i = 0; i < 30; ++i) {
    mean_weights_.push_back(1 / (2 * (15 + lambda_)));
    cov_weights_.push_back(1 / (2 * (15 + lambda_)));
  }
}

// --------------------------------------------------------------------------------------------------------
// Sets covariances for state and measurement noises
void IntegratedLocalizer::set_cov(const std::vector<double>& init_state_cov,
                                  const std::vector<double>& state_noise,
				                          const std::vector<double>& gps_noise,
				                          const std::vector<double>& lidar_noise) {
  state_cov_ = Eigen::Matrix<double, 16, 16>::Zero();
  state_noise_ = Eigen::Matrix<double, 16, 16>::Zero();
  for (int i = 0; i < init_state_cov.size(); ++i) {
    state_cov_(i, i) = init_state_cov[i];
    state_noise_(i, i) = state_noise[i];
  }

  gps_noise_ = Eigen::Matrix<double, 7, 7>::Zero();
  lidar_noise_ = Eigen::Matrix<double, 7, 7>::Zero();
  for (int i = 0; i < lidar_noise.size(); ++i) {
    gps_noise_(i, i) = gps_noise[i];
    lidar_noise_(i, i) = lidar_noise[i];
  }
}

// --------------------------------------------------------------------------------------------------------
// Checks if the state history size is larger than limit
void IntegratedLocalizer::check_state_history_size() {
  if (state_history_.size() > state_history_size_limit_) {
    state_history_.pop_front();
  }
}

// --------------------------------------------------------------------------------------------------------
// Checks if the imu data buffer size is larger than limit
void IntegratedLocalizer::check_imu_data_size() {
  if (imu_data_.size() > imu_data_size_limit_) {
    imu_data_.pop_front();
    is_imu_data_pop_front_ = true;
  }
}


// --------------------------------------------------------------------------------------------------------
// State propagation by UKF using imu data integration
void IntegratedLocalizer::state_propagation() {
  // Creates sigma points
  //create_sigma_points();

  // Propagates sigma points
  //propagate_sigma_points();

  // Computes state prediction from propagated sigma points
  compute_pred_state();

}

// --------------------------------------------------------------------------------------------------------
// Computes delta rotation from dt, angular velocity, and bias by simple 1st
// order integration
Eigen::Matrix3d IntegratedLocalizer::compute_dR(const Eigen::Vector3d& ang_vel,
                                                const Eigen::Vector3d& w_bias,
                                                double dt) {
  // Adjusts angular velcoity for bias
  Eigen::Vector3d adj_ang_vel = ang_vel - w_bias;

  // Computes delta rotation by exponential of so3
  Eigen::Matrix3d dR = so3_vec_to_SO3(adj_ang_vel * dt);

  return dR;
}

// --------------------------------------------------------------------------------------------------------
// Computes SO3 matrix from so3 vector by utilizing conversion between
// Eigen::AngleAxisd and Eigen::Matrix3d
Eigen::Matrix3d IntegratedLocalizer::so3_vec_to_SO3(
    const Eigen::Vector3d& so3_vec) {
  // Represents so3 vec as angle-axis
  double norm = so3_vec.norm();
  Eigen::AngleAxisd aa;
  if (norm == 0) {
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(0) = 1;
    aa = Eigen::AngleAxisd(0, axis);
  }
  else {
    aa = Eigen::AngleAxisd(norm, so3_vec / norm);
  }

  // Converts Eigen::AngleAxisd to Matrix3d
  Eigen::Matrix3d SO3(aa);

  return SO3;
}

// --------------------------------------------------------------------------------------------------------
// Increments state with update vector
IntegratedLocalizer::State IntegratedLocalizer::increment_state(
    const State& state,
    const Eigen::Matrix<double, 16, 1>& update_vec){
  State new_state = state;
  new_state.pos += update_vec.segment(0, 3);
  new_state.vel += update_vec.segment(3, 3);

  // Increments rotation by converting update vector to SO3 matrix
  //Eigen::Vector3d so3_vec = update_vec.segment(6, 3);
  //Eigen::Matrix3d update_ori = so3_vec_to_SO3(so3_vec);
  //new_state.ori *= update_ori;
  new_state.ori += update_vec.segment(6, 4);
  //std::cout << "new_state.ori" << new_state.ori << std::endl;

  new_state.a_bias += update_vec.segment(10, 3);
  new_state.w_bias += update_vec.segment(13, 3);

  return new_state;
}


// --------------------------------------------------------------------------------------------------------
// Obtains orientation difference in so3 vector using Eigen::AngleAxisd
Eigen::Vector3d IntegratedLocalizer::orientation_diff(
    const Eigen::Matrix3d& ori_a,
    const Eigen::Matrix3d& ori_b) {
  Eigen::Matrix3d ori_diff = ori_b.transpose() * ori_a;
  Eigen::AngleAxisd diff_aa = Eigen::AngleAxisd(ori_diff);
  Eigen::Vector3d diff_vec = diff_aa.axis() * diff_aa.angle();

  return diff_vec;
}


// --------------------------------------------------------------------------------------------------------
// Obtains the differences between two mearuements (position + orientation)
Eigen::Matrix<double, 7, 1> IntegratedLocalizer::measurement_diff(
    const Pose& measurement_a,
    const Pose& measurement_b) {
  Eigen::Matrix<double, 7, 1> diff;
  diff.segment(0, 3) = measurement_a.pos - measurement_b.pos;
  diff.segment(3, 4) = measurement_a.ori - measurement_b.ori;

  return diff;
}

// --------------------------------------------------------------------------------------------------------
// This function does the prediction step of the linear kalman Filter
// The linear kalman filter is assumed to be a constant velocity model
void IntegratedLocalizer::compute_pred_state() {
  if(delta_t < 5){
    pred_state_.pos = Eigen::Vector3d::Zero();
    pred_state_.vel = Eigen::Vector3d::Zero();
    pred_state_.ori = Eigen::Vector4d::Zero();
    pred_state_.a_bias = Eigen::Vector3d::Zero();
    pred_state_.w_bias = Eigen::Vector3d::Zero();
    A_matrix_(0, 3) = delta_t;
    A_matrix_(1, 4) = delta_t;
    A_matrix_(2, 5) = delta_t;

    State state_use;
    Eigen::Matrix<double, 16, 16> p_state_cov_;
    if(state_history_update.empty() == 1){
      state_use = state_history_.back();
      p_state_cov_ = state_cov_;
    }
    else{
      state_use = state_history_update.back();
      //p_state_cov_ = state_history_updated_cov_.back();
      p_state_cov_ = state_cov_;
    }

    Eigen::Matrix<double, 16, 1> state_vector = state_to_vector(state_use);
    Eigen::Matrix<double, 16, 1> update_vector = A_matrix_ * state_vector;
    pred_state_ = vector_to_state(update_vector);

    // 1, Obtains predicted orientation by weighted mean of rotation matrices
    //pred_state_.ori = compute_orientation_mean();

    // 2, Produce the predicted state covariance, by the weighted sigma points

    pred_state_cov_ = Eigen::Matrix<double, 16, 16>::Zero();
    pred_state_cov_ = A_matrix_ * p_state_cov_ * A_matrix_.transpose();
    pred_state_cov_ += state_noise_;
    //pred_state_.timestamp = imu_data_.back().timestamp;
  }
}


// --------------------------------------------------------------------------------------------------------
// Predicts lidar localizer measurement from state
IntegratedLocalizer::Pose IntegratedLocalizer::lidar_measurement_H_function(
    const IntegratedLocalizer::State& state) {
  Pose lidar_measurement;
  lidar_measurement.pos = state.pos;
  lidar_measurement.ori = state.ori;
  return lidar_measurement;
}


void IntegratedLocalizer::state_update_using_pred_sigma_points_lidar() {

  if(state_history_predict.empty() == 1){
    updated_state_ = state_history_.back();
    updated_state_cov_ = state_cov_;
    return;
  }
    State state_pred_ = state_history_predict.back();
    //Eigen::Matrix<double, 16, 16> state_cov_pred = state_history_pred_cov.back();
    Eigen::Matrix<double, 16, 16> state_cov_pred = state_cov_;

    //Eigen::Matrix<double, 16, 1> pred_state_vector = state_to_vector(state_);
    Eigen::Matrix<double, 16, 1> pred_state_vector = state_to_vector(state_pred_);
    Eigen::Matrix<double, 7, 1> pred_lidar_measurements = H_ * pred_state_vector;
    Pose pred_lidar_measurement_ = vector_to_pose(pred_lidar_measurements);

    I_ = Eigen::Matrix<double, 16, 16>::Zero();
    for(int i=0;i<16;i++){
      I_(i,i) = 1.0;
    }

    Eigen::Matrix<double, 7, 1> real_measure_diff;
    real_measure_diff = measurement_diff(lidar_measurement_,
                                         pred_lidar_measurement_);

    Eigen::Matrix<double, 16, 7> Kalman_gain = state_cov_pred * H_.transpose() * ((H_ * state_cov_pred * H_.transpose()) + lidar_noise_).inverse();
    Eigen::Matrix<double, 16, 1> update_vector = pred_state_vector + Kalman_gain * real_measure_diff;
    // Updates state and state covariance
    updated_state_ = vector_to_state(update_vector);

    std::cout<<ros::Time::now()<<std::endl;

    updated_state_cov_ = (I_ - Kalman_gain * H_) * state_cov_pred;

    // temporary test. reset the bias
    updated_state_.a_bias = Eigen::Vector3d::Zero();
    updated_state_.w_bias = Eigen::Vector3d::Zero();
    //updated_state_.ori = lidar_measurement_.ori;
    //updated_state_.timestamp = state_.timestamp;
    updated_state_.timestamp = lidar_measurement_.timestamp;
}

void IntegratedLocalizer::state_update_using_pred_sigma_points_gps() {


  if(state_history_predict.empty() == 1){
    updated_state_ = state_history_.back();
    updated_state_cov_ = state_cov_;
    return;
  }
    State state_pred_ = state_history_predict.back();
    //Eigen::Matrix<double, 16, 16> state_cov_pred = state_history_pred_cov.back();
    Eigen::Matrix<double, 16, 16> state_cov_pred = state_cov_;

    Eigen::Matrix<double, 16, 1> pred_state_vector = state_to_vector(state_pred_);

    Eigen::Matrix<double, 7, 1> pred_gps_measurements = H_ * pred_state_vector;
    Pose pred_gps_measurement_ = vector_to_pose(pred_gps_measurements);

    I_ = Eigen::Matrix<double, 16, 16>::Zero();
    for(int i=0;i<16;i++){
      I_(i,i) = 1.0;
    }

    Eigen::Matrix<double, 7, 1> real_measure_diff;
    real_measure_diff = measurement_diff(gps_measurement_,
                                         pred_gps_measurement_);

    Eigen::Matrix<double, 16, 7> Kalman_gain = state_cov_pred * H_.transpose() * ((H_ * state_cov_pred * H_.transpose()) + gps_noise_).inverse();
    Eigen::Matrix<double, 16, 1> update_vector = pred_state_vector + Kalman_gain * real_measure_diff;

    // Updates state and state covariance
    //updated_state_ = increment_state(pred_state_, update_vector);
    updated_state_ = vector_to_state(update_vector);
    updated_state_cov_ = (I_ - (Kalman_gain * H_)) * state_cov_pred;

    // temporary test. reset the bias
    updated_state_.a_bias = Eigen::Vector3d::Zero();
    updated_state_.w_bias = Eigen::Vector3d::Zero();
    //updated_state_.ori = gps_measurement_.ori;
    //updated_state_.timestamp = state_.timestamp;
    updated_state_.timestamp = gps_measurement_.timestamp;
}
// --------------------------------------------------------------------------------------------------------
// Covertes ros topic pose to localizer pose
IntegratedLocalizer::Pose IntegratedLocalizer::msg_to_pose(
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
  Pose pose;
  pose.pos(0) = pose_msg->pose.position.x;
  pose.pos(1) = pose_msg->pose.position.y;
  pose.pos(2) = pose_msg->pose.position.z;
  //Eigen::Quaterniond ori(pose_msg->pose.orientation.w,
  //                      pose_msg->pose.orientation.x,
 	//		                  pose_msg->pose.orientation.y,
 	//		                  pose_msg->pose.orientation.z);

  //pose.ori = ori;
  //std::cout << "lidar msg ori = " << pose.ori << std::endl;
  pose.ori(0) = pose_msg->pose.orientation.x;
  pose.ori(1) = pose_msg->pose.orientation.y;
  pose.ori(2) = pose_msg->pose.orientation.z;
  pose.ori(3) = pose_msg->pose.orientation.w;

  int sec = pose_msg->header.stamp.sec;
  int nsec = pose_msg->header.stamp.nsec;
  pose.timestamp = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
  return pose;
}

// --------------------------------------------------------------------------------------------------------
IntegratedLocalizer::State IntegratedLocalizer::vector_to_state(
    const Eigen::Matrix<double, 16, 1>& vector_info) {
  State new_state;
  new_state.pos = vector_info.segment(0, 3);
  new_state.vel = vector_info.segment(3, 3);
  new_state.ori = vector_info.segment(6, 4);
  new_state.a_bias = vector_info.segment(10, 3);
  new_state.w_bias = vector_info.segment(13, 3);
  return new_state;
}

// --------------------------------------------------------------------------------------------------------
IntegratedLocalizer::Pose IntegratedLocalizer::vector_to_pose(
    const Eigen::Matrix<double, 7, 1>& vector_info) {
  Pose new_pose;
  new_pose.pos = vector_info.segment(0, 3);
  new_pose.ori = vector_info.segment(3, 4);
  return new_pose;
}

// --------------------------------------------------------------------------------------------------------
Eigen::Matrix<double, 16, 1> IntegratedLocalizer::state_to_vector(
    const IntegratedLocalizer::State& state_vector) {

  Eigen::Matrix<double, 16, 1> diff;
  diff.segment(0, 3) = state_vector.pos;
  diff.segment(3, 3) = state_vector.vel;
  diff.segment(6, 4) = state_vector.ori;
  diff.segment(10, 3) = state_vector.a_bias;
  diff.segment(13, 3) = state_vector.w_bias;

  return diff;
}


IntegratedLocalizer::Pose IntegratedLocalizer::state_to_pose(
    const IntegratedLocalizer::State& state_pose) {
  Pose new_pose;
  new_pose.pos = state_pose.pos;
  new_pose.ori = state_pose.ori;
  return new_pose;
}

// --------------------------------------------------------------------------------------------------------
// Converts lidar pose to imu pose
IntegratedLocalizer::Pose IntegratedLocalizer::lidar_pose_to_imu_pose(
    Pose lidar_pose) {
  Pose imu_pose;
  Eigen::Matrix3d orientation_li;
  //Eigen::Quaterniond ori(lidar_pose.ori(3),
  //                      lidar_pose.ori(0),
 	//		                  lidar_pose.ori(1),
 	//		                  lidar_pose.ori(2));
  Eigen::Quaterniond q1;
  q1.x() = imu_pose.ori(0);
  q1.y() = imu_pose.ori(1);
  q1.z() = imu_pose.ori(2);
  q1.w() = imu_pose.ori(3);
  Eigen::Matrix3d ori = q1.toRotationMatrix();
  orientation_li = ori * lidar_to_imu_.ori;
  imu_pose.pos = ori * lidar_to_imu_.pos + lidar_pose.pos;
  imu_pose.timestamp = lidar_pose.timestamp;

  Eigen::Quaterniond q(orientation_li);
  imu_pose.ori(0) = q.x();
  imu_pose.ori(1) = q.y();
  imu_pose.ori(2) = q.z();
  imu_pose.ori(3) = q.w();

  return imu_pose;
}

// --------------------------------------------------------------------------------------------------------
// Converts imu pose to lidar pose
IntegratedLocalizer::Pose IntegratedLocalizer::imu_pose_to_lidar_pose(
    Pose imu_pose) {
  Pose lidar_pose;
  Eigen::Matrix3d orientation_il;
  //Eigen::Quaterniond ori(imu_pose.ori(3),
  //                      imu_pose.ori(0),
 	//		                  imu_pose.ori(1),
 	//		                  imu_pose.ori(2));
  Eigen::Quaterniond q1;
  q1.x() = imu_pose.ori(0);
  q1.y() = imu_pose.ori(1);
  q1.z() = imu_pose.ori(2);
  q1.w() = imu_pose.ori(3);
  Eigen::Matrix3d ori = q1.toRotationMatrix();
  orientation_il = ori * imu_to_lidar_.ori;
  lidar_pose.pos = ori * imu_to_lidar_.pos + imu_pose.pos;
  lidar_pose.timestamp = imu_pose.timestamp;

  Eigen::Quaterniond q(orientation_il);
  lidar_pose.ori(0) = q.x();
  lidar_pose.ori(1) = q.y();
  lidar_pose.ori(2) = q.z();
  lidar_pose.ori(3) = q.w();

  return lidar_pose;
}





// The IMU topic comes in at 100 Hz.The IMU topic is used to call the predict function of the linear kalman filter
// The callback function calls the 'compute_pred_state()' which uses constant velocity model for prediction
void IntegratedLocalizer::imu_data_callback_CorrectedImu(
    const integrated_localizer::CorrectedIMU::ConstPtr& imu_data) {
  IMU imu_sample;

  imu_sample.lin_acc(0) = imu_data->Acceleration[0];
  imu_sample.lin_acc(1) = imu_data->Acceleration[1];
  imu_sample.lin_acc(2) = imu_data->Acceleration[2];
  imu_sample.ang_vel(0) = imu_data->RotationRate[0];
  imu_sample.ang_vel(1) = imu_data->RotationRate[1];
  imu_sample.ang_vel(2) = imu_data->RotationRate[2];

  if (!is_rad_per_sec_) {
    imu_sample.ang_vel(0) = imu_sample.ang_vel(0) * M_PI / 180.0;
    imu_sample.ang_vel(1) = imu_sample.ang_vel(1) * M_PI / 180.0;
    imu_sample.ang_vel(2) = imu_sample.ang_vel(2) * M_PI / 180.0;
  }

  //prev_imu_timestamp = lidar_pose.timestamp;

  imu_sample.timestamp = imu_data->GpsTimeWeek;
  imu_data_.push_back(imu_sample);
  check_imu_data_size();
  // Only with localizer initialized
  if (is_state_initialized_) {
    // Only with more than 2 imu samples to get dt
    if (imu_data_.size() >= 1) {
      // Propagates state using imu sample by UKF
      delta_t = imu_sample.timestamp - prev_imu_timestamp;
      compute_pred_state();
      std::cout<<"predict"<<std::endl;
      state_ = pred_state_;
      state_cov_ = pred_state_cov_;
      state_history_predict.push_back(pred_state_);
      //state_history_pred_cov.push_back(pred_state_cov_);
      check_state_history_size();
    }
  }
  prev_imu_timestamp = imu_sample.timestamp;
}


// The IMU topic comes in at 100 Hz.The IMU topic is used to call the predict function of the linear kalman filter
// The callback function calls the 'compute_pred_state()'

void IntegratedLocalizer::imu_data_callback_sensorMsgs(
    const sensor_msgs::Imu::ConstPtr& imu_data) {
  IMU imu_sample;

  imu_sample.lin_acc(0) = imu_data->linear_acceleration.x;
  imu_sample.lin_acc(1) = imu_data->linear_acceleration.y;
  imu_sample.lin_acc(2) = imu_data->linear_acceleration.z;
  imu_sample.ang_vel(0) = imu_data->angular_velocity.x;
  imu_sample.ang_vel(1) = imu_data->angular_velocity.y;
  imu_sample.ang_vel(2) = imu_data->angular_velocity.z;

  if (!is_rad_per_sec_) {
    imu_sample.ang_vel(0) = imu_sample.ang_vel(0) * M_PI / 180.0;
    imu_sample.ang_vel(1) = imu_sample.ang_vel(1) * M_PI / 180.0;
    imu_sample.ang_vel(2) = imu_sample.ang_vel(2) * M_PI / 180.0;
  }

    int sec = imu_data->header.stamp.sec;
    int nsec = imu_data->header.stamp.nsec;
    imu_sample.timestamp = static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;


  //ROS_INFO("IMU sensor_msgs timestamp ================= %10.10lf!", imu_sample.timestamp);
    imu_data_.push_back(imu_sample);
    check_imu_data_size();

    // Only with localizer initialized
    if (is_state_initialized_) {
      // Only with more than 2 imu samples to get dt
      if (imu_data_.size() >= 1) {
        // Propagates state using imu sample by UKF
        delta_t = imu_sample.timestamp - prev_imu_timestamp;
        compute_pred_state();
        std::cout<<"predict"<<std::endl;
        state_ = pred_state_;
        state_cov_ = pred_state_cov_;
        state_history_predict.push_back(pred_state_);
        //state_history_pred_cov.push_back(pred_state_cov_);
        check_state_history_size();
      }
    }

    prev_imu_timestamp = imu_sample.timestamp;
  }


// Update step of the Linear Kalman filter when data is published on the NDT 'lidar' topic.
// The 'state_update_using_pred_sigma_points_lidar()' uses the NDT (lidar) pose in the update step of the kalman filter.

void IntegratedLocalizer::lidar_pose_callback(
  const geometry_msgs::PoseStamped::ConstPtr& lidar_pose_msg) {
    //lidar_initialized = 1;
    Pose lidar_pose = msg_to_pose(lidar_pose_msg);
    if (lidar_pose.timestamp - prev_lidar_pose_timestamp_ > 0.2) {
      ROS_INFO("Time diff between two Lidar pose samples: %f", lidar_pose.timestamp - prev_lidar_pose_timestamp_);
    }

    prev_lidar_pose_timestamp_ = lidar_pose.timestamp;
    Pose lidar_measurement = lidar_pose_to_imu_pose(lidar_pose);

    if (!is_state_initialized_) {
      set_init_state(lidar_measurement);
      is_state_initialized_ = true;
      lidar_points_manipulation2(lidar_measurement);
      std::cout<<"lidar initialize"<<std::endl;
      ROS_INFO("Set init state from Lidar localizer data!");
    }
    else {
      lidar_measurement_ = lidar_measurement;
      lidar_points_manipulation2(lidar_measurement_);
      state_update_using_pred_sigma_points_lidar();
      state_ = updated_state_;
      std::cout<<"lidar update"<<std::endl;
      state_cov_ = updated_state_cov_;
      state_history_update.push_back(state_);
      //state_history_updated_cov_.push_back(state_cov_);
      check_state_history_size();
    }
  }


// Update step of the Linear Kalman filter when data is published on the 'GPS' topic.
// It takes in the consecutive GPS position readings, normalizes with delta_time to get the velocity, and uses this velocity to interpolate the GPS position data.
// The 'lidar_points_manipulation()' function uses this normalized velocity and time to interpolate the position on the NDT profile, which will later be used in the update step.
// The 'state_update_using_pred_sigma_points_gps()' uses the interpolated pose from the earlier function in the update step of the kalman filter.

void IntegratedLocalizer::gps_pose_callback(
    const geometry_msgs::PoseStamped::ConstPtr& gps_pose_msg) {
      Pose gps_pose = msg_to_pose(gps_pose_msg);
      if (gps_pose.timestamp - prev_gps_pose_timestamp_ > 0.2) {

        ROS_INFO("Time diff between two GPS pose samples: %f", gps_pose.timestamp - prev_gps_pose_timestamp_);
      }

      if (prev_gnss_x == 0.0 && prev_gnss_y == 0.0){
        prev_gnss_x = gps_pose.pos(0);
        prev_gnss_y = gps_pose.pos(1);
      }
      delta_x = gps_pose.pos(0) - prev_gnss_x;
      delta_y = gps_pose.pos(1) - prev_gnss_y;
      delta_time = gps_pose.timestamp - prev_gps_pose_timestamp_;
      std::cout<<"Difference between old and new updates "<<delta_time<<std::endl;
      velocity_deltax = delta_x/delta_time;
      velocity_deltay = delta_y/delta_time;

      //Pose gps_measurement = lidar_pose_to_imu_pose(gps_pose);
      Pose gps_measurement = gps_pose;
      lidar_points_manipulation(velocity_deltax, velocity_deltay, delta_time, gps_measurement);

      if (!is_state_initialized_) {
        set_init_state(gps_measurement);
        is_state_initialized_ = true;
        is_gnss_initialized = 1;
        std::cout<<"gps initialize"<<std::endl;
        ROS_INFO("Set init state from GPS localizer data!");
      }
      else {
        Pose gp_pose = state_to_pose(state_history_.back());
        gps_measurement_ = gp_pose;
        state_update_using_pred_sigma_points_gps();
        state_ = updated_state_;
        std::cout<<"gps update"<<std::endl;
        state_cov_ = updated_state_cov_;
        state_history_update.push_back(state_);
        //state_history_updated_cov_.push_back(state_cov_);
        check_state_history_size();
  }
    prev_gnss_x = gps_pose.pos(0);
    prev_gnss_y = gps_pose.pos(1);
    prev_gps_pose_timestamp_ = gps_pose.timestamp;
}


// This function is used to interpolate the pose data along the NDT (more accurate) profile.
// It takes in arguments the normalized velocity in the x & y direction and the delta_time between consecutive data.
// The state is updated.

void IntegratedLocalizer::lidar_points_manipulation(double vx, double vy, double dt, const Pose& gps_measure){
  if(is_gnss_initialized != 1){
    State locate;
    locate.pos(0) = gps_measure.pos(0) + vx*dt;
    locate.pos(1) = gps_measure.pos(1) + vy*dt;
    locate.pos(2) = gps_measure.pos(2);
    locate.ori = gps_measure.ori;
    locate.vel = Eigen::Vector3d::Zero();
    locate.a_bias = Eigen::Vector3d::Zero();
    locate.w_bias = Eigen::Vector3d::Zero();
    locate.timestamp = gps_measure.timestamp;
    state_history_.push_back(locate);
    state_history_update.push_back(locate);
    check_state_history_size();
  }
  else{
    State locate;
    State temp = state_history_.back();
    locate.pos(0) = temp.pos(0) + vx*dt;
    locate.pos(1) = temp.pos(1) + vy*dt;
    locate.pos(2) = temp.pos(2);
    locate.ori = temp.ori;
    locate.vel = Eigen::Vector3d::Zero();
    locate.a_bias = Eigen::Vector3d::Zero();
    locate.w_bias = Eigen::Vector3d::Zero();
    locate.timestamp = temp.timestamp;
    state_history_.push_back(locate);
    check_state_history_size();
  }
}

void IntegratedLocalizer::lidar_points_manipulation2(const Pose& lidar_measure){
  State locate;
  locate.pos = lidar_measure.pos;
  locate.ori = lidar_measure.ori;
  locate.vel = Eigen::Vector3d::Zero();
  locate.a_bias = Eigen::Vector3d::Zero();
  locate.w_bias = Eigen::Vector3d::Zero();
  locate.timestamp = lidar_measure.timestamp;
  state_history_.push_back(locate);
  check_state_history_size();
}
// --------------------------------------------------------------------------------------------------------
// Sets initial state using the initial pose
void IntegratedLocalizer::set_init_state(const Pose& init_pose){

  // state initialization
  state_.pos = init_pose.pos;
  state_.vel = Eigen::Vector3d::Zero();
  state_.ori = init_pose.ori;
  state_.a_bias = Eigen::Vector3d::Zero();
  state_.w_bias = Eigen::Vector3d::Zero();
  state_.timestamp = init_pose.timestamp;
  state_history_.push_back(state_);
}


// --------------------------------------------------------------------------------------------------------
// Checks if the localizer state is initialized
bool IntegratedLocalizer::is_state_initialized() {
  return is_state_initialized_;
}

// --------------------------------------------------------------------------------------------------------
// Checks if the slam started state is initialized
bool IntegratedLocalizer::is_slam_state_started() {
  return is_slam_state_started_;
}


// --------------------------------------------------------------------------------------------------------
// Returns latest lidar pose by converting the latest state to lidar pose
IntegratedLocalizer::Pose IntegratedLocalizer::get_latest_lidar_pose() {
  State latest_state = state_history_update.back();
  Pose imu_pose;
  imu_pose.pos = latest_state.pos;
  imu_pose.ori = latest_state.ori;
  imu_pose.timestamp = latest_state.timestamp;
  Pose lidar_pose = imu_pose_to_lidar_pose(imu_pose);
  return lidar_pose;
}

// --------------------------------------------------------------------------------------------------------
// Returns latest lidar pose by converting the latest state to lidar pose
IntegratedLocalizer::Pose IntegratedLocalizer::get_latest_camera_pose() {
  State latest_state = state_history_.back();
  Pose imu_pose;
  imu_pose.pos = latest_state.pos;
  imu_pose.ori = latest_state.ori;
  imu_pose.timestamp = latest_state.timestamp;

  //std::cout << "imu_pose.pos = " << imu_pose.pos << std::endl;
  //std::cout << "imu_pose.ori = " << imu_pose.ori << std::endl;

  Pose camera_pose = imu_pose_to_lidar_pose(imu_pose);
  return camera_pose;
}
