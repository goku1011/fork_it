#ifndef INTEGRATED_LOCALIZER_H
#define INTEGRATED_LOCALIZER_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <deque>
#include "GPSUtils.h"

#include <integrated_localizer/CorrectedIMU.h>


// =========================================================================================================
template<typename Type>
void RotationMatrixFromQuaternion(Type x, Type y, Type z, Type w,
								Type& m00, Type& m01, Type& m02,
								Type& m10, Type& m11, Type& m12,
								Type& m20, Type& m21, Type& m22) {
	m00 = 1 - 2 * y * y - 2 * z * z;
	m01 = 2 * x * y - 2 * z * w;
	m02 = 2 * x * z + 2 * y * w;
	m10 = 2 * x * y + 2 * z * w;
	m11 = 1 - 2 * x * x - 2 * z * z;
	m12 = 2 * y * z - 2 * x * w;
	m20 = 2 * x * z - 2 * y * w;
	m21 = 2 * y * z + 2 * x * w;
	m22 = 1 - 2 * x * x - 2 * y * y;
}

class IntegratedLocalizer {
  public:
    struct State {
      Eigen::Vector3d pos;
      Eigen::Vector3d vel;
      Eigen::Vector4d ori;
      Eigen::Vector3d a_bias;
      Eigen::Vector3d w_bias;
      double timestamp;
    };

    struct Pose {
      Eigen::Vector3d pos;
      Eigen::Vector4d ori;
      double timestamp;
    };

		struct Pose2 {
      Eigen::Vector3d pos;
      Eigen::Matrix3d ori;
      double timestamp;
    };

  private:
    enum class AugType {VO, LIDAR};

    Pose2 lidar_to_imu_;
    Pose2 imu_to_lidar_;
    Pose lidar_pose_to_imu_pose(Pose);
    Pose imu_pose_to_lidar_pose(Pose);
    void set_transform_lidar_imu(const std::vector<double>&);

    Pose2 camera_to_imu_;
    Pose2 imu_to_camera_;
    //Pose camera_pose_to_imu_pose(Pose);
    Pose imu_pose_to_camera_pose(Pose);
    void set_transform_camera_imu(const std::vector<double>&);

    State state_;
    Eigen::Matrix<double, 16, 16> state_noise_;
    Eigen::Matrix<double, 16, 16> state_cov_;

    std::vector<Pose> aug_poses_;
    std::vector<AugType> aug_types_;
    std::vector<Eigen::Matrix<double, 16, 7>> state_aug_pose_cross_cov_;
    std::vector<Eigen::Matrix<double, 7, 7>> aug_pose_cov_;
    std::vector<Eigen::Matrix<double, 7, 7>> aug_pose_cross_cov_;

    std::deque<State> state_history_;
		std::deque<State> state_history_update;
		std::deque<State> state_history_predict;
		
    int state_history_size_limit_ = 100;
    void check_state_history_size();

    struct IMU {
      Eigen::Vector3d lin_acc;
      Eigen::Vector3d ang_vel;
      double timestamp;
    };
    std::deque<IMU> imu_data_;
    int imu_data_size_limit_ = 100;
    void check_imu_data_size();

    Pose msg_to_pose(const geometry_msgs::PoseStamped::ConstPtr&);
    std::deque<Pose> vo_measurements_;
    Pose lidar_measurement_;
		Pose gps_measurement_;
    Eigen::Matrix<double, 7, 7> gps_noise_;
    Eigen::Matrix<double, 7, 7> lidar_noise_;

    // gps pose output
	  geographic_msgs::GeoPoseStamped current_gps_pose_;
	  geographic_msgs::GeoPoseStamped gps_geopose_slam_start_;

    void set_init_state(const Pose&);
    void set_cov(const std::vector<double>&,
	         const std::vector<double>&,
	         const std::vector<double>&,
		 const std::vector<double>&);
    void state_propagation();

    bool is_state_initialized_ = false;
    bool is_slam_state_started_ = false;
    bool is_slam_first_started_ = false;
    int imu_index_ = -1;
    bool is_imu_data_pop_front_ = false;

    Eigen::Vector3d gravity_;
    Eigen::Matrix3d compute_dR(const Eigen::Vector3d&,
	                       const Eigen::Vector3d&,
			       double);
    Eigen::Matrix3d so3_vec_to_SO3(const Eigen::Vector3d&);

    std::vector<State> sigma_points_;
    std::vector<State> pred_sigma_points_;
    double alpha_, beta_, kappa_, lambda_;
    float attenuation_coefficient_;
    int is_rad_per_sec_;
    std::vector<double> mean_weights_;
    std::vector<double> cov_weights_;
    State pred_state_;
    Eigen::Matrix<double, 16, 16> pred_state_cov_;
    Pose pred_lidar_measurement_;
		Pose pred_gps_measurement_;
    Eigen::Matrix<double, 7, 7> pred_lidar_measurement_cov_;
    Eigen::Matrix<double, 16, 7> pred_state_lidar_cross_cov_;
    Eigen::Matrix<double, 7, 7> pred_gps_measurement_cov_;
    Eigen::Matrix<double, 16, 7> pred_state_gps_cross_cov_;
    State updated_state_;
    Eigen::Matrix<double, 16, 16> updated_state_cov_;

    void compute_weights();
    void create_sigma_points();
    void propagate_sigma_points();
    void compute_pred_state();
    void create_pred_sigma_points();
    void state_update_using_pred_sigma_points_lidar();
    void state_update_using_pred_sigma_points_gps();
    Pose lidar_measurement_H_function(const State&);
    Pose gps_measurement_H_function(const State&);
    Eigen::Matrix3d compute_orientation_mean();

    State increment_state(const State&, const Eigen::Matrix<double, 16, 1>&);
    //Eigen::Matrix<double, 15, 1> state_diff(const State&, const State&);
    Eigen::Matrix<double, 7, 1> measurement_diff(const Pose&, const Pose&);
    Eigen::Vector3d orientation_diff(const Eigen::Matrix3d&,
	                             const Eigen::Matrix3d&);

    ros::Time prev_imu_timestamp_;

    double prev_lidar_pose_timestamp_;
		double prev_gps_pose_timestamp_;
		double prev_imu_timestamp;
		double prev_gnss_x;
		double prev_gnss_y;
		double delta_t;
    double TAI_offset_;

		Eigen::Matrix<double, 16, 1> state_to_vector(const State&);
		State vector_to_state(const Eigen::Matrix<double, 16, 1>&);
		Pose vector_to_pose(const Eigen::Matrix<double, 7, 1>&);

  public:
    IntegratedLocalizer(const std::vector<double>&,
	                const std::vector<double>&,
			            const std::vector<double>&,
			            const std::vector<double>&,
			            double,
			            double,
			            double,
                  float,
                  int,
			            const std::vector<double>&,
                  const std::vector<double>&,
                  double);

    void imu_data_callback_sensorMsgs(const sensor_msgs::Imu::ConstPtr&);
    void imu_data_callback_CorrectedImu(const integrated_localizer::CorrectedIMU::ConstPtr&);
    //void camera_data_callback(const sensor_msgs::Image::ConstPtr&);
    //void lidar_data_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void lidar_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void gps_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
		void vo_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);

    Pose get_latest_lidar_pose();
    Pose get_latest_camera_pose();
    bool is_state_initialized();
    bool is_slam_state_started();

    double latitude0_, longitude0_, altitude0_;
    // Rotation matrix from GPS World coordinate to GPS local coordinate
  	Eigen::Matrix3d Q_GW_G_0_;

		Eigen::Matrix<double, 7, 16> H_;
		Eigen::Matrix<double, 16, 16> A_matrix_;
		Eigen::Matrix<double, 16, 16> I_;

		std::deque<Eigen::Matrix<double, 7, 7>> state_history_cov;
		std::deque<Eigen::Matrix<double, 7, 7>> state_history_pred_cov;
		std::deque<Eigen::Matrix<double, 7, 7>> state_history_updated_cov_;
};

#endif
