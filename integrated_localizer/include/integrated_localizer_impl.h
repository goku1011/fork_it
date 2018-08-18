#ifndef INTEGRATED_LOCALIZER_IMPL_H
#define INTEGRATED_LOCALIZER_IMPL_H

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include "integrated_localizer.h"
#include <iostream>
#include <string>
#include <vector>

#include "GPSUtils.h"

// =========================================================================================================
template<typename Type>
void quaternionFromRotationMatrix(Type m00, Type m01, Type m02,
								Type m10, Type m11, Type m12,
								Type m20, Type m21, Type m22,
								Type &x, Type &y, Type &z, Type &w) {
	// Use the Graphics Gems code, from
    // ftp://ftp.cis.upenn.edu/pub/graphics/shoemake/quatut.ps.Z
    Type t = m00 + m11 + m22;
    // we protect the division by s by ensuring that s>=1
    if (t >= 0) { // by w
        Type s = sqrt(t + 1);
        w = 0.5 * s;
        s = 0.5 / s;
        x = (m21 - m12) * s;
        y = (m02 - m20) * s;
        z = (m10 - m01) * s;
    } else if ((m00 > m11) && (m00 > m22)) { // by x
        Type s = sqrt(1 + m00 - m11 - m22);
        x = s * 0.5;
        s = 0.5 / s;
        y = (m10 + m01) * s;
        z = (m02 + m20) * s;
        w = (m21 - m12) * s;
    } else if (m11 > m22) { // by y
        Type s = sqrt(1 + m11 - m00 - m22);
        y = s * 0.5;
        s = 0.5 / s;
        x = (m10 + m01) * s;
        z = (m21 + m12) * s;
        w = (m02 - m20) * s;
    } else { // by z
        Type s = sqrt(1 + m22 - m00 - m11);
        z = s * 0.5;
        s = 0.5 / s;
        x = (m02 + m20) * s;
        y = (m21 + m12) * s;
        w = (m10 - m01) * s;
    }
}

class IntegratedLocalizerImpl {
  public:
    IntegratedLocalizerImpl(ros::NodeHandle nh);
    ~IntegratedLocalizerImpl();

    void run();


  private:

    // Checks if two poses are the same or not
    bool is_different_pose(const IntegratedLocalizer::Pose&,
                       const IntegratedLocalizer::Pose&);

    // Writes ros msg using pose from main localizer
    /*void write_msg(geometry_msgs::PoseWithCovarianceStamped&,
               const IntegratedLocalizer::Pose&);*/
    void write_msg(geometry_msgs::PoseStamped&,
               const IntegratedLocalizer::Pose&);

    // Writes
    void write_geopose_msg(geographic_msgs::GeoPoseStamped&,
                      const IntegratedLocalizer::Pose&);

    // read parameters from param.yaml file
    void read_param_file(ros::NodeHandle nh);

    IntegratedLocalizer* localizerPtr;

    // Sets ros topic subscribers
    ros::Subscriber imu_data_sub;
    ros::Subscriber camera_data_sub;
    ros::Subscriber lidar_data_sub;
    ros::Subscriber vo_pose_sub;
    ros::Subscriber lidar_pose_sub;
    ros::Subscriber gps_pose_sub;

    // Sets ros topic publisher
    ros::Publisher output_pose_pub;
    ros::Publisher output_camera_pose_pub;
    ros::Publisher output_camera_geopose_pub;

    IntegratedLocalizer::Pose prev_pose;
    IntegratedLocalizer::Pose prev_camera_pose;

    // Reads msg names from ROS parameter server
    std::string imu_data_msg_name;
    std::string camera_data_msg_name;
    std::string lidar_data_msg_name;
    std::string vo_pose_msg_name;
    std::string lidar_pose_msg_name;
    std::string gps_pose_msg_name;
    std::string output_pose_msg_name;
    std::string output_camera_pose_msg_name;
    std::string output_camera_geopose_msg_name;

    // Reads covariance parameters from ROS parameter server
    std::vector<double> init_state_cov;
    std::vector<double> state_noise;
    std::vector<double> gps_noise;
    std::vector<double> lidar_noise;

    // Read UKF parameters from ROS parameter server
    double alpha;
    double beta;
    double kappa;
    float attenuation_coefficient;
    int is_rad_per_sec;
    int is_sensor_msgs_imu;
    double TAI_offset;

    // Reads lidar to imu transformation parameters from ROS parameter server
    std::vector<double> lidar_to_imu;

    // Reads camera to imu transformation parameters from ROS parameter server
    std::vector<double> camera_to_imu;

    // rotation matrix from camera to gps
	  std::vector<double> Q_camera_2_gps;

};


#endif
