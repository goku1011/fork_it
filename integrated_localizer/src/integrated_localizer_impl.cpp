#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include "integrated_localizer.h"
#include "integrated_localizer_impl.h"
#include <iostream>
#include <string>
#include <vector>

// --------------------------------------------------------------------------------------------------------
IntegratedLocalizerImpl::IntegratedLocalizerImpl(ros::NodeHandle nh) {

  // get all the parameters
  read_param_file(nh);

  // Initializes localizer with parameters
  localizerPtr = new IntegratedLocalizer(init_state_cov,
                                state_noise,
                                gps_noise,
				                        lidar_noise,
				                        alpha,
				                        beta,
				                        kappa,
                                attenuation_coefficient,
                                is_rad_per_sec,
			                          lidar_to_imu,
                                camera_to_imu,
                                TAI_offset);

  // Sets ros topic subscribers
  if (is_sensor_msgs_imu == 1) {
    imu_data_sub = nh.subscribe(imu_data_msg_name,
                              1000,
			      &IntegratedLocalizer::imu_data_callback_sensorMsgs,
			      localizerPtr);
  } else {
    imu_data_sub = nh.subscribe(imu_data_msg_name,
                              1000,
			      &IntegratedLocalizer::imu_data_callback_CorrectedImu,
			      localizerPtr);
  }


//  camera_data_sub = nh.subscribe(camera_data_msg_name,
//                                 100,
//   				  &IntegratedLocalizer::camera_data_callback,
//   				  localizerPtr);

//  lidar_data_sub = nh.subscribe(lidar_data_msg_name,
//                                100,
//				    &IntegratedLocalizer::lidar_data_callback,
//				    localizerPtr);

//  vo_pose_sub = nh.subscribe(vo_pose_msg_name,
//                             1000,
//			      &IntegratedLocalizer::vo_pose_callback,
//			      localizerPtr);

  lidar_pose_sub = nh.subscribe(lidar_pose_msg_name,
                                1000,
				    &IntegratedLocalizer::lidar_pose_callback,
				    localizerPtr);

  gps_pose_sub = nh.subscribe(gps_pose_msg_name,
                                1000,
				    &IntegratedLocalizer::gps_pose_callback,
				    localizerPtr);

  // Sets ros topic publisher
  /*output_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
                              output_pose_msg_name,
                              1000);*/
  output_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
                              output_pose_msg_name,
                              1000);

  output_camera_pose_pub = nh.advertise<geographic_msgs::GeoPoseStamped>(
                              output_camera_pose_msg_name,
                              1000);

}

// --------------------------------------------------------------------------------------------------------
IntegratedLocalizerImpl::~IntegratedLocalizerImpl() {
  delete localizerPtr;
}


// --------------------------------------------------------------------------------------------------------
// main function
void IntegratedLocalizerImpl::run() {
  //ros::Time time_now = ros::Time::now();
  //std::cout << "time " <<time_now << std::endl;
  if (localizerPtr->is_state_initialized()) {
      IntegratedLocalizer::Pose latest_pose = localizerPtr->get_latest_lidar_pose();

      // Publishes only new pose different from previous pose
      if (is_different_pose(prev_pose, latest_pose)) {
  	    //geometry_msgs::PoseWithCovarianceStamped msg;
        geometry_msgs::PoseStamped msg;
  	    write_msg(msg, latest_pose);
  	    output_pose_pub.publish(msg);
        //std::cout << "published pose = " << latest_pose.pos << std::endl;

	      prev_pose = latest_pose;
      }
    }
    else {
      //ROS_INFO("Localizer state not initialized yet!");
      //std::cout << "Localizer state not initialized yet! \n" ;
    }

  //std::cout << " is_slam_state_started = " << localizerPtr->is_slam_state_started() << std::endl;
  if (localizerPtr->is_slam_state_started()) {
      IntegratedLocalizer::Pose latest_camera_pose = localizerPtr->get_latest_camera_pose();


      // Publishes only new pose different from previous pose
      //std::cout << "is different pose = " << is_different_pose(prev_camera_pose, latest_camera_pose) << std::endl;
      if (is_different_pose(prev_camera_pose, latest_camera_pose)) {
        geographic_msgs::GeoPoseStamped msg;
  	    write_geopose_msg(msg, latest_camera_pose);
  	    output_camera_pose_pub.publish(msg);

	      prev_camera_pose = latest_camera_pose;
      }
    }
    else {
      //ROS_INFO("Localizer state not initialized yet!");
      //std::cout << "Localizer state not initialized yet 1! \n" ;
    }
}
// --------------------------------------------------------------------------------------------------------
void IntegratedLocalizerImpl::read_param_file(ros::NodeHandle nh) {
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Reads msg names from ROS parameter server

  if (nh.getParam("/imu_data_msg_name", imu_data_msg_name)) {
    ROS_INFO("Got IMU data msg name: %s", imu_data_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting imu data msg name!");
    return;
  }

  if (nh.getParam("/camera_data_msg_name", camera_data_msg_name)) {
    ROS_INFO("Got camera data msg name: %s", camera_data_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting camera data msg name!");
    return;
  }

  if (nh.getParam("/lidar_data_msg_name", lidar_data_msg_name)) {
    ROS_INFO("Got lidar data msg name: %s", lidar_data_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting lidar data msg name!");
    return;
  }

  if (nh.getParam("/vo_pose_msg_name", vo_pose_msg_name)) {
    ROS_INFO("Got VO pose msg name: %s", vo_pose_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting VO pose msg name!");
    return;
  }

  if (nh.getParam("/lidar_pose_msg_name", lidar_pose_msg_name)) {
    ROS_INFO("Got lidar pose msg name: %s", lidar_pose_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting lidar pose msg name!");
    return;
  }

  if (nh.getParam("/gps_pose_msg_name", gps_pose_msg_name)) {
    ROS_INFO("Got gps pose msg name: %s", gps_pose_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting gps pose msg name!");
    return;
  }

  if (nh.getParam("/output_pose_msg_name", output_pose_msg_name)) {
    ROS_INFO("Got output pose msg name: %s", output_pose_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting output pose msg name!");
    return;
  }

  if (nh.getParam("/output_camera_pose_msg_name", output_camera_pose_msg_name)) {
    ROS_INFO("Got output camera pose msg name: %s", output_camera_pose_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting output camera pose msg name!");
    return;
  }

  if (nh.getParam("/output_camera_geopose_msg_name", output_camera_geopose_msg_name)) {
    ROS_INFO("Got output camera geopose msg name: %s", output_camera_geopose_msg_name.c_str());
  }
  else {
    ROS_INFO("Error in getting output camera geopose msg name!");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Reads covariance parameters from ROS parameter server

  if (nh.getParam("/init_state_cov", init_state_cov)) {
    ROS_INFO("Got initial state cov!");
  }
  else {
    ROS_INFO("Error in getting initial state cov!");
    return;
  }

  if (nh.getParam("/state_noise", state_noise)) {
    ROS_INFO("Got state noise!");
  }
  else {
    ROS_INFO("Error in getting state noise!");
    return;
  }

  if (nh.getParam("/gps_noise", gps_noise)) {
    ROS_INFO("Got gps noise!");
  }
  else {
    ROS_INFO("Error in getting vo noise!");
    return;
  }

  if (nh.getParam("/lidar_noise", lidar_noise)) {
    ROS_INFO("Got lidar noise!");
  }
  else {
    ROS_INFO("Error in getting lidar noise!");
    return;
  }

  if (init_state_cov.size() != 16) {
    ROS_INFO("Wrong size of initial state cov!");
    return;
  }

  if (state_noise.size() != 16) {
    ROS_INFO("Wrong size of state noise!");
    return;
  }

  if (gps_noise.size() != 7) {
    ROS_INFO("Wrong size of vo noise!");
    return;
  }

  if (lidar_noise.size() != 7) {
    ROS_INFO("Wrong size of lidar noise!");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Read UKF parameters from ROS parameter server

  if (nh.getParam("/alpha", alpha)) {
    ROS_INFO("Got alpha for UKF!");
  }
  else {
    ROS_INFO("Error in getting alpha for UKF!");
    return;
  }

  if (nh.getParam("/beta", beta)) {
    ROS_INFO("Got beta for UKF!");
  }
  else {
    ROS_INFO("Error in getting beta for UKF!");
    return;
  }
  if (nh.getParam("/kappa", kappa)) {
    ROS_INFO("Got kappa for UKF!");
  }
  else {
    ROS_INFO("Error in getting kappa for UKF!");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Read IMU integration attenuation coefficient from ROS parameter server

  if (nh.getParam("/attenuation_coefficient", attenuation_coefficient)) {
    ROS_INFO("Got attenuation_coefficient !");
  }
  else {
    ROS_INFO("Error in getting attenuation coefficient for IMU integration");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Read IMU Gyro unit (bool variable)
  if (nh.getParam("/is_rad_per_sec", is_rad_per_sec)) {
    ROS_INFO("Got unit of gyroscope measurement !");
  }
  else {
    ROS_INFO("Error in getting unit of gyroscope measurement");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Get type of IMU sensor message
  if (nh.getParam("/is_sensor_msgs_imu", is_sensor_msgs_imu)) {
    ROS_INFO("Got type of IMU message !");
  }
  else {
    ROS_INFO("Error in getting type of IMU message");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Reads lidar to imu transformation parameters from ROS parameter server

  if (nh.getParam("/lidar_to_imu", lidar_to_imu)) {
    ROS_INFO("Got lidar to imu!");
  }
  else {
    ROS_INFO("Error in getting lidar to imu!");
    return;
  }

  if (lidar_to_imu.size() != 12) {
    ROS_INFO("Wrong size of lidar to imu!");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Reads TAI_offset, which is 19 seconds for Unix epoch and GPS epoch conversion

  if (nh.getParam("/TAI_offset", TAI_offset)) {
    ROS_INFO("Got TAI_offset, which is 19 seconds!");
  }
  else {
    ROS_INFO("Error in getting TAI_offset!");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Reads camera to imu transformation parameters from ROS parameter server

  if (nh.getParam("/camera_to_imu", camera_to_imu)) {
    ROS_INFO("Got camera to imu!");
  }
  else {
    ROS_INFO("Error in getting camera to imu!");
    return;
  }

  if (camera_to_imu.size() != 12) {
    ROS_INFO("Wrong size of camera to imu!");
    return;
  }

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // Get the rotation matrix from camera to gps
	if (nh.getParam("/Q_camera_2_gps", Q_camera_2_gps)) {
    	ROS_INFO("Got roation matrix from camera to gps!");
  	}
  	else {
    	ROS_INFO("Error in getting rotation matrix from camera to gps!");
    	return;
  	}

	if (Q_camera_2_gps.size() != 9) {
    ROS_INFO("Wrong size of rotation matrix from camera to gps!");
    return;
  }
}


// --------------------------------------------------------------------------------------------------------
// Checks if two poses are the same or not using timestamp and data
bool IntegratedLocalizerImpl::is_different_pose(const IntegratedLocalizer::Pose& pose_a,
                       const IntegratedLocalizer::Pose& pose_b) {
  if ( (pose_a.timestamp != pose_b.timestamp) ||
       (pose_a.ori != pose_b.ori) || (pose_a.pos != pose_b.pos) ) {
    return true;
  }
  else {
    return false;
  }
}

// --------------------------------------------------------------------------------------------------------
// Writes ros msg using pose from main localizer
void IntegratedLocalizerImpl::write_msg(geometry_msgs::PoseStamped& msg,
               const IntegratedLocalizer::Pose& pose) {
    msg.pose.position.x = pose.pos(0);
    msg.pose.position.y = pose.pos(1);
    msg.pose.position.z = pose.pos(2);

    Eigen::Quaterniond ori(pose.ori);

    //msg.pose.orientation.x = ori.vec()(0);
    //msg.pose.orientation.y = ori.vec()(1);
    //msg.pose.orientation.z = ori.vec()(2);
    //msg.pose.orientation.w = ori.w();
    msg.pose.orientation.x = pose.ori(0);
    msg.pose.orientation.y = pose.ori(1);
    msg.pose.orientation.z = pose.ori(2);
    msg.pose.orientation.w = pose.ori(3);

    double timestamp = pose.timestamp;
    int sec = static_cast<int>(timestamp);
    int nsec = static_cast<int>((timestamp - static_cast<double>(sec)) * 1e9);

    msg.header.frame_id = "map_frame";
    msg.header.stamp.sec = sec;
    msg.header.stamp.nsec = nsec;

    //std::cout << ori.w() << " " << ori.vec()(0) << " "
    //          << ori.vec()(1) << " " << ori.vec()(2) << std::endl;
}

// --------------------------------------------------------------------------------------------------------
// Writes ros msg using pose from main localizer
void IntegratedLocalizerImpl::write_geopose_msg(geographic_msgs::GeoPoseStamped& slam_geopose_msg,
               const IntegratedLocalizer::Pose& pose) {

  int sec = static_cast<int>(pose.timestamp);
	int nsec = static_cast<int>((pose.timestamp - static_cast<double>(sec)) * 1e9);
	slam_geopose_msg.header.stamp.sec = sec;
	slam_geopose_msg.header.stamp.nsec = nsec;
	slam_geopose_msg.header.frame_id = "/slam_geopose";

	// Transpose of Q_cam_2_gps
	Eigen::Matrix3d Q_G_C;
	Q_G_C(0,0) = Q_camera_2_gps[0]; // Q00
	Q_G_C(1,0) = Q_camera_2_gps[1]; // Q01
	Q_G_C(2,0) = Q_camera_2_gps[2]; // Q02
	Q_G_C(0,1) = Q_camera_2_gps[3]; // Q10
	Q_G_C(1,1) = Q_camera_2_gps[4]; // Q11
	Q_G_C(2,1) = Q_camera_2_gps[5]; // Q12
	Q_G_C(0,2) = Q_camera_2_gps[6]; // Q20
	Q_G_C(1,2) = Q_camera_2_gps[7]; // Q21
	Q_G_C(2,2) = Q_camera_2_gps[8]; // Q22

  //std::cout << " Q_G_C = " << Q_G_C << std::endl;


	// Get SLAM pose in ENU coordinate
	Eigen::Matrix3d Q_GW_SW = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Q_G_SW_0 = Eigen::Matrix3d::Identity();
	Q_G_SW_0 = Q_G_C;
	Q_GW_SW = localizerPtr->Q_GW_G_0_ * Q_G_SW_0;

  //std::cout << "Q_GW_G_0_ = " <<localizerPtr->Q_GW_G_0_ << std::endl;


	Eigen::Vector3d slam_pose_ENU = Eigen::Vector3d::Zero();
	slam_pose_ENU = Q_GW_SW * Eigen::Vector3d(pose.pos(0), pose.pos(1), pose.pos(2));

	//std::cout << " slam pose in ENU = " << slam_pose_ENU << std::endl;

	double xEast = slam_pose_ENU(0);
	double yNorth = slam_pose_ENU(1);
	double zUp = slam_pose_ENU(2);

	// Convert slam pose from ENU to Geodetic (lat, lon, alt)
	double x_ecef, y_ecef, z_ecef;
	//double lat0 = 0, lon0 = 0, h0 = 0;
	GPSUtils gpsutils;
	gpsutils.enu2Ecef(xEast, yNorth, zUp, localizerPtr->latitude0_ * M_PI / 180.0, localizerPtr->longitude0_ * M_PI / 180.0,
          localizerPtr->altitude0_,
					x_ecef, y_ecef, z_ecef);
	double lat, lon, alt;
	gpsutils.ecef2Geodetic(x_ecef, y_ecef, z_ecef, lat, lon, alt);

	//output position
	slam_geopose_msg.pose.position.latitude = lat;  // unit: degree
	slam_geopose_msg.pose.position.longitude = lon; // unit: degree
	slam_geopose_msg.pose.position.altitude = alt;  // unit: meter

	std::cout << "-----------> SLAM integrated geopose position = " << lat << " " << lon << " " << alt << std::endl;


	// Rotation matrix from Camera to SLAM World coordinate
	Eigen::Matrix3d Q_SW_C = Eigen::Matrix3d::Identity();
  Q_SW_C(0, 0) = pose.ori(0, 0);
  Q_SW_C(0, 1) = pose.ori(0, 1);
  Q_SW_C(0, 2) = pose.ori(0, 2);
  Q_SW_C(1, 0) = pose.ori(1, 0);
  Q_SW_C(1, 1) = pose.ori(1, 1);
  Q_SW_C(1, 2) = pose.ori(1, 2);
  Q_SW_C(2, 0) = pose.ori(2, 0);
  Q_SW_C(2, 1) = pose.ori(2, 1);
  Q_SW_C(2, 2) = pose.ori(2, 2);

  //std::cout << "Q_SW_C = " << Q_SW_C << std::endl;


	// Rotation matrix GW (GPS World: ENU) to GPS
	// This rotation matrix is computed from SLAM orientation, and it will be compared with
	// The GPS orientation, which is the groundtruth
	Eigen::Matrix3d Q_GW_G = Eigen::Matrix3d::Identity();
	Q_GW_G = Q_GW_SW * Q_SW_C * Q_G_C.transpose();

  Eigen::Quaterniond q(Q_GW_G);
	/*double qx = 0;
	double qy = 0;
	double qz = 0;
	double qw = 1;
	quaternionFromRotationMatrix(Q_GW_G(0, 0), Q_GW_G(0, 1), Q_GW_G(0, 2),
								Q_GW_G(1, 0), Q_GW_G(1, 1), Q_GW_G(1, 2),
								Q_GW_G(2, 0), Q_GW_G(2, 1), Q_GW_G(2, 2),
								qx, qy, qz, qw); 	*/
	slam_geopose_msg.pose.orientation.x = q.x();
	slam_geopose_msg.pose.orientation.y = q.y();
	slam_geopose_msg.pose.orientation.z = q.z();
	slam_geopose_msg.pose.orientation.w = q.w();

	std::cout << "-----------> SLAM integrated quaternion = " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

}

// --------------------------------------------------------------------------------------------------------
// Writes ros msg using pose from main localizer
/*void IntegratedLocalizerImpl::write_msg(geometry_msgs::PoseWithCovarianceStamped& msg,
               const IntegratedLocalizer::Pose& pose) {
    msg.pose.pose.position.x = pose.pos(0);
    msg.pose.pose.position.y = pose.pos(1);
    msg.pose.pose.position.z = pose.pos(2);
    //msg.pose.covariance

    Eigen::Quaterniond ori(pose.ori);

    msg.pose.pose.orientation.x = ori.vec()(0);
    msg.pose.pose.orientation.y = ori.vec()(1);
    msg.pose.pose.orientation.z = ori.vec()(2);
    msg.pose.pose.orientation.w = ori.w();

    double timestamp = pose.timestamp;
    int sec = static_cast<int>(timestamp);
    int nsec = static_cast<int>((timestamp - static_cast<double>(sec)) * 1e9);

    msg.header.stamp.sec = sec;
    msg.header.stamp.nsec = nsec;

    //std::cout << ori.w() << " " << ori.vec()(0) << " "
    //          << ori.vec()(1) << " " << ori.vec()(2) << std::endl;
}*/
