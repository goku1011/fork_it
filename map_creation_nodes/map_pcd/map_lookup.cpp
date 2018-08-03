#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <Eigen/Geometry>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef USE_FAST_PCL
#include <fast_pcl/registration/ndt.h>
#include <fast_pcl/filters/voxel_grid.h>
#else
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#endif

#include <autoware_msgs/ConfigApproximateNdtMapping.h>
#include <autoware_msgs/ConfigNdtMappingOutput.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

struct Pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

// global variables
static Pose previous_pose, guess_pose, guess_pose_imu, guess_pose_odom, guess_pose_imu_odom, current_pose_imu, current_pose_odom, current_pose_imu_odom, ndt_pose, added_pose, localizer_pose;
static Pose starting_point, current_pose;
static ros::Time current_scan_time;
static ros::Time previous_scan_time;
static ros::Duration scan_duration;

static double diff = 0.0;
static double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw; // current_pose - previous_pose
static double offset_imu_x, offset_imu_y, offset_imu_z, offset_imu_roll, offset_imu_pitch, offset_imu_yaw;
static double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;
static double offset_imu_odom_x, offset_imu_odom_y, offset_imu_odom_z, offset_imu_odom_roll, offset_imu_odom_pitch, offset_imu_odom_yaw;

static double current_velocity_x = 0.0;
static double current_velocity_y = 0.0;
static double current_velocity_z = 0.0;

static double current_velocity_imu_x = 0.0;
static double current_velocity_imu_y = 0.0;
static double current_velocity_imu_z = 0.0;

static pcl::PointCloud<pcl::PointXYZI> map, submap;

static pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
// Default values
static int max_iter = 30;            // Maximum iterations
static float ndt_res = 1.0;      // Resolution
static double step_size = 0.1;   // Step size
static double trans_eps = 0.01;  // Transformation epsilon


static int flag = 1;
static int set_starting_point = 0;


// Leaf size of VoxelGrid filter.
static double voxel_leaf_size = 2.0;

static ros::Time callback_start, callback_end, t1_start, t1_end, t2_start, t2_end, t3_start, t3_end, t4_start, t4_end,
    t5_start, t5_end;
static ros::Duration d_callback, d1, d2, d3, d4, d5;

static ros::Publisher map_pub;
static ros::Publisher current_pose_pub;
static ros::Publisher guess_pose_linaer_pub;
static geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;

static ros::Publisher ndt_stat_pub;
static std_msgs::Bool ndt_stat_msg;

static int initial_scan_loaded = 0;
static int intial_pose_loaded = 0;

static Eigen::Matrix4f gnss_transform = Eigen::Matrix4f::Identity();

static double min_scan_range = 5.0;
static double min_add_scan_shift = 1.0;
static double max_submap_size = 5.0;

static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static Eigen::Matrix4f tf_btol, tf_ltob;

static bool isMapUpdate = true;
static bool _use_openmp = false;
static bool _use_imu = false;
static bool _use_odom = false;
static bool _imu_upside_down = false;

static std::string _imu_topic = "/imu_raw";


static double fitness_score;

static int submap_num = 0;
static double submap_size = 0.0;

static sensor_msgs::Imu imu;
static nav_msgs::Odometry odom;


Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rx * ry * rz;
}


void callback(const sensor_msgs::PointCloud2::ConstPtr& input, const geometry_msgs::PoseStamped::ConstPtr& current_pose){

  // Transformation Matrix


  listener.lookupTransform("/odom", "/base_laser", ros::Time(0), transform);

  pcl_ros::transformPointCloud(original_PC, transformed_PC, transform_matrix)


  Eigen::Matrix4f m1 = m.cast<float>();
  //std::cout<<m<<std::endl;
  //std::cout<<q.toRotationMatrix()<<std::endl<<std::endl;

  //std::cout<<"Pose "<<current_pose->header.stamp.sec<<" "<<current_pose->header.stamp.nsec<<std::endl;
  //std::cout<<"Velodyne "<<input->header.stamp.sec<<" "<<input->header.stamp.nsec<<std::endl;
  std::cout<<"Difference "<<abs(current_pose->header.stamp.nsec - input->header.stamp.nsec)<<std::endl<<std::endl;

  std::cout<<current_pose->pose.position.x<<std::endl;
  std::cout<<current_pose->pose.position.y<<std::endl;
  std::cout<<current_pose->pose.position.z<<std::endl;

  //double r;
  pcl::PointXYZI p;
  pcl::PointCloud<pcl::PointXYZI> tmp, scan, tmp_out;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  pcl::fromROSMsg(*input, tmp);

  sensor_msgs::PointCloud2::Ptr scan_ptr(new sensor_msgs::PointCloud2);
  //sensor_msgs::PointCloud2 scan_ptr;
  pcl_ros::transformPointCloud(m1, *input, *scan_ptr);
  //pcl::transformPointCloud(tmp, tmp_out, m1);


  pcl::fromROSMsg(*scan_ptr, *transformed_scan_ptr);

  if (initial_scan_loaded == 0)
  {
    map += *transformed_scan_ptr;
    initial_scan_loaded = 1;
  }

  map += *transformed_scan_ptr;

  sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(map, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "mp";
  map_pub.publish(*map_msg_ptr);
  //map_pub.publish(*scan_ptr);

  std::string s1 = "map_";
  std::string s2 = std::to_string(submap_num);
  std::string s3 = ".pcd";
  std::string pcd_filename = s1 + s2 + s3;

  std::string s11 = "map_pri";
  std::string s21 = std::to_string(submap_num);
  std::string s31 = ".pcd";
  std::string pcd_filename_prior = s11 + s21 + s31;
/*
  if(map.size() != 0)
  {
    if(pcl::io::savePCDFileBinary(pcd_filename, map) == -1){
      std::cout << "Failed saving " << pcd_filename << "." << std::endl;
    }
    std::cout << "Saved " << pcd_filename << " (" << map.size() << " points)" << std::endl;
  }

  submap_num++;
*/
  if((*transformed_scan_ptr).size() != 0)
  {
    if(pcl::io::savePCDFileBinary(pcd_filename, *transformed_scan_ptr) == -1){
      std::cout << "Failed saving " << pcd_filename << "." << std::endl;
    }
    //std::cout << "Saved " << pcd_filename << " (" << (*transformed_scan_ptr).size() << " points)" << std::endl;
  }

  if(tmp.size() != 0)
  {
    if(pcl::io::savePCDFileBinary(pcd_filename_prior, tmp) == -1){
      std::cout << "Failed saving " << pcd_filename_prior << "." << std::endl;
    }
    //std::cout << "Saved " << pcd_filename_prior << " (" << tmp.size() << " points)" << std::endl;
  }
  submap_num++;

/*
  if(submap_size >= max_submap_size){
    std::string s1 = "submap_";
    std::string s2 = std::to_string(submap_num);
    std::string s3 = ".pcd";
    std::string pcd_filename = s1 + s2 + s3;
    std::cout<<submap_num<<std::endl;
    if(submap.size() != 0)
    {
      if(pcl::io::savePCDFileBinary(pcd_filename, submap) == -1){
        std::cout << "Failed saving " << pcd_filename << "." << std::endl;
  		}
  		std::cout << "Saved " << pcd_filename << " (" << submap.size() << " points)" << std::endl;

  		map = submap;
  		submap.clear();
  		submap_size = 0.0;
  	}
  	submap_num++;
  }
  */
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_pose");

  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_data_(nh, "/velodyne_points", 1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> pose_data_(nh, "/gnss_pose", 1);

  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/project_map", 1000);

  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), point_cloud_data_, pose_data_);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}
