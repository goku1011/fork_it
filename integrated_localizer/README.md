# Mapping-Localization
Linear Kalman Filter

Model 

Integrated Pose @ 110 Hz.
Interpolates along the accurate NDT profile. i.e., makes the 10 Hz NDT profile to 110 Hz.
(Since the NDT pose data is more accurate than GNSS pose, hence the linear kalman filter is used to interpolate for higher frequency along the NDT pose profile)



Run the launch file -
roslaunch integrated_localizer integrated_localizer.launch

Run the sample rosbag file - 
rosbag play bag_with_gnss_ndt.bag



Uses topic /imu/data for prediction step.
Uses /gnss_pose & /ndt_pose for update step.
Outputs /integrated_pose as output pose.


Visualize the pose data using the command -
python plot.py





