#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "integrated_localizer_impl.h"
#include <iostream>


// --------------------------------------------------------------------------------------------------------
// main function
int main(int argc, char** argv) {

  ros::init(argc, argv, "integrated_localizer_node");
  ros::NodeHandle nh;

  IntegratedLocalizerImpl integrated_localizer_implementation(nh);

  // Sets publishing rate
  ros::Rate loop_rate(500);  // 500 Hz

  while (ros::ok()) {

    integrated_localizer_implementation.run();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


