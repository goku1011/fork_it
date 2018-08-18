#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "integrated_localizer_impl.h"

#include <thread>
#include <iostream>


namespace integrated_localizer
{

// --------------------------------------------------------------------------------------------------------
class integrated_localizer_nodelet: public nodelet::Nodelet{
  public:
    integrated_localizer_nodelet() {}
    ~integrated_localizer_nodelet() {}

    virtual void onInit();
    void nodelet_run();
    boost::shared_ptr<IntegratedLocalizerImpl> integrated_localizer_impl_Ptr_;
    std::thread nodelet_run_thread_;

};

// --------------------------------------------------------------------------------------------------------
// main function
void integrated_localizer_nodelet::onInit() {

  NODELET_DEBUG("Initializing nodelet...");
  std::cout << "test nodelet !!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  integrated_localizer_impl_Ptr_.reset(new IntegratedLocalizerImpl(getNodeHandle()));

  std::cout << "test nodelet !!!!" << std::endl;

  nodelet_run_thread_ = std::thread(&integrated_localizer_nodelet::nodelet_run, this);

  /*
  // Sets publishing rate
  ros::Rate loop_rate(500);  // 500 Hz

  while (ros::ok()) {

    //integrated_localizer_implementation.run();
    integrated_localizer_impl_Ptr_->run();
    ros::spinOnce();
    loop_rate.sleep();
  }
  */

}


// --------------------------------------------------------------------------------------------------------
void integrated_localizer_nodelet::nodelet_run() {
  // Sets publishing rate
  ros::Rate loop_rate(500);  // 500 Hz

  while (ros::ok()) {

    //integrated_localizer_implementation.run();
    integrated_localizer_impl_Ptr_->run();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
  
}

// --------------------------------------------------------------------------------------------------------
//Register this plugin with pluginlib. Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(integrated_localizer::integrated_localizer_nodelet, nodelet::Nodelet)


