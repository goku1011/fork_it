cmake_minimum_required(VERSION 2.8.3)
project(integrated_localizer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
   FILES
   Icd.msg
   Geoid.msg
   EulerAttitude.msg
   WheelSpeedReport.msg
   Kalman.msg
   StaticHeadingEvent.msg
   RawIMU.msg
   CorrectedIMU.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

find_package(Eigen3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES integrated_localizer
   CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(integrated_localizer 
src/integrated_localizer.cpp
src/integrated_localizer_impl.cpp
src/GPSUtils.cpp)

add_dependencies(integrated_localizer ${catkin_EXPORTED_TARGETS})


target_link_libraries(integrated_localizer ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/integrated_localizer_node.cpp)
add_library(${PROJECT_NAME}_nodelet src/integrated_localizer_nodelet.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  integrated_localizer
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  integrated_localizer
)
