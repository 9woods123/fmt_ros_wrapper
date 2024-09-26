cmake_minimum_required(VERSION 3.0.2)
project(fmt_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  rospy
  std_msgs
  roscpp
)

find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS system filesystem)

catkin_package(

CATKIN_DEPENDS 
 mavros 
 mavros_msgs 
 rospy
 std_msgs
 sensor_msgs
 std_msgs
 pcl_conversions
 pcl_ros
 geometry_msgs
 tf
)



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)



add_executable(flying_example src/flying_example.cpp)
target_link_libraries(flying_example
  ${catkin_LIBRARIES}
)



add_executable(define_msg_rate src/define_msg_rate.cpp)
target_link_libraries(define_msg_rate
  ${catkin_LIBRARIES}
)


add_executable(sensor_ros_wrapper src/sensor_ros_wrapper.cpp)
target_link_libraries(sensor_ros_wrapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(transform_ros_wrapper src/transform_ros_wrapper.cpp)
target_link_libraries(transform_ros_wrapper
  ${catkin_LIBRARIES}
)


add_executable(gettf_from_mavros src/gettf_from_mavros.cpp)
target_link_libraries(gettf_from_mavros
  ${catkin_LIBRARIES} 
)




