cmake_minimum_required(VERSION 3.0.2)
project(fmt_ros_wrapper)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  rospy
  std_msgs
  roscpp
  tf
  cv_bridge
)

find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenCV REQUIRED)

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
 cv_bridge
)





include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(example src/example.cpp)
target_link_libraries(example
  ${catkin_LIBRARIES}
)



add_executable(define_msg_rate src/define_msg_rate.cpp)
target_link_libraries(define_msg_rate
  ${catkin_LIBRARIES}
)

## 不分包
# add_executable(sensor_ros_wrapper src/sensor_ros_wrapper_origin.cpp)
# target_link_libraries(sensor_ros_wrapper
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )


# 分包重组，仍然存在效率问题
add_executable(sensor_ros_wrapper src/sensor_ros_wrapper.cpp)
target_link_libraries(sensor_ros_wrapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(img_ros_wrapper src/img_ros_wrapper.cpp)
target_link_libraries(img_ros_wrapper
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


add_executable(transform_ros_wrapper src/transform_ros_wrapper.cpp)
target_link_libraries(transform_ros_wrapper
  ${catkin_LIBRARIES}
)


add_executable(gettf_from_mavros src/gettf_from_mavros.cpp)
target_link_libraries(gettf_from_mavros
  ${catkin_LIBRARIES} 
)


add_executable(cmd2fmt src/cmd2fmt.cpp)
target_link_libraries(cmd2fmt
  ${catkin_LIBRARIES} 
)


