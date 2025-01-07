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
  image_transport
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
 image_transport
)





include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
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

add_executable(transform_ros_wrapper src/transform_ros_wrapper.cpp)
target_link_libraries(transform_ros_wrapper
  ${catkin_LIBRARIES}
)


add_executable(cmd2fmt src/cmd2fmt.cpp)
target_link_libraries(cmd2fmt
  ${catkin_LIBRARIES} 
)


add_library(image_udp_receiver src/image_udp_receiver.cpp)
target_link_libraries(image_udp_receiver
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(image_ros_wrapper src/image_ros_wrapper.cpp)
target_link_libraries(image_ros_wrapper
  image_udp_receiver
)



# offboard example

add_executable(car_offboard src/offboard_example/car_offboard.cpp)
target_link_libraries(car_offboard
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(boat_offboard src/offboard_example/boat_offboard.cpp)
target_link_libraries(boat_offboard
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(submarine_offboard src/offboard_example/submarine_offboard.cpp)
target_link_libraries(submarine_offboard
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(drone_offboard src/offboard_example/drone_offboard.cpp)
target_link_libraries(drone_offboard
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
