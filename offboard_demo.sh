#!/bin/bash

# 启动 QGroundControl 在第一个标签
# gnome-terminal --tab --title="QGroundControl" -- bash -c "./QGroundControl.AppImage; exec bash"
# echo "QGroundControl started!"
# sleep 5  # 等待 5 秒

# 启动 MAVROS (px.launch) 在第二个标签
gnome-terminal --tab --title="MAVROS" -- bash -c "cd FMT-Firmware_woods/fmt_ros_ws/ && source devel/setup.bash && roslaunch fmt_ros_wrapper px.launch; exec bash"
sleep 5  # 等待 5 秒 低于5s 可能无法正常启动

# 启动 QEMU 模拟器在第三个标签
gnome-terminal --tab --title="QEMU" -- bash -c "cd FMT-Firmware_woods/target/qemu/qemu-vexpress-a9/ && ./qemu.sh; exec bash"
sleep 5  # 等待 5 秒

gnome-terminal --tab --title="ROS Example" -- bash -c "cd FMT-Firmware_woods/fmt_ros_ws/ && source devel/setup.bash && roslaunch fmt_ros_wrapper define_msg_rate.launch; exec bash"
sleep 5  # 等待 5 秒


gnome-terminal --tab --title="ROS Example" -- bash -c "cd FMT-Firmware_woods/fmt_ros_ws/ && source devel/setup.bash && roslaunch fmt_ros_wrapper fmt_ros_node.launch; exec bash"

sleep 2  # 等待 5 秒

gnome-terminal --tab --title="ROS Example" -- bash -c "cd FMT-Firmware_woods/fmt_ros_ws/ && source devel/setup.bash && roslaunch fmt_ros_wrapper offboard_example.launch; exec bash"


# sleep 2  # 等待 5 秒

# gnome-terminal --tab --title="ROS Example" -- bash -c "cd FMT-Firmware_woods/omp_ws/ && source devel/setup.bash && roslaunch online_motion_planner fmt_sim.launch; exec bash"
# gnome-terminal --tab --title="ROS Example" -- bash -c "cd FMT-Firmware_woods/fmt_ros_ws/ && source devel/setup.bash && roslaunch online_motion_planner fmt_sim_dynablox.launch; exec bash"


