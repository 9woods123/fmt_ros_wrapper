#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

#define FLIGHT_ALTITUDE 3.0f
#define RATE            20   // loop rate hz
#define TOTAL_TIME      30    // time to complete one rotation in seconds
#define STEPS           (TOTAL_TIME * RATE)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spin_in_place_demo");
    ros::NodeHandle nh;

    ROS_INFO("Spin in place demo!");

    uint32_t i = 0;

    ros::Rate rate(RATE);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode set_mode;

    set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
    }

    set_mode.request.custom_mode = "AUTO.TAKEOFF";
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("Takeoff enabled");
    }

    // delay to wait takeoff finish
    ros::Duration(10).sleep();

    mavros_msgs::PositionTarget setpoint_msg;
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    while (ros::ok()) {
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFZ;

        // Set position (stay in place)
        setpoint_msg.position.x = 0.0f; // x position (fixed)
        setpoint_msg.position.y = 0.0f; // y position (fixed)
        setpoint_msg.position.z = FLIGHT_ALTITUDE; // keep altitude constant

        // Calculate the yaw for one and a half rotation (3π/2)
        float total_yaw = 2.0f * M_PI / 2; // Final yaw after one and a half rotation
        float yaw_increment = total_yaw / (STEPS / 2); // Increment yaw for half rotation
        setpoint_msg.yaw = M_PI / 2 + yaw_increment * i; // Start from π/2 and add yaw increment

        // Reset velocity and acceleration or force
        setpoint_msg.velocity.x = 0.0f;
        setpoint_msg.velocity.y = 0.0f;
        setpoint_msg.velocity.z = 0.0f;
        setpoint_msg.acceleration_or_force.x = 0.0f;
        setpoint_msg.acceleration_or_force.y = 0.0f;
        setpoint_msg.acceleration_or_force.z = 0.0f;

        setpoint_pub.publish(setpoint_msg);

        i++;
        // if (i >= STEPS) {
        //     // Set yaw to 3π/2 before exiting
        //     setpoint_msg.yaw = total_yaw; // Final yaw position
        //     setpoint_pub.publish(setpoint_msg); // Publish final position
        //     break; // stop after completing the rotation
        // }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
