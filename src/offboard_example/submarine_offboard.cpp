#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/Waypoint.h>

#define RATE            20   // loop rate hz
#define CYCLE_S         30   // time to complete one figure 8 cycle in seconds
#define STEPS           (CYCLE_S * RATE)

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Submarine Offboard Demo");
    ros::NodeHandle nh;
    
    ROS_INFO("Submarine Offboard Demo!");

    const float PI = 3.14159265359;
    const float dt = 1.0f / RATE;
    uint32_t i = 0;
    float dir = 1.0;

    ros::Rate rate(RATE);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode set_mode;

    set_mode.request.custom_mode = "OFFBOARD";
    // Set the mode to OFFBOARD using MAVROS service
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
    }

    // Create service client for arming
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  // true for unlocking, false for locking
    arming_client.call(arm_cmd);

    // Delay for 1 second
    ros::Duration(1).sleep();

    mavros_msgs::PositionTarget setpoint_msg;
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    while (ros::ok()) {
        if (i == 0) {
            dir *= -1;  // Toggle direction
        }

        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.coordinate_frame = mavros_msgs::Waypoint::FRAME_BODY_FRD;  // Update to use correct coordinate frame
        setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                                 mavros_msgs::PositionTarget::IGNORE_PY | 
                                 mavros_msgs::PositionTarget::IGNORE_PZ | 
                                 mavros_msgs::PositionTarget::IGNORE_VY | 
                                 mavros_msgs::PositionTarget::IGNORE_YAW | 
                                 mavros_msgs::PositionTarget::IGNORE_AFX | 
                                 mavros_msgs::PositionTarget::IGNORE_AFY | 
                                 mavros_msgs::PositionTarget::IGNORE_AFZ;

        // Set velocities (submarine-specific)
        
        setpoint_msg.velocity.y = 2.5;
        setpoint_msg.velocity.z = -0.5 * dir;  // Up and down direction based on 'dir'

        // Set yaw rate (based on 'dir')
        setpoint_msg.yaw_rate = 50 * PI / 360.0f * dir;

        setpoint_pub.publish(setpoint_msg);

        ros::spinOnce();
        rate.sleep();
        
        i = (i + 1) % STEPS;
    }

    return 0;
}
