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
    ros::init(argc, argv, "Car Offboard Demo");
    ros::NodeHandle nh;
    
    ROS_INFO("Car Offboard Demo!");

	const float PI = 3.14159265359;
    const float dt = 1.0f / RATE;
    uint32_t i = 0;

    ros::Rate rate(RATE);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode set_mode;

    set_mode.request.custom_mode = "OFFBOARD";
    // Set the mode to OFFBOARD using MAVROS service
    if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
        ROS_INFO("Offboard mode enabled");
    }

    // 创建服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // 创建服务请求和响应对象
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  // true表示解锁，false表示加锁
	// 发送解锁请求
	arming_client.call(arm_cmd);

    // delay 1s
    ros::Duration(2).sleep();

    mavros_msgs::PositionTarget setpoint_msg;
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);


    while (ros::ok()) {
        std::cout<<"========================"<<std::endl;
        

        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.coordinate_frame =mavros_msgs::Waypoint::FRAME_BODY_FRD;;
        setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                                mavros_msgs::PositionTarget::IGNORE_PY |
                                mavros_msgs::PositionTarget::IGNORE_PZ | 								
                                
                                mavros_msgs::PositionTarget::IGNORE_VZ | 
                                mavros_msgs::PositionTarget::IGNORE_VY | 

                                mavros_msgs::PositionTarget::IGNORE_YAW | 
								mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY | 
                                mavros_msgs::PositionTarget::IGNORE_AFZ;

        // mcn echo auto_cmd

        // Set velocity
        setpoint_msg.velocity.y = 3.0;

        // Set yaw rare
        // setpoint_msg.yaw = -100 * PI / 360.0f;
        
        setpoint_msg.yaw_rate = -100 * PI / 360.0f;

        setpoint_pub.publish(setpoint_msg);

        ros::spinOnce();
        rate.sleep();
		
		i = (i + 1) % STEPS;
    }

    return 0;
}
