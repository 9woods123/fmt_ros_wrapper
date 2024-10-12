#include <ros/ros.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

#define RATE 20   // 循环频率


class DroneController {
public:
    DroneController() {
        setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        cmd_sub = nh.subscribe("cmd2fmt", 10, &DroneController::cmdCallback, this); // 订阅 cmd2fmt 话题
    }

    void armAndTakeoff(float takeoff_altitude) {
        // 设置为 OFFBOARD 模式
        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = "OFFBOARD";
        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
            ROS_INFO("Offboard mode enabled");
        }

        // 设置目标位置为起飞高度
        mavros_msgs::PositionTarget setpoint_msg;
        setpoint_msg.header.stamp = ros::Time::now();
        setpoint_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        setpoint_msg.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE | mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY;

        // 只设置 z 轴（起飞高度）
        setpoint_msg.position.x = 0.0f; // 固定 x
        setpoint_msg.position.y = 0.0f; // 固定 y
        setpoint_msg.position.z = takeoff_altitude; // 起飞高度
        setpoint_msg.yaw = 3.1415926/2.0; // 使用保存的当前航向角

        // 发布起飞的 setpoint
        for (int i = 0; i < 100; ++i) {  // 发布 100 次，确保进入 OFFBOARD 模式
            setpoint_pub.publish(setpoint_msg);
            ros::Duration(0.05).sleep();
        }

        // 执行起飞
        set_mode.request.custom_mode = "AUTO.TAKEOFF";
        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
            ROS_INFO("Takeoff enabled");
        }

        ros::Duration(10).sleep(); // 等待起飞完成
    }

    void cmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg) {
        // 这里接收 cmd2fmt 话题中的位置指令并直接发布
        mavros_msgs::PositionTarget setpoint_msg = *msg;
        setpoint_pub.publish(setpoint_msg);
    }

    void spin() {
        ros::Rate rate(RATE);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Publisher setpoint_pub;
    ros::Subscriber cmd_sub;
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "drone_controller");

    DroneController drone;

    // 设置起飞高度，例如 5.0 米
    float takeoff_altitude = 3.0f;
    drone.armAndTakeoff(takeoff_altitude);  // 起飞到指定高度

    drone.spin();  // 开始订阅并发布位置信息

    return 0;
}
