#include <ros/ros.h>
#include <mavros_msgs/StreamRate.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "define_msg_rate");
    ros::NodeHandle nh;

    // 创建一个 /mavros/set_stream_rate 服务的客户端
    ros::ServiceClient stream_rate_client =
        nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");

    // 定义要更改频率的消息 ID 和对应的频率
    int message_ids[] = {31, 32};  // MAVLINK_MSG_ID_ATTITUDE_QUATERNION (31) 和 MAVLINK_MSG_ID_LOCAL_POSITION_NED (32)
    int message_rates[] = {100, 100};  // 对应的频率：50Hz for id 31, 100Hz for id 32

    // 使用 for 循环申请每个消息的频率
    for (int i = 0; i < 2; ++i) {
        mavros_msgs::StreamRate srv;
        srv.request.stream_id = message_ids[i];
        srv.request.message_rate = message_rates[i];
        srv.request.on_off = true;  // 启用消息发送

        // 调用服务
        if (stream_rate_client.call(srv)) {
            ROS_INFO("Stream rate for message %d set successfully to %d Hz!", message_ids[i], message_rates[i]);
        } else {
            ROS_ERROR("Failed to set stream rate for message %d.", message_ids[i]);
        }
    }

    return 0;
}
