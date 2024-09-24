#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>

#define UDP_PORT 14559
#define NUM_FLOATS_PER_VECTOR 7 // x, y, z, q.w, q.x, q.y, q.z

int main(int argc, char** argv) {
    ros::init(argc, argv, "transform_publisher");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster br;

    // 设置 UDP 套接字
    int sockfd;
    struct sockaddr_in serv_addr, cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("ERROR opening socket");
        return -1;
    }
    bzero((char*)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(UDP_PORT);
    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR("ERROR on binding");
        return -1;
    }

    ros::Rate rate(100.0);


    while (ros::ok()) {

        // 接收 UDP 数据
        char buffer[1024 * 100];
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);
        if (n < 0) {
            ROS_ERROR("ERROR in recvfrom");
            continue;
        }

        // 解析 x, y, z, q.w, q.x, q.y, q.z 数据
        float x, y, z, q_w, q_x, q_y, q_z;
        memcpy(&x, &buffer[0], sizeof(float));
        memcpy(&y, &buffer[sizeof(float)], sizeof(float));
        memcpy(&z, &buffer[2 * sizeof(float)], sizeof(float));
        memcpy(&q_w, &buffer[3 * sizeof(float)], sizeof(float));
        memcpy(&q_x, &buffer[4 * sizeof(float)], sizeof(float));
        memcpy(&q_y, &buffer[5 * sizeof(float)], sizeof(float));
        memcpy(&q_z, &buffer[6 * sizeof(float)], sizeof(float));

        // 创建 TransformStamped 消息
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();

        // transformStamped.header.frame_id = "base_link"; // 参考坐标系
        // transformStamped.child_frame_id = "lidar"; // 子坐标系
        transformStamped.header.frame_id = "base_link"; // 参考坐标系
        transformStamped.child_frame_id = "lidar"; // 子坐标系

        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = z;
        transformStamped.transform.rotation.w = q_w;
        transformStamped.transform.rotation.x = q_x;
        transformStamped.transform.rotation.y = q_y;
        transformStamped.transform.rotation.z = q_z;

        // 广播 transform
        br.sendTransform(transformStamped);
                // 处理回调
        rate.sleep();  
    
    }

    return 0;
}
