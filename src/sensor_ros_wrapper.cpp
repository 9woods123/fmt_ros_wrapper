#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <iostream>
#include <netinet/in.h>
#include <arpa/inet.h>

#define UDP_PORT 14556
#define NUM_FLOATS_PER_VECTOR 3 // 只包含 x, y, z



int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

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

    while (ros::ok()) {
        // 接收 UDP 数据
        char buffer[1024 * 100];
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);
        if (n < 0) {
            ROS_ERROR("ERROR in recvfrom");
            continue;
        }

        // 解析 x, y, z 数据
        int num_points = n / (NUM_FLOATS_PER_VECTOR * sizeof(float));

        sensor_msgs::PointCloud2 msg;
        msg.header.frame_id = "lidar";
        msg.header.stamp = ros::Time::now();

        // 填充消息字段
        const uint32_t POINT_STEP = 12; // 每个点 3 个 float (x, y, z)
        msg.fields.resize(3);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[0].count = 1;
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[1].count = 1;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[2].count = 1;
        msg.data.resize(num_points * POINT_STEP);

        uint8_t *ptr = msg.data.data();
        for (int i = 0; i < num_points; ++i) {
            float x, y, z;
            memcpy(&x, &buffer[i * NUM_FLOATS_PER_VECTOR * sizeof(float)], sizeof(float));
            memcpy(&y, &buffer[i * NUM_FLOATS_PER_VECTOR * sizeof(float) + sizeof(float)], sizeof(float));
            memcpy(&z, &buffer[i * NUM_FLOATS_PER_VECTOR * sizeof(float) + 2 * sizeof(float)], sizeof(float));

            *((float*)(ptr + 0)) = x; // x C++ 中，float 类型的标准大小通常是 4 字节。
            *((float*)(ptr + 4)) = y; // y
            *((float*)(ptr + 8)) = z; // z
            ptr += POINT_STEP;
        }

        msg.point_step = POINT_STEP;
        msg.is_bigendian = false;
        msg.width = msg.data.size() / POINT_STEP;
        msg.height = 1;
        msg.row_step = msg.data.size();
        msg.is_dense = true;

        // 发布点云消息
        pub.publish(msg);
    }

    return 0;
}
