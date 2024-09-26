// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <vector>
// #include <iostream>
// #include <netinet/in.h>
// #include <arpa/inet.h>

// #define UDP_PORT 14556
// #define NUM_FLOATS_PER_VECTOR 3 // 只包含 x, y, z



// int main(int argc, char** argv) {
//     ros::init(argc, argv, "point_cloud_publisher");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10);

//     // 设置 UDP 套接字
//     int sockfd;
//     struct sockaddr_in serv_addr, cli_addr;
//     socklen_t clilen = sizeof(cli_addr);
//     sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//     if (sockfd < 0) {
//         ROS_ERROR("ERROR opening socket");
//         return -1;
//     }

//     bzero((char*)&serv_addr, sizeof(serv_addr));
//     serv_addr.sin_family = AF_INET;
//     serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
//     serv_addr.sin_port = htons(UDP_PORT);
//     if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
//         ROS_ERROR("ERROR on binding");
//         return -1;
//     }


//     while (ros::ok()) {
//         // 接收 UDP 数据
//         char buffer[1024 * 100];
//         int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);
//         if (n < 0) {
//             ROS_ERROR("ERROR in recvfrom");
//             continue;
//         }

//         // 解析 x, y, z 数据
//         int num_points = n / (NUM_FLOATS_PER_VECTOR * sizeof(float));
//         std::cout<<"num_points: "<<num_points<<std::endl;
        
//         sensor_msgs::PointCloud2 msg;
//         msg.header.frame_id = "drone";
//         msg.header.stamp = ros::Time::now();

//         // 填充消息字段
//         const uint32_t POINT_STEP = 12; // 每个点 3 个 float (x, y, z)
//         msg.fields.resize(3);
//         msg.fields[0].name = "x";
//         msg.fields[0].offset = 0;
//         msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
//         msg.fields[0].count = 1;
//         msg.fields[1].name = "y";
//         msg.fields[1].offset = 4;
//         msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
//         msg.fields[1].count = 1;
//         msg.fields[2].name = "z";
//         msg.fields[2].offset = 8;
//         msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
//         msg.fields[2].count = 1;
//         msg.data.resize(num_points * POINT_STEP);

//         uint8_t *ptr = msg.data.data();

        
//         for (int i = 0; i < num_points; ++i) {
//             float x, y, z;
//             memcpy(&x, &buffer[i * NUM_FLOATS_PER_VECTOR * sizeof(float)], sizeof(float));
//             memcpy(&y, &buffer[i * NUM_FLOATS_PER_VECTOR * sizeof(float) + sizeof(float)], sizeof(float));
//             memcpy(&z, &buffer[i * NUM_FLOATS_PER_VECTOR * sizeof(float) + 2 * sizeof(float)], sizeof(float));

//             *((float*)(ptr + 0)) = x; // x C++ 中，float 类型的标准大小通常是 4 字节。
//             *((float*)(ptr + 4)) = -y; // y  ue中是左手系
//             *((float*)(ptr + 8)) = z; // z
//             ptr += POINT_STEP;
//         }

//         msg.point_step = POINT_STEP;
//         msg.is_bigendian = false;
//         msg.width = msg.data.size() / POINT_STEP;
//         msg.height = 1;
//         msg.row_step = msg.data.size();
//         msg.is_dense = true;

//         // 发布点云消息
//         pub.publish(msg);
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/asio.hpp>

using boost::asio::ip::udp;


class PointCloudReceiver {
public:
    PointCloudReceiver(ros::NodeHandle& nh) 
        : socket_(io_service_, udp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 14556)) // 
    {
        pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 10);
        start_receive();

    }

    void run() {
        io_service_.run(); // 启动事件循环
    }

private:
    void start_receive() {
        socket_.async_receive_from(
            boost::asio::buffer(receive_buffer_), sender_endpoint_,
            boost::bind(&PointCloudReceiver::handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred) {

        if (!error) {
            parse_point_cloud(receive_buffer_, bytes_transferred); // 使用数组名
            start_receive(); // 继续接收
        }


    }

    void parse_point_cloud(const uint8_t* data, std::size_t length) {

        // 假设每个点由3个浮点数（x, y, z）组成
        size_t num_points = length / (3 * sizeof(float));

        sensor_msgs::PointCloud2 pointcloud_msg;
        pointcloud_msg.header.stamp = ros::Time::now();
        pointcloud_msg.header.frame_id = "lidar"; // 设置坐标系
        pointcloud_msg.height = 1; // 一维点云
        pointcloud_msg.width = num_points;
        pointcloud_msg.is_dense = true; // 无效点云

        // 设置点云的字段
        sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
        modifier.setPointCloud2Fields(3, 
            "x", 1, sensor_msgs::PointField::FLOAT32,
            "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32
        );

        // 分配内存
        pointcloud_msg.data.resize(num_points * sizeof(float) * 3);
        pointcloud_msg.point_step = sizeof(float) * 3; // 每个点的字节数
        pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width; // 行步长

        // 将数据复制到 PointCloud2 消息中
        float* point_data = reinterpret_cast<float*>(pointcloud_msg.data.data());
        const float* received_data = reinterpret_cast<const float*>(data);
        for (size_t i = 0; i < num_points; ++i) {
            point_data[i * 3]     = received_data[i * 3];     // x
            point_data[i * 3 + 1] = -received_data[i * 3 + 1]; // y 需要根据你的坐标系调整
            point_data[i * 3 + 2] = received_data[i * 3 + 2]; // z
        }

        pointcloud_pub_.publish(pointcloud_msg);
        
    }

    boost::asio::io_service io_service_;
    udp::socket socket_;
    udp::endpoint sender_endpoint_;
    uint8_t receive_buffer_[4096]; // 接收缓冲区
    ros::Publisher pointcloud_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "udp_point_cloud_receiver");
    ros::NodeHandle nh;

    PointCloudReceiver receiver(nh);
    receiver.run(); // 启动事件循环
    ros::spin();
    return 0;
}
