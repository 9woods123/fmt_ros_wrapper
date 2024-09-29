#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define UDP_PORT 14556
#define NUM_FLOATS_PER_VECTOR 3 // 每个点只包含 x, y, z

class PointCloudPublisher {
        
    public:
    
        PointCloudPublisher(ros::NodeHandle& nh)
            : pub(nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 10)) {
            setup_socket();
        }

        void run() {
            while (ros::ok()) {
                receive_data();
            }
        }

    private:
        ros::Publisher pub;
        int sockfd;
        struct sockaddr_in serv_addr, cli_addr;
        socklen_t clilen = sizeof(cli_addr);

        void setup_socket() {
            // 创建套接字
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                ROS_ERROR("ERROR opening socket");
                exit(-1);
            }

            // 初始化服务器地址结构
            bzero((char*)&serv_addr, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
            serv_addr.sin_port = htons(UDP_PORT);

            // 绑定套接字
            if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
                ROS_ERROR("ERROR on binding");
                exit(-1);
            }
        }

        void receive_data() {
            char buffer[1024 * 100]; // 接收缓冲区
            int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);
            if (n < 0) {
                ROS_ERROR("ERROR in recvfrom");
                return;
            }

            publish_point_cloud(buffer, n);
        }


        void publish_point_cloud(const char* data, int length) {
            // 确保数据长度足够包含时间戳
            if (length < sizeof(float)) {
                ROS_WARN("Received data is too short to contain a timestamp.");
                return;
            }

            // 计算点的数量
            int num_points = (length - sizeof(float)) / (NUM_FLOATS_PER_VECTOR * sizeof(float)); // 减去时间戳的长度

            // 解析时间戳
            const float* receivedData = reinterpret_cast<const float*>(data); // 将数据转换为 const float*
            float timestamp = *receivedData; // 第一个浮点数是时间戳

            // 创建 PointCloud2 消息
            sensor_msgs::PointCloud2 msg;
            msg.header.frame_id = "base_link"; // 设置坐标系
            msg.header.stamp = ros::Time(timestamp); // 设置消息的时间戳

            std::cout<<"timestamp ORIGIN"<<timestamp<<std::endl;
            std::cout<<"time now ORIGIN"<<ros::Time::now().toSec()<<std::endl;

            msg.fields.resize(3);
            setup_fields(msg);
            msg.point_step = sizeof(float) * 3; // 每个点的字节数
            msg.width = num_points; // 点的数量
            msg.height = 1; // 高度设置为1，表示一维点云
            msg.row_step = msg.point_step * num_points; // 每行的字节数
            msg.is_dense = true; // 数据是否是密集的

            // 确保消息数据大小正确
            msg.data.resize(num_points * msg.point_step);

            // 复制数据到消息
            float* ptr = reinterpret_cast<float*>(msg.data.data());
            const float* received_data = reinterpret_cast<const float*>(data + sizeof(float)); // 跳过时间戳
            for (int i = 0; i < num_points; ++i) {
                ptr[i * 3]     = received_data[i * 3];         // x
                ptr[i * 3 + 1] = -received_data[i * 3 + 1]; // y (UE4使用左手坐标系)
                ptr[i * 3 + 2] = received_data[i * 3 + 2];     // z
            }

            // 发布消息
            pub.publish(msg);
        }



        void setup_fields(sensor_msgs::PointCloud2& msg) {

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(3, 
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32
            );
        }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    PointCloudPublisher publisher(nh);
    publisher.run();

    return 0;
}
