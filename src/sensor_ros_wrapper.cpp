#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>

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
            int num_points = length / (NUM_FLOATS_PER_VECTOR * sizeof(float));

            sensor_msgs::PointCloud2 msg;
            msg.header.frame_id = "lidar"; // 设置坐标系
            msg.header.stamp = ros::Time::now();
            msg.fields.resize(3);
            setup_fields(msg);

            msg.data.resize(num_points * msg.point_step);
            float* ptr = reinterpret_cast<float*>(msg.data.data());

            // 复制数据到消息
            const float* received_data = reinterpret_cast<const float*>(data);
            for (int i = 0; i < num_points; ++i) {
                ptr[i * 3]     = received_data[i * 3];         // x
                ptr[i * 3 + 1] = -received_data[i * 3 + 1]; // y (UE4使用左手坐标系)
                ptr[i * 3 + 2] = received_data[i * 3 + 2]; // z
            }

            msg.point_step = sizeof(float) * 3;
            msg.width = num_points;
            msg.height = 1;
            msg.row_step = msg.point_step * num_points;
            msg.is_dense = true;

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
