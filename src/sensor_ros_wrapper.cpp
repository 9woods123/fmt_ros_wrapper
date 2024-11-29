#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


#define UDP_PORT 14556
#define NUM_FLOATS_PER_VECTOR 3 // 每个点包含 x, y, z
#define UDP_MTU 1000

// 数据包头部定义
struct PacketHeader {
    uint32_t seq_num;            // 当前包在整段数据中的位置
    uint32_t total_pkgs_num;      // 总包数
    bool is_last;     // 
    uint32_t data_len; // 当前包的数据长度
};

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
    uint32_t total_packets_expected;
    // 分包缓冲区，按 seq_num 存储对应的包
    std::map<uint32_t, std::vector<uint8_t>> packet_buffer;

    void setup_socket() {
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            ROS_ERROR("ERROR opening socket");
            exit(-1);
        }

        bzero((char*)&serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        serv_addr.sin_port = htons(UDP_PORT);

        // struct timeval timeout;
        // timeout.tv_sec = 1;  // 1 秒超时
        // timeout.tv_usec = 0;
        // setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            ROS_ERROR("ERROR on binding");
            exit(-1);
        }


    }

    void receive_data() {


    auto start = std::chrono::high_resolution_clock::now();
        char buffer[UDP_MTU*1000];

        std::cout<<"=========================="<<std::endl;
        // 接收数据包
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);

        if (n < sizeof(PacketHeader)) {
            ROS_WARN("Received data is too short to contain header.");
            return;
        }

        std::cout<<"== sizeof(PacketHeader)="<< sizeof(PacketHeader)<<std::endl;

        // 解析包头
        PacketHeader header;
        std::memcpy(&header, buffer, sizeof(PacketHeader));


        // 如果接收到新的包组，清空缓存并更新总包数
        if (header.seq_num == 0 ) {
            packet_buffer.clear();  // 清空缓存
            total_packets_expected = header.total_pkgs_num;  // 更新组的总包数
        }

        std::vector<uint8_t> packet_data(buffer + sizeof(PacketHeader), buffer + sizeof(PacketHeader) + header.data_len);
        packet_buffer[header.seq_num] = packet_data;

    // 检查是否收到所有数据包
        if (packet_buffer.size() == total_packets_expected) {
            // 重组数据包
            std::vector<char> complete_data;
            for (int i = 0; i < total_packets_expected; i++) {
                if (packet_buffer.find(i) != packet_buffer.end()) {
                    complete_data.insert(complete_data.end(), packet_buffer[i].begin(), packet_buffer[i].end());
                } else {
                    // 如果缺少某个包，等待该包的到来
                    std::cout << "Missing packet " << i << ", waiting for retransmission." << std::endl;
                    return;
                }
            }
            publish_point_cloud(complete_data.data(), complete_data.size());            // 重组完成，处理数据
            packet_buffer.clear();  // 清空缓存
            total_packets_expected = 0;  // 重置
        }

            auto end = std::chrono::high_resolution_clock::now();

    // 计算执行时间
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Data received and processed in: " << 1000*elapsed.count() << " ms." << std::endl;
    // 重组 耗时 1ms内。
    }

    void publish_point_cloud(const char* data, int length) {
        if (length < NUM_FLOATS_PER_VECTOR * sizeof(float)) {
            ROS_WARN("Received data is too short to contain any points.");
            return;
        }

        int num_points = length / (NUM_FLOATS_PER_VECTOR * sizeof(float));

        sensor_msgs::PointCloud2 msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = ros::Time::now() - ros::Duration(0.01);
        msg.fields.resize(3);
        setup_fields(msg);

        msg.point_step = sizeof(float) * NUM_FLOATS_PER_VECTOR;
        msg.width = num_points;
        msg.height = 1;
        msg.row_step = msg.point_step * num_points;
        msg.is_dense = true;
        msg.data.resize(num_points * msg.point_step);

        float* ptr = reinterpret_cast<float*>(msg.data.data());
        const float* received_data = reinterpret_cast<const float*>(data);
        for (int i = 0; i < num_points; ++i) {
            ptr[i * 3] = received_data[i * 3];          // x
            ptr[i * 3 + 1] = -received_data[i * 3 + 1]; // y (UE4 -> ROS 坐标转换)
            ptr[i * 3 + 2] = received_data[i * 3 + 2];  // z
        }
        pub.publish(msg);
    }

    void setup_fields(sensor_msgs::PointCloud2& msg) {
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(3,
                                       "x", 1, sensor_msgs::PointField::FLOAT32,
                                       "y", 1, sensor_msgs::PointField::FLOAT32,
                                       "z", 1, sensor_msgs::PointField::FLOAT32);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    PointCloudPublisher publisher(nh);
    publisher.run();

    return 0;
}
