#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>

#define UDP_PORT 14600
#define UDP_MTU 1000

// 图像数据包头部定义
struct CameraPacketHeader {
    uint32_t seq_num;      // 当前包在整段数据中的位置
    uint32_t total_pkgs_num;  // 总包数
    bool is_last;           // 是否为最后一包
    uint32_t data_len;      // 当前包的数据长度
};

// 图像元数据
struct ImageMetaData {
    uint32_t head;     // 图像头部标识符，值为 0xFF55FF55
    uint32_t width;    // 图像宽度
    uint32_t height;   // 图像高度
};

class CameraPublisher {
public:
    CameraPublisher(ros::NodeHandle& nh)
        : image_pub(nh.advertise<sensor_msgs::Image>("rgb_img", 10)) {
        setup_socket();
    }

    void run() {
        while (ros::ok()) {
            receive_data();
        }
    }

private:
    ros::Publisher image_pub;
    int sockfd;
    struct sockaddr_in serv_addr, cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    uint32_t total_packets_expected;
    // 分包缓冲区，按 seq_num 存储对应的包
    std::map<uint32_t, std::vector<uint8_t>> packet_buffer;
    ImageMetaData img_meta_data;  // 存储图像的元数据

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

        if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
            ROS_ERROR("ERROR on binding");
            exit(-1);
        }
    }

    void receive_data() {

        char buffer[UDP_MTU * 10000];

        // 接收数据包
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);

        if (n < sizeof(CameraPacketHeader)) {
            ROS_WARN("Received data is too short to contain header.");
            return;
        }

        // 解析包头
        CameraPacketHeader header;
        std::memcpy(&header, buffer, sizeof(CameraPacketHeader));

        // 如果接收到新的包组，清空缓存并更新总包数
        if (header.seq_num == 0) {
            packet_buffer.clear();  // 清空缓存
            total_packets_expected = header.total_pkgs_num;  // 更新组的总包数
        }

        // 处理数据包内容
        std::vector<uint8_t> packet_data(buffer + sizeof(CameraPacketHeader), buffer + sizeof(CameraPacketHeader) + header.data_len);
        packet_buffer[header.seq_num] = packet_data;

        // 获取图像元数据，如果这是第一个数据包
        if (header.seq_num == 0) {
            std::memcpy(&img_meta_data, buffer + sizeof(CameraPacketHeader), sizeof(ImageMetaData));
        }

        // 检查是否收到所有数据包
        if (packet_buffer.size() == total_packets_expected) {
            // 重组数据包
            std::vector<uint8_t> complete_data;
            for (int i = 0; i < total_packets_expected; i++) {
                if (packet_buffer.find(i) != packet_buffer.end()) {
                    complete_data.insert(complete_data.end(), packet_buffer[i].begin(), packet_buffer[i].end());
                } else {
                    // 如果缺少某个包，等待该包的到来
                    std::cout << "Missing packet " << i << ", waiting for retransmission." << std::endl;
                    return;
                }
            }

            // 去掉最前面的头部部分（包含 PacketHeader 和 ImageMetaData）
            std::vector<uint8_t> image_data(complete_data.begin() + sizeof(ImageMetaData), complete_data.end());

            // 检查图像头部标识符
            if (img_meta_data.head == 0xFF55FF55) {
                // 发布图像数据
                publish_camera_data(image_data.data(), image_data.size(), img_meta_data.width, img_meta_data.height);
            }

            // 清空缓存
            packet_buffer.clear();
            total_packets_expected = 0;  // 重置
        }


    }


    void publish_camera_data(const uint8_t* data, int length, uint32_t width, uint32_t height) {
        std::cout << "length: " << length << " width: " << width << " height: " << height << " w*h: " << 3 * width * height << std::endl;

        // 将字节数据转换为 OpenCV 图像
        cv::Mat img(height, width, CV_8UC3, const_cast<uint8_t*>(data));

        // 将 RGB 转为 BGR，OpenCV 默认是 BGR 格式
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

        // 显示图像
        cv::imshow("Received Image", img);  // 显示图像窗口
        cv::waitKey(1);  // 等待一个小的时间，更新窗口（通常 1 毫秒就足够）

        // 转换为 ROS 图像消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "base_link";

        // 发布图像消息
        image_pub.publish(msg);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;

    CameraPublisher publisher(nh);
    publisher.run();

    return 0;
}
