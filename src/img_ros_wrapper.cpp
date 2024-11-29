#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>

#define UDP_PORT 14556

class ImagePublisher {
public:
    ImagePublisher(ros::NodeHandle& nh)
        : image_pub(nh.advertise<sensor_msgs::Image>("image", 10)) {
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
        char buffer[1024 * 100]; // 接收缓冲区
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);
        if (n < 0) {
            ROS_ERROR("ERROR in recvfrom");
            return;
        }

        process_image(buffer, n);
    }

    void process_image(const char* data, int length) {
        if (length < sizeof(uint32_t) * 3) {
            ROS_WARN("Received data is too short for image header.");
            return;
        }

        // 解析头部数据
        const uint32_t* header = reinterpret_cast<const uint32_t*>(data);
        uint32_t magic = header[0];
        uint32_t width = header[1];
        uint32_t height = header[2];

        // 检查包头是否正确
        if (magic != 0xFF55FF55) {
            ROS_WARN("Invalid magic number in image header.");
            return;
        }

        size_t image_data_offset = sizeof(uint32_t) * 3;
        size_t expected_image_size = width * height * 3; // RGB 图像

        if (length < image_data_offset + expected_image_size) {
            ROS_WARN("Received data is too short for the image payload.");
            return;
        }

        // 创建图像消息
        sensor_msgs::Image img_msg;
        img_msg.header.frame_id = "camera_link";
        img_msg.header.stamp = ros::Time::now();
        img_msg.height = height;
        img_msg.width = width;
        img_msg.encoding = "rgb8";
        img_msg.is_bigendian = false;
        img_msg.step = width * 3; // 每行字节数
        img_msg.data.resize(expected_image_size);

        // 复制图像数据
        const uint8_t* image_data = reinterpret_cast<const uint8_t*>(data + image_data_offset);
        std::copy(image_data, image_data + expected_image_size, img_msg.data.begin());

        // 发布消息
        image_pub.publish(img_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    ImagePublisher publisher(nh);
    publisher.run();

    return 0;
}
