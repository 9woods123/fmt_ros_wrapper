#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class TFPublisher {
public:
    TFPublisher() {
        // 订阅位置话题
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10, &TFPublisher::poseCallback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
        // 创建变换
        geometry_msgs::TransformStamped transform;

        transform.header.stamp = pose_msg->header.stamp;
        transform.header.stamp = ros::Time::now();

        transform.header.frame_id = "/map";  // 参考框架
        transform.child_frame_id = "/drone";  // 子框架

        transform.transform.translation.x = pose_msg->pose.position.x;
        transform.transform.translation.y = pose_msg->pose.position.y;
        transform.transform.translation.z = pose_msg->pose.position.z;

        // 使用 tf2::Quaternion 创建四元数
        tf2::Quaternion q(pose_msg->pose.orientation.x,
                          pose_msg->pose.orientation.y,
                          pose_msg->pose.orientation.z,
                          pose_msg->pose.orientation.w);
        
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // 发布变换
        tf_broadcaster_.sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_from_mavros");
    TFPublisher tf_publisher;
    ros::spin();
    return 0;
}
