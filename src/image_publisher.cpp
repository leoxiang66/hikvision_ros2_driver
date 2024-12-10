#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp> // Include PointCloud2 message type
#include "utils.h"

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode() : Node("hikvision_image_publisher")
    {
        // Create a shared_ptr to this node
        auto node_shared_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

        // Initialize image_transport with the shared_ptr
        image_transport::ImageTransport it(node_shared_ptr);

        // Read parameters
        declare_parameter<int>("exposure_time", 10000.0);
        declare_parameter<int>("frequency", 10);
        declare_parameter<int>("sync_point", 10000000);

        int exposure_time = get_parameter("exposure_time").get_value<int>();
        int frequency = get_parameter("frequency").get_value<int>();
        int sync_point = get_parameter("sync_point").get_value<int>();

        std::cout << "Exposure Time: " << exposure_time << std::endl;
        std::cout << "Frequency: " << frequency << std::endl;
        std::cout << "Sync point: " << sync_point << std::endl;

        // Advertise the image topic
        image_pub_ = it.advertise(image_topic_, 1);

        RCLCPP_INFO(this->get_logger(), "Starting camera work...");
        camera_setup(
            0,             // the index of the camera to control
            exposure_time, // the exposure time of the camera
            image_pub_     // the reference to the image publisher
        );
    }

private:
    std::string image_topic_ = "hikvision/camera/image_raw";
    image_transport::Publisher image_pub_;
};

// New node to subscribe to /livox/lidar
class LidarSubscriberNode : public rclcpp::Node
{
public:
    LidarSubscriberNode() : Node("lidar_subscriber_node")
    {
        // Create a subscriber to /livox/lidar
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&LidarSubscriberNode::lidarCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Lidar subscriber node initialized.");


        // Read parameters
        declare_parameter<int>("exposure_time", 10000);
        declare_parameter<int>("frequency", 10);
        declare_parameter<int>("sync_point", 10000000);

        exposure_time = get_parameter("exposure_time").get_value<int>();
        frequency = get_parameter("frequency").get_value<int>();
        sync_point = get_parameter("sync_point").get_value<int>();
        timer = new Timer(sync_point,frequency);
    }

private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Print the header timestamp
        // RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message with timestamp: %u.%u",
        //             msg->header.stamp.sec, msg->header.stamp.nanosec);

        // timer->syncToNextIntervalMinusExposureTime(exposure_time);
        std::this_thread::sleep_for(std::chrono::microseconds(50000) - std::chrono::microseconds(exposure_time) - std::chrono::microseconds(20000));
        issue_action_command();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    int exposure_time;
    int frequency;
    int sync_point;
    Timer *timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create and spin both nodes
    auto image_publisher_node = std::make_shared<ImagePublisherNode>();
    auto lidar_subscriber_node = std::make_shared<LidarSubscriberNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(image_publisher_node);
    executor.add_node(lidar_subscriber_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}