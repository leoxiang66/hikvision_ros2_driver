#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
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

        // read parameters
        declare_parameter<float>("exposure_time", 10000.0);
        declare_parameter<int>("frequency", 10);
        declare_parameter<int>("sync_point", 10000000);

        float exposure_time = get_parameter("exposure_time").get_value<float>();
        float frequency = get_parameter("frequency").get_value<int>();
        int sync_point = get_parameter("sync_point").get_value<int>();

        std::cout << "Exposure Time: " << exposure_time << std::endl;
        std::cout << "Frequency: " << frequency << std::endl;
        std::cout << "Sync point: " << sync_point << std::endl;
        // RCLCPP_INFO(this->get_logger(), "Image Topic: %s", image_topic_.c_str());

        // Advertise the image topic
        image_pub_ = it.advertise(image_topic_, 1);

        RCLCPP_INFO(this->get_logger(), "Starting camera work...");
        camera_work(
            0,                      // the index of the camera to control 
            frequency,              // the frequency to publish images
            sync_point,             // the sync point
            exposure_time,          // the exposure time of the camera
            image_pub_              // the reference to the image publisher
            );            
    }

private:
    std::string image_topic_ = "hikvision/camera/image_raw";
    image_transport::Publisher image_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
    rclcpp::shutdown();
    return 0;
}