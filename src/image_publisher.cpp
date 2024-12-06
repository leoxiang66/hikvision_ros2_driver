#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <hikvision_api/utils.h>
#include <hikvision_api/timer.h>
#include <thread>

// Convert and publish the image as CompressedImage
void publishCompressedImage(MV_FRAME_OUT *stImageInfo, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &image_pub, FrameInfo *pframe_info)
{
    // Step 1: Convert Hikvision image to OpenCV Mat
    cv::Mat img;
    if (stImageInfo->stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed)
    {
        img = cv::Mat(stImageInfo->stFrameInfo.nHeight, stImageInfo->stFrameInfo.nWidth, CV_8UC3, stImageInfo->pBufAddr);
    }
    else if (stImageInfo->stFrameInfo.enPixelType == PixelType_Gvsp_Mono8)
    {
        img = cv::Mat(stImageInfo->stFrameInfo.nHeight, stImageInfo->stFrameInfo.nWidth, CV_8UC1, stImageInfo->pBufAddr);
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("hikvision_image_publisher"), "Unsupported pixel format!");
        return;
    }

    // Step 2: Encode the image using OpenCV compression
    std::vector<uchar> compressed_buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90}; // Set JPEG quality to 90
    cv::imencode(".jpg", img, compressed_buffer, compression_params);

    // Step 3: Create a CompressedImage message
    auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    msg->format = "jpeg"; // Set format to JPEG

    // Populate the header using FrameInfo attributes
    const uint64_t timestamp_nano = pframe_info->getTimestampNano();
    const unsigned int frame_id = pframe_info->getFrameID();
    msg->header.stamp = rclcpp::Time(static_cast<uint32_t>(timestamp_nano / 1000000000), static_cast<uint32_t>(timestamp_nano % 1000000000));
    msg->header.frame_id = std::to_string(frame_id);

    // Copy the compressed data into the message
    msg->data = std::move(compressed_buffer);

    // Step 4: Publish the CompressedImage message
    image_pub->publish(*msg);
}

// Thread function to continuously capture and publish compressed images
void pop_thread(void *handle, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &image_pub)
{
    while (rclcpp::ok())
    {
        auto frame = pop_image_buffer(handle, 1, false);
        if (frame != NULL)
        {
            FrameInfo *frame_info = get_frame_info(&(frame->stFrameInfo));
            print_frame_info(frame, true);
            publishCompressedImage(frame, image_pub, frame_info);
            delete frame;
            delete frame_info;
        }
    }
}

// Camera work function
void camera_work(unsigned int idx, double freq, uint64_t sync_point, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &image_pub)
{
    Timer timer(sync_point, freq);

    void *cam = init_SDK(idx);
    if (cam == NULL)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hikvision_image_publisher"), "Failed to initialize SDK.");
        return;
    }

    set_exposure_auto_off(cam);
    set_exposure_time(cam, 15000.0);
    get_exposure_time(cam);
    set_pixel_format(cam, PixelType_Gvsp_BGR8_Packed);
    turn_on_IEEE1588(cam);
    print_IEEE1588_status(cam);
    set_trigger_mode_on(cam);
    set_trigger_source_to_action(cam);
    set_action_keys(cam);
    start_grabbing(cam);

    std::thread capture_thread(pop_thread, cam, std::ref(image_pub));
    capture_thread.detach();

    timer.syncToFirstInterval();

    while (rclcpp::ok())
    {
        issue_action_command();
        timer.syncToNextInterval();
    }

    stop_grabbing(cam);
    close_device(cam);
}

class ImagePublisherNode : public rclcpp::Node
{
public:
    ImagePublisherNode() : Node("hikvision_image_publisher")
    {
        // Declare and get parameters
        declare_parameter<std::string>("image_topic", "/camera/image_raw");
        get_parameter("image_topic", image_topic_);

        RCLCPP_INFO(this->get_logger(), "Image Topic: %s", image_topic_.c_str());

        // Create a publisher for CompressedImage messages
        image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(image_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Starting camera work...");
        camera_work(0, 20.0, 10000000, image_pub_);
    }

private:
    std::string image_topic_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisherNode>());
    rclcpp::shutdown();
    return 0;
}