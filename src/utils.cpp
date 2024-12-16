#include "utils.h"

void camera_work(unsigned int idx, double freq, uint64_t sync_point, float ex_time, image_transport::Publisher &image_pub)
{
    Timer timer(sync_point, freq);

    void *cam = init_SDK(idx);
    if (cam == NULL)
    {
        RCLCPP_ERROR(rclcpp::get_logger("hikvision_image_publisher"), "Failed to initialize SDK.");
        return;
    }

    set_exposure_auto_off(cam);
    set_exposure_time(cam, ex_time);
    get_exposure_time(cam);
    set_pixel_format(cam, PixelType_Gvsp_BGR8_Packed);
    turn_on_IEEE1588(cam);
    wait_until_slave(cam);

    // todo: launch lidar
    pid_t pid = fork();

    if (pid < 0)
    {
        std::cerr << "Fork failed!" << std::endl;
        return;
    }
    else if (pid == 0)
    { // 子进程:
        execl("/bin/bash", "bash", "lidar.launch", (char *)nullptr);
        std::cerr << "Execution failed!" << std::endl;
        _exit(1);
    }
    else
    {
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
}

void publishImage(MV_FRAME_OUT *stImageInfo, image_transport::Publisher &image_pub)
{
    printf("Time before step 1: ");
    print_current_time();
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

    printf("Time before step 2: ");
    print_current_time();
    // Step 2: Convert OpenCV Mat to sensor_msgs/Image using cv_bridge
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();

    const uint64_t timestamp_nano = combine_high_low(stImageInfo->stFrameInfo.nDevTimeStampHigh, stImageInfo->stFrameInfo.nDevTimeStampLow);
    const unsigned int frame_id = stImageInfo->stFrameInfo.nFrameNum;

    printf("Time before step 3: ");
    print_current_time();
    // Step 3: Populate the header with FrameInfo attributes
    msg->header.stamp = rclcpp::Time(static_cast<uint32_t>(timestamp_nano / 1000000000), static_cast<uint32_t>(timestamp_nano % 1000000000));
    msg->header.frame_id = std::to_string(frame_id);

    printf("Time before step 4: ");
    print_current_time();
    // Step 4: Publish the image message
    image_pub.publish(msg);

    printf("Time after step 4: ");
    print_current_time();
}

void pop_thread(void *handle, image_transport::Publisher &image_pub)
{
    while (rclcpp::ok())
    {
        auto frame = pop_image_buffer(handle, 1, false);
        if (frame != NULL)
        {
            // FrameInfo *frame_info = get_frame_info(&(frame->stFrameInfo));
            // print_frame_info(frame, true);
            publishImage(frame, image_pub);
            delete frame;
            // delete frame_info;
        }
    }
}
