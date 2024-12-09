#ifndef HIK_UTILS_H
#define HIK_UTILS_H

#include <hikvision_api/timer.h>
#include <hikvision_api/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <chrono>


/**
 * @brief Performs camera operations based on the provided parameters.
 *
 * This function is responsible for managing the camera's work, including
 * capturing images at a specified frequency and synchronizing with a given
 * sync point. The function may also handle exposure time settings.
 *
 * @param idx The index of the camera to operate on.
 * @param freq The frequency at which to capture images, in Hertz.
 * @param sync_point A timestamp indicating the synchronization point for image capture.
 * @param ex_time The exposure time for the camera, in seconds.
 * @param image_pub A reference to the image transport publisher used to publish captured images.
 */
void camera_work(unsigned int idx, double freq, uint64_t sync_point, float ex_time, image_transport::Publisher &image_pub);

/**
 * @brief Publishes an image frame to the specified image transport publisher.
 *
 * This function takes the output frame information from the camera and publishes
 * it using the provided image transport publisher. It may also handle additional
 * frame information for processing or logging purposes.
 *
 * @param stImageInfo Pointer to the structure containing image frame output information.
 * @param image_pub A reference to the image transport publisher used to publish the image.
 * @param pframe_info Pointer to additional frame information that may be needed for processing.
 */
void publishImage(MV_FRAME_OUT *stImageInfo, image_transport::Publisher &image_pub, FrameInfo *pframe_info);

/**
 * @brief Thread function for processing image data and publishing it.
 *
 * This function runs in a separate thread and is responsible for handling
 * the image data from the camera. It pops the image data upon available and publishes it
 * by calling the `publishImage` function.
 *
 * @param handle A pointer to the handle used for managing the thread's operations.
 * @param image_pub A reference to the image transport publisher used to publish images.
 */
void pop_thread(void *handle, image_transport::Publisher &image_pub);

#endif // !HIK_UTILS_H


