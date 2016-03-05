#pragma once

#include "libuvc/libuvc.h"

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <cv_bridge/cv_bridge.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using namespace camera_info_manager;

class StereoDriver{

    public:

        StereoDriver(
            int vendor_id,
            int product_id,
            ros::NodeHandle nh
        );
        ~StereoDriver();

        //bool open(int, int);
        void StartStream();
        bool IsOpen();
        //void stop();

    private:
        uvc_context_t *ctx;
        uvc_device_t *dev;
        uvc_device_handle_t *devh;
        uvc_stream_ctrl_t ctrl;
        uvc_error_t error_t_state;
        uvc_device_t **list;
        uvc_error_t res;

        ros::NodeHandle nh_;
        CameraInfoManager cinfo_left_;
        CameraInfoManager cinfo_right_;
        image_transport::ImageTransport it_;
        image_transport::CameraPublisher pub_left_;
        image_transport::CameraPublisher pub_right_;
        image_transport::Publisher pub_rgb_;

        int stereo_image_id_;
        bool open_;
        unsigned int vendor_id_;
        unsigned int product_id_;

        // Accept a new image frame from the camera
        void ImageCallback(uvc_frame_t *frame);
        static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
};
