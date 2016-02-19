#pragma once

#include "libuvc/libuvc.h"

#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using namespace camera_info_manager;

class CameraDriver{

    public:

        CameraDriver(
            int vendor_id,
            int product_id,
            ros::NodeHandle nh,
            CameraInfoManager &cinfo
        );
        ~CameraDriver();

        //bool open(int, int);
        void startStream();
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
        //ros::Rate loop_rate;
        CameraInfoManager *cinfo_;
        image_transport::ImageTransport it_;
        image_transport::Publisher pub_left_;
        image_transport::Publisher pub_right_;
        image_transport::Publisher pub_rgb_;

        unsigned int vendor_id_;
        unsigned int product_id_;

        // Accept a new image frame from the camera
        void ImageCallback(uvc_frame_t *frame);
        static void ImageCallbackAdapter(uvc_frame_t *frame, void *ptr);
};
