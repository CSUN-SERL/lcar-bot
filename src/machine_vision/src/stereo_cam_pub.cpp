#include "libuvc/libuvc.h"

#include <opencv/cv.h>
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream> // for converting the command line parameter to integer#
#include <stdlib.h>

image_transport::Publisher pubLeft;
image_transport::Publisher pubRight;
image_transport::Publisher pubBGR;

void cb(uvc_frame_t *frame, void *ptr) {
  frame->frame_format = UVC_FRAME_FORMAT_YUYV;
  std::cout << "Frame format" << std::endl;
  std::cout << frame->frame_format << std::endl;
  uvc_frame_t *greyLeft;
  uvc_frame_t *greyRight;
  uvc_error_t retLeft;
  uvc_error_t retRight;

  /* We'll convert the image from YUV/JPEG to gray8, so allocate space */
  greyLeft = uvc_allocate_frame(frame->width * frame->height);
  greyRight = uvc_allocate_frame(frame->width * frame->height);
  if (!greyLeft) {
    printf("unable to allocate grey left frame!");
    return;
  }
  if (!greyRight) {
    printf("unable to allocate grey right frame!");
    return;
  }

  /* Do the BGR conversion */
  retLeft = uvc_yuyv2y(frame, greyLeft);
  retRight = uvc_yuyv2uv(frame, greyRight);

  if (retLeft) {
    uvc_perror(retLeft, "uvc_yuyv2y");
    uvc_free_frame(greyLeft);
    printf("Error with yuyv to y conversion");
    return;
  }

  if (retRight) {
    uvc_perror(retRight, "uvc_yuyv2uv");
    uvc_free_frame(greyRight);
    printf("Error with yuyv to uv conversion");
    return;
  }

  cv::Mat left (greyLeft->height, greyLeft->width, CV_8UC1, greyLeft->data);
  cv::Mat right (greyRight->height, greyRight->width, CV_8UC1, greyRight->data);
  cv::Mat bgrFrame (frame->height, frame->width, CV_8UC3, frame->data);

  if(!left.empty()){
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "mono8", left).toImageMsg();
      pubLeft.publish(msg);
  }

  if(!right.empty()){
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "mono8", right).toImageMsg();
      pubRight.publish(msg);
  }

  /*
  if(!bgrFrame.empty()){
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "bgr8", bgrFrame).toImageMsg();
      pubBGR.publish(msg);
  }
  */

  uvc_free_frame(greyLeft);
  uvc_free_frame(greyRight);
  //uvc_free_frame(frame);
}

int main (int argc, char** argv){
    //ros inits
    ros::init(argc, argv, "stereo_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pubLeft = it.advertise( "stereo_cam/left/image_raw", 1);
    pubRight = it.advertise("stereo_cam/right/image_raw", 1);
    pubBGR = it.advertise(  "stereo_cam/bgr/image_raw", 1);

    //libuvc inits
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;
    uvc_device_t **list;

    res = uvc_init(&ctx, NULL);

    if (res < 0) {
      uvc_perror(res, "uvc_init");
      return res;
    }
    puts("UVC initialized");
    res = uvc_get_device_list(ctx, &list);
    if (res < 0) {
      uvc_perror(res, "uvc_get_device_list");
      return res;
    }

    /* Locates the first attached UVC device, stores in dev */
    res = uvc_find_device(
        ctx, &dev,
        0, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

    if (res < 0) {
      uvc_perror(res, "uvc_find_device"); /* no devices found */
    } else {
      puts("Device found");

      /* Try to open the device: requires exclusive access */
      res = uvc_open(dev, &devh);

      if (res < 0) {
        uvc_perror(res, "uvc_open"); /* unable to open device */
        printf("Unable to open device");
      } else {
        puts("Device opened");

        /* Print out a messagls e containing all the information that libuvc
         * knows about the device */
        uvc_print_diag(devh, stderr);

        /* Try to negotiate a 640x480 30 fps YUYV stream profile */
        res = uvc_get_stream_ctrl_format_size(
            devh, &ctrl, /* result stored in ctrl */
            UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
            640, 480, 30 /* width, height, fps */
        );

        /* Print out the result */
        uvc_print_stream_ctrl(&ctrl, stderr);

        if (res < 0) {
          uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
        } else {
          /* Start the video stream. The library will call user function cb:
           *   cb(frame, (void*) 12345)
           */
          res = uvc_start_streaming(devh, &ctrl, cb, (void*)12345, 0);

          if (res < 0) {
            uvc_perror(res, "start_streaming"); /* unable to start stream */
          } else {
            puts("Streaming...");

            uvc_set_ae_mode(devh, 0); /* e.g., turn on auto exposure */

            //sleep(10); /* stream for 10 seconds */
            while(nh.ok())
                ros::spinOnce();

            /* End the stream. Blocks until last callback is serviced */
            uvc_stop_streaming(devh);
            puts("Done streaming.");
          }
        }

        /* Release our handle on the device */
        uvc_close(devh);
        puts("Device closed");
      }

      /* Release the device descriptor */
      uvc_unref_device(dev);
    }

    /* Close the UVC context. This closes and cleans up any existing device handles,
     * and it closes the libusb context if one was not provided. */
    uvc_exit(ctx);
    puts("UVC exited");

    return 0;
}
