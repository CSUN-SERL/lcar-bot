#include "libuvc/libuvc.h"

#include <opencv/cv.h>
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream> // for converting the command line parameter to integer#
#include <stdlib.h>

using namespace std;
using namespace cv;
using namespace cv::ml;

image_transport::Publisher pubLeft;
image_transport::Publisher pubRight;
image_transport::Publisher pubRGB;


//Draw detected objects on a window
void DrawLocations(cv::Mat & img, const std::vector< Rect > & found, const Scalar & color, string label) {
    if (!found.empty()) {
       std::vector< Rect >::const_iterator loc = found.begin();
       std::vector< Rect >::const_iterator end = found.end();
       for (; loc != end; ++loc) {
           rectangle(img, *loc, color, 2);
                    //detected_objects.push(img.clone());
            }
    }
}

////////////////////////////////////////////////////////////////////////////////
//extract svm weights and store in a float vector
void GetSvmDetector(const Ptr<SVM>& svm, vector< float > & hog_detector) {
    // get the support vectors
    Mat sv = svm->getSupportVectors();
    const int sv_total = sv.rows;
    // get the decision function
    cv::Mat alpha;
    cv::Mat svidx;
    double rho = svm->getDecisionFunction(0, alpha, svidx);
    cout << alpha.total();
    cout << "\n";
    svidx.reshape(1, 1);
    cout << svidx.total();
    cout << "\n";
    cout << sv_total;
    cout << "\n";
    cout << rho << "\n";
    //    CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
    //    CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
    //            (alpha.type() == CV_32F && alpha.at<float>(0) == 1.f));
    //    CV_Assert(sv.type() == CV_32F);
    hog_detector.clear();

    hog_detector.resize(sv.cols + 1);
    memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof (hog_detector[0]));
    hog_detector[sv.cols] = (float) -rho;
}
////////////////////////////////////////////////////////////////////////////////
//Perform real time HOG object detection
Mat HogObjectDetection(Mat img) {
    cout << "break here\n";
    const string modelLibPath = ros::package::getPath("machine_vision/LinearHOG.xml");
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    cout << "load svm\n";
    VideoCapture cap;
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    Scalar window(255, 0, 0);
    vector< Rect > objects;
    GetSvmDetector(svm , hog_detector);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1000);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1000);
    HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog.winSize = Size(128, 256);
    //HOGDescriptor people;
    //hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    hog.setSVMDetector(hog_detector);

    //int value = 0;
    // namedWindow("video capture", CV_WINDOW_AUTOSIZE);
    // cap.open(0);
    // while (true) {
    //     cap >> img;
    //     if (!img.data)
    //         break;
        objects.clear();
        hog.detectMultiScale(img, objects, true);
        //people.detectMultiScale(img, locations);
        DrawLocations(img, objects, reference, "door");
    //     imshow("video capture", img);
    //     if (waitKey(10) >= 0){ //any key pressed
    //         std::terminate(); //terminates all running threads
    //     }
    // }
    return img;
}
////////////////////////////////////////////////////////////////////////////////
void cb(uvc_frame_t *frame, void *ptr) {
  ros::Rate loop_rate(30);
  loop_rate.sleep();
  frame->frame_format = UVC_FRAME_FORMAT_YUYV;
  std::cout << "Frame format" << std::endl;
  std::cout << frame->frame_format << std::endl;
  uvc_frame_t *greyLeft;
  uvc_frame_t *greyRight;
  uvc_frame_t *frameRGB;
  uvc_error_t retLeft;
  uvc_error_t retRight;
  uvc_error_t retRGB;

  /* We'll convert the image from YUV/JPEG to gray8, so allocate space */
  greyLeft = uvc_allocate_frame(frame->width * frame->height);
  greyRight = uvc_allocate_frame(frame->width * frame->height);
  frameRGB = uvc_allocate_frame(frame->width * frame->height);

  if (!greyLeft) {
    printf("unable to allocate grey left frame!");
    return;
  }
  if (!greyRight) {
    printf("unable to allocate grey right frame!");
    return;
  }
  if(!frameRGB){
      printf("unable to allocate RGB frame!");
      return;
  }

  /* Do the BGR conversion */
  retLeft = uvc_yuyv2y(frame, greyLeft);
  retRight = uvc_yuyv2uv(frame, greyRight);
  retRGB = uvc_yuyv2rgb(frame, frameRGB);

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
  if (retRGB) {
    uvc_perror(retRGB, "uvc_yuyv2rgb");
    uvc_free_frame(frameRGB);
    printf("Error with yuyv to rgb conversion");
    return;
  }

  cv::Mat left (greyLeft->height, greyLeft->width, CV_8UC1, greyLeft->data);
  cv::Mat right (greyRight->height, greyRight->width, CV_8UC1, greyRight->data);
  cv::Mat RGB (frameRGB->height, frameRGB->width, CV_8UC3, frameRGB->data);
  HogObjectDetection(left);
  HogObjectDetection(right);
  HogObjectDetection(RGB);
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

  ///*
  if(!RGB.empty()){
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
                                                "bgr8", RGB).toImageMsg();
      pubRGB.publish(msg);
  }
  //*/

  uvc_free_frame(greyLeft);
  uvc_free_frame(greyRight);
  uvc_free_frame(frameRGB);

}
////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv){
    //ros inits
    ros::init(argc, argv, "object_detect");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pubLeft = it.advertise( "stereo_cam/left/image_raw", 1);
    pubRight = it.advertise("stereo_cam/right/image_raw", 1);
    pubRGB = it.advertise(  "stereo_cam/rgb/image_raw", 1);

    //ros::Rate loop_rate(30); //10Hz


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
            while(nh.ok()){
                ros::spinOnce();
                //loop_rate.sleep();
              }
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
