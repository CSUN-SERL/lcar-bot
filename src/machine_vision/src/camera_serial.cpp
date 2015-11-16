#include "opencv2/opencv.hpp"
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

const string ROS_NODE_NAME = "cam_serial";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;


public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/camera/image_raw", 1,
    //  &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/stereo_camera/left", 1);

    cv::namedWindow("OPENCV_WINDOW");
  }

  ~ImageConverter()
  {
    cv::destroyWindow("OPENCV_WINDOW");
  }


};
int main(int argc, char** argv)
{class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }


};

    ros::init(argc, argv, ROS_NODE_NAME);

    Mat frameLeft;
    Mat frameRight;

    VideoCapture lCap(0);
    VideoCapture rCap(1);

    if(!lCap.isOpened() || !rCap.isOpened()) return -1;

    //namedWindow("cam_left",  CV_WINDOW_AUTOSIZE);
    //namedWindow("cam_right", CV_WINDOW_AUTOSIZE);

    //lCap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    //lCap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    //rCap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    //rCap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);


    for(;;){
      //Mat frameLeft;
      //Mat frameRight;
        lCap >> frameLeft;
        rCap >> frameRight;

        if(!frameLeft.empty())
          imshow("cam_left", frameLeft);

        if(!frameRight.empty())
          imshow("cam_right", frameRight);

        if(waitKey(30) >= 0) break;

    }

    return 0;
}
