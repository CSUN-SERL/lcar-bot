#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

static const std::string LEFT_WINDOW = "left cam";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_left = it_.subscribe("/usb_cam/image_raw", 1,
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(LEFT_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(LEFT_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(LEFT_WINDOW, cv_ptr->image);
    char quit = cv::waitKey(3);
    if ((int)quit == 27) return;

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
