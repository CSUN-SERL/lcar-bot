#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg)->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error in image subsrcriber: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  
  std::string topic;
  if(argc >= 2)
      topic = argv[1];
  else topic = "mono_cam/image_raw";
  
  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
