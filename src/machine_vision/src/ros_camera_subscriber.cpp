#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

std::string topic_;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow(topic_, cv_bridge::toCvShare(msg,"rgb8")->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error in image subscriber: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  
  bool paramSet = ros::param::get("~topic", topic_);

  if(!paramSet){
      ROS_ERROR_STREAM("a topic must be specified!\n\t ex: rosrun machine_vision camera_subscriber _topic:=/cam1");
      return 1;
  }

  cv::namedWindow(topic_, CV_WINDOW_KEEPRATIO);
  cv::startWindowThread();
  
  image_transport::Subscriber sub = it.subscribe(topic_, 5, imageCallback);
  ROS_INFO_STREAM("subscribed to topic: " << topic_);
  
  ros::spin();
  cv::destroyWindow(topic_);
}
   
