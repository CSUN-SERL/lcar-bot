#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include <sstream> // for converting the command line parameter to integer#
#include <stdlib.h>

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "mono_cam", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // get node specfic parameters from parameter server

  std::string topic;
  bool paramSet = ros::param::get("~image_topic", topic);

  if(!paramSet){
      topic = "mono_cam/image_raw";
  }
  image_transport::Publisher pub = it.advertise(topic, 1);

  int video_source;
  paramSet = ros::param::get("~video_id", video_source);
  if(!paramSet){
      video_source = -1;
  }
  cout << video_source << "\n";
  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()){
      ROS_ERROR_STREAM("Error opening camera");
      return 1;
  }
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  int numFrames = 0;
  while (nh.ok()) {
    try{
        cout << "attempting frame capture\n";
        cap >> frame;
    // Check if grabbed frame is actually full with some content
        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);

            cout << numFrames++ << endl;
        }
        else cout << "frame skipped\n";
    }
    catch(cv::Exception e){
        e.formatMessage();
        ROS_ERROR("%s", e.what());
    }
    ros::spinOnce();
    //loop_rate.sleep();
  }
}
