
#include <cstdlib>
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>

double min_distance_;
double fill_;

ros::Publisher pub_;

namespace enc = sensor_msgs::image_encodings;

void disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity_msg)
{   
    ros::Rate loop_rate (10); // hz
    loop_rate.sleep();         
    
    assert(disparity_msg->image.encoding == enc::TYPE_32FC1);
    const cv::Mat_<float> dmat(disparity_msg->image.height, disparity_msg->image.width, // convert to cv::mat for analysis
                               (float*)&disparity_msg->image.data[0], disparity_msg->image.step);
    
    float min_disparity = disparity_msg->min_disparity;
    float max_disparity = disparity_msg->max_disparity;
    float z_depth_total = 0;
    float baseline = disparity_msg->T;
    float focal_length = disparity_msg->f;
    int img_size = dmat.rows * dmat.cols;
    double threshold = fill_ * img_size; 
    
    std_msgs::Float64 prox;
    prox.data = 0;
    int prox_count = 0;
    
    for (int row = 0; row < dmat.rows; ++row) {
      const float* d = dmat[row];
      for (int col = 0; col < dmat.cols; ++col) {
          float z_depth = focal_length * baseline / d[col];
          if(z_depth > 0 && z_depth < min_distance_){
              prox_count++;
              z_depth_total += z_depth;
          }  
      }
    }
    //if(prox_count > threshold){
        prox.data = z_depth_total / prox_count;        
        //ROS_ERROR_STREAM("ERROR land! " << prox_count << "  |  " << prox.data);
        pub_.publish(prox);
    //}
    
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "object_avoidance", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  
  std::string topic;
  bool paramSet = ros::param::get("~topic", topic);
  if(!paramSet)
      topic = "stereo_cam/disparity";
  
  paramSet = ros::param::get("~fill", fill_);
  if(!paramSet)
      fill_ = 0.1; // proximal object must fill 10% of image
  
  paramSet = ros::param::get("~min_distance", min_distance_);
  if(!paramSet)
      min_distance_= 2.0; // proximal object must be closer than 2.0 fee(?))
  
  ROS_INFO_STREAM ("topic: " << topic 
                   << "\n\t\t_fill: " << fill_
                   << "\n\t\t_min_distance: " << min_distance_
                  );
  
  ros::Subscriber sub = nh.subscribe(topic, 5, disparityCallback);
  pub_ = nh.advertise<std_msgs::Float64>("object_avoidance/depth", 1);
  
  ros::spin(); 
}


