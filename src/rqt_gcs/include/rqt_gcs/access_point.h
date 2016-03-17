#ifndef ACCESS_POINT
#define ACCESS_POINT

#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <opencv2/highgui.hpp>

class AccessPoint
{
  private:
    cv::Mat image;                          //Access Point image
    sensor_msgs::NavSatFix capture_location;	//GPS location at time of image capture
		std_msgs::Float64 compass_heading;
		std_msgs::Float64 altitude;
    ros::Time capture_time;

	public:
		AccessPoint(){}
		~AccessPoint(){}

    //Setter Functions
    void SetImage(cv::Mat& img){ image = img; }
    void SetLocation(sensor_msgs::NavSatFix& coord){ capture_location = coord; }
    void SetHeading(std_msgs::Float64 heading){ compass_heading = heading; }
    void SetAltitude(std_msgs::Float64 altitude){ this->altitude = altitude; }
    void SetTime(ros::Time t){ capture_time = t; }

    //getters
    cv::Mat GetImage(){ return image; }
    sensor_msgs::NavSatFix GetLocation(){ return capture_location; }
    std_msgs::Float64 GetHeading(){ return compass_heading; }
    std_msgs::Float64 GetAltitude(){ return altitude; }
    ros::Time GetTime(){ return capture_time; }
};

#endif //ACCESS_POINT
