#ifdef ACCESS_POINT_H
#define ACCESS_POINT_H

#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64>
#include <opencv2/cv.h>

class AccessPoint
{
	public:
		cv::Mat image; 						//Access Point image
		sensor_msgs::NavSatFix gps_location;	//GPS location at time of image capture
		std_msgs::Float64 compass_heading;
		std_msgs::Float64 altitude;
		ros::Time current_time;

	public:
		AccessPoint();
		~AccessPoint();

		//setters
		void SetImage(cv::Mat& img);
		void SetGPS(sensor_msgs::NavSatFix& coord);
		void SetCompass(std_msgs::Float64& heading);
		void SetAltitude(std_msgs::Float64& altitude);
		void SetTime(ros::Time& t);

		//getters
		cv::Mat GetImage();
		sensor_msgs::NavSatFix GetGPS();
		std_msgs::Float64 GetCompass();
		std_msgs::Float64 GetAltitude();
		ros::Time GetTime();
};

// AccessPoint::AccessPoint()
// {
// 	image = null;
// 	GPS_location = 0;
// 	compass_heading = 0;
// 	altitude = 0;
// 	current_time = std::time(nullptr);
// }
//destructor
AccessPoint::~AccessPoint()
{

}

//setters

void AccessPoint::SetImage(cv::Mat& img){
	image = img;
}

void AccessPoint::SetImage(sensor_msgs::NavSatFix& coord)
{
	gps_location = coord;
}

void AccessPoint::SetCompass(std_msgs::Float64& heading)
{
	compass_heading = heading;
}

void AccessPoint::SetAltitude(std_msgs::Float64& altitude)
{
	this.altitude = altitude;
}

void AccessPoint::SetTime(ros::Time t)
{
	current_time = t;
}

//getters

cv::Mat GetImage()
{
	return image;
}

float GetGPS()
{
	return gps_location;
}

std_msgs::Float64 GetCompass()
{
	return compass_heading;
}

std_msgs::Float64& GetAltitude()
{
	return altitude;
}

ros::Time GetTime()
{
	return current_time;
}

ros::Time GetCurrentTime()
{
	return ros::Time::now();
}

#endif //ACCESSPOINT_H
