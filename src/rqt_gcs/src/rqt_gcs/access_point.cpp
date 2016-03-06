#ifdef ACCESS_POINT_H
#define ACCESS_POINT_H

#include <rqt_gcs/simple_control.h>
#include <rqt_gcs/simple_control.cpp>

class AccessPoint
{
	public:
		Mat image; 						//Access Point image
		float GPS_location;	//GPS location at time of image capture
		float compass_heading;
		float altitude;
		time_t current_time;

	public:
		AccessPoint();
		~AccessPoint();

		void set_image(Mat img);
		void set_GPS(float coord);
		void set_compass(float heading);
		void set_altitude(float altitude);
		void set_time(time_t t);

		Mat get_img();
		float get_GPS();
		float get_compass();
		float get_altitude();
		time_t get_time();
};

AccessPoint::AccessPoint()
{
	image = null;
	GPS_location = 0;
	compass_heading = 0;
	altitude = 0;
	current_time = std::time(nullptr);
}
//destructor
AccessPoint::~AccessPoint()
{

}

//setters

void AccessPoint::SetImage(Mat img)
{
	image = img;
}

void AccessPoint::SetImage(float coord)
{
	GPS_location = coord;
}

void AccessPoint::SetCompass(float heading)
{
	compass_heading = heading;
}

void AccessPoint::SetAltitude(float altitude)
{
	this.altitude=altitude;
}

void AccessPoint::SetTime(time_t t)
{
	current_time = t;
}

//getters

Mat GetImage()
{
	return image;
}

float GetGPS()
{
	return GPS_location;
}

float GetCompass()
{
	return compass_heading;
}

float GetAltitude()
{
	return altitude;
}

time_t GetTime()
{
	return current_time;
}

#endif //ACCESSPOINT_H
