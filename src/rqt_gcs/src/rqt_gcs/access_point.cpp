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

void AccessPoint::set_image(Mat img)
{
	image = img;
}

void AccessPoint::set_GPS(float coord)
{
	GPS_location = coord;
}

void AccessPoint::set_compass(float heading)
{
	compass_heading = heading;
}

void AccessPoint::set_altitude(float altitude)
{
	this.altitude=altitude;
}

void AccessPoint::set_time(time_t t)
{
	current_time = t;
}

//getters

Mat get_img()
{
	return image;
}

float get_GPS()
{
	return GPS_location;
}

float get_compass()
{
	return compass_heading;
}

float get_altitude()
{
	return altitude;
}

time_t get_time()
{
	return current_time;
}

#endif //ACCESSPOINT_H
