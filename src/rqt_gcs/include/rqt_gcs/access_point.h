#ifndef ACCESS_POINT
#define ACCESS_POINT

#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

class AccessPoint
{
public:
    //Enumerators
    enum ObjectType{
        door,
        window,
        hole
    };

    AccessPoint(){}
    ~AccessPoint(){}

    //Setter Functions
    void SetTime(ros::Time t){ capture_time = t; }
    void SetImage(sensor_msgs::Image img){ image = img; }
    void SetLocation(sensor_msgs::NavSatFix coord){ capture_location = coord; }
    void SetHeading(std_msgs::Float64 heading){ compass_heading = heading; }
    void SetAltitude(std_msgs::Float64 altitude){ this->altitude = altitude; }
    void SetType(ObjectType type){ this->type = type; }

    //Getter Functions
    sensor_msgs::Image  GetImage(){ return image; }
    sensor_msgs::NavSatFix GetLocation(){ return capture_location; }
    std_msgs::Float64 GetHeading(){ return compass_heading; }
    std_msgs::Float64 GetAltitude(){ return altitude; }
    ObjectType GetType(){ return type; }
    ros::Time GetTime(){ return capture_time; }

private:
    ros::Time               capture_time;        //Time of image capture
    sensor_msgs::Image      image;               //Access Point image
    sensor_msgs::NavSatFix  capture_location;    //GPS location at time of image capture
    std_msgs::Float64       compass_heading;
    std_msgs::Float64       altitude;
    ObjectType              type;
};

#endif //ACCESS_POINT
