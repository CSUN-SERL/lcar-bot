
/* 
 * File:   point.h
 * Author: n8
 *
 * Created on April 4, 2017, 12:14 AM
 */

#ifndef POINT_H
#define POINT_H

namespace gcs
{

struct Point 
{
public:
    Point() :
    x(0),
    y(0),
    z(0)
    { }

    Point(double x, double y, double z) :
    x(x),
    y(y),
    z(z)
    { }

    Point(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    Point& operator=(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    float distanceTo(const Point& other)
    {
        float dist_x = x - other.x;
        float dist_y = y - other.y;
        float dist_z = z - other.z;
        return std::sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
    }

    double x;
    double y;
    double z;
};

struct RPY
{
    RPY() :
    roll(0),
    pitch(0),
    yaw(0)
    {}
    
    RPY(float r, float p, float y) :
    roll(r),
    pitch(p),
    yaw(y)
    { }
    
    RPY(const RPY& other) :
    roll(other.roll),
    pitch(other.pitch),
    yaw(other.yaw)
    { }
    
    RPY& operator=(const RPY& other)
    {
        roll = other.roll;
        pitch = other.pitch;
        yaw = other.yaw;
        return *this;
    }
    
    float roll;
    float pitch;
    float yaw;
};

struct Position
{
    Position(){ }
    
    Position(const Point& pos, const RPY& rpy) :
    position(pos),
    orientation(rpy)
    { }
    
    Position(const Position& other) :
    position(other.position),
    orientation(other.orientation)
    { }
    
    Position& operator=(const Position& other)
    {
        position = other.position;
        orientation = other.orientation;
        return *this;
    }

    float distanceTo(const Position& other)
    {
        return position.distanceTo(other.position);
    }

    Point position;
    RPY orientation;
};

}

#endif /* POINT_H */

