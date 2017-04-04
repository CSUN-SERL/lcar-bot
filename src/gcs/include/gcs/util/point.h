
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
    Point()
    {}

    Point(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    Point(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    inline Point& operator=(const Point& other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    double x;
    double y;
    double z;
};

}

#endif /* POINT_H */

