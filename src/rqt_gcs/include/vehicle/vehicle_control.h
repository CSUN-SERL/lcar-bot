/* 
 * File:   vehicle_control.h
 * Author: n8
 *
 * Created on September 7, 2016, 5:09 PM
 */

#ifndef VEHICLECONTROL_H
#define VEHICLECONTROL_H

#include <string>
#include <sensor_msgs/NavSatFix.h>

#include "util/data_types.h"

class VehicleControl
{
    
public:
    const int id; // child classes provide id to this class' constructor
    
    VehicleControl(int id): id(id){ }
    virtual ~VehicleControl(){};
    virtual void Arm(bool value)=0;
    virtual void SetMode(std::string)=0;
    virtual sensor_msgs::NavSatFix GetLocation()=0;
    virtual void SetRTL()=0;
    virtual void StartMission(){}; // todo make pure virtual and add override implementation to UAVControl
    virtual void PauseMission()=0;
    virtual void ResumeMission()=0;
    virtual void StopMission()=0;
    
protected:

    virtual void Run()=0;
    virtual void RunLocal()=0;
    virtual void RunGlobal()=0;
    //virtual void SetWayPoint()=0; // todo implement this in UAVControl
    
};

#endif /* VEHICLECONTROL_H */

