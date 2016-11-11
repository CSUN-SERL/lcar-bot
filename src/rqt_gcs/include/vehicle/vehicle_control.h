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

namespace rqt_gcs
{

class VehicleControl
{
    
public:
    const int id; // child classes provide id to this class' constructor
    
    VehicleControl(int id): id(id){};
    virtual ~VehicleControl(){};
    virtual void Arm(bool value){};
    virtual bool IsArmed()=0;
    virtual void SetMode(std::string){};
    virtual void SetWayPoint(const sensor_msgs::NavSatFix& location){};
    virtual int GetDistanceToWP(){};
    virtual sensor_msgs::NavSatFix GetLocation()=0;
    virtual void SetRTL(){};
    virtual void StartMission(){ mission_mode = active; }; // todo make pure virtual and add override implementation to UAVControl
    virtual void PauseMission(){ mission_mode = paused; };
    virtual void ResumeMission(){ mission_mode = active; };
    virtual void StopMission(){ mission_mode = stopped; };
    virtual MissionMode GetMissionMode(){ return mission_mode; };
    virtual float GetMissiontProgress() { return -1; }
    virtual int GetBattery() { return battery; };
    virtual std::string GetMode() { return  mode; }
    
protected:

    virtual void Run()=0;
    virtual void RunLocal()=0;
    virtual void RunGlobal()=0;
    
    ros::Timer run_timer;
    MissionMode mission_mode;
    int battery;
    std::string mode;
    
};

}
#endif /* VEHICLECONTROL_H */

