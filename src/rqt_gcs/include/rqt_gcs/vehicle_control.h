/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   VehicleControl.h
 * Author: serl
 *
 * Created on September 7, 2016, 5:09 PM
 */

#ifndef VEHICLECONTROL_H
#define VEHICLECONTROL_H

class VehicleControl
{
public:
    VehicleControl(){};
    virtual ~VehicleControl(){};
    
    virtual void Arm(bool value)=0;
    virtual void SetMode(std::string mode)=0;
    virtual sensor_msgs::NavSatFix GetLocation()=0;
    virtual void SetRTL()=0;
    virtual void StartMission(){}; // todo make pure virtual and add override implementation to UAVControl
    virtual void PauseMission()=0;
    virtual void ResumeMission()=0;
    virtual void StopMission()=0;
    
    void SetId(int id){ this->id = id; }
    int GetId() { return id; }
    
protected:

    virtual void Run()=0;
    virtual void RunLocal()=0;
    virtual void RunGlobal()=0;
    //virtual void SetWayPoint()=0;
    
    int id;
    
//private:
    
};

#endif /* VEHICLECONTROL_H */

