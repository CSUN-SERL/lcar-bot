/* 
 * File:   vehicle_control.h
 * Author: n8
 *
 * Created on September 7, 2016, 5:09 PM
 */

#ifndef VEHICLECONTROL_H
#define VEHICLECONTROL_H

#include <string>

#include <ros/timer.h>
#include <ros/publisher.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>

#include <vehicle/data_types.h>
#include <vehicle/position.h>

namespace gcs
{

#define VEHICLE_TYPE_MAX 1000
    
enum VehicleType
{   // the id space for each vehicle type  
    invalid_low =             VEHICLE_TYPE_MAX - 1,
    ugv =                     VEHICLE_TYPE_MAX, //1000
    quad_rotor =        ugv + VEHICLE_TYPE_MAX, //2000
    octo_rotor = quad_rotor + VEHICLE_TYPE_MAX, //3000
    vtol =       octo_rotor + VEHICLE_TYPE_MAX, //4000
    invalid_high =     vtol + VEHICLE_TYPE_MAX
};


class VehicleControl
{
    
public:
    const int id; // child classes provide id to this class' constructor
    
    VehicleControl(int id): id(id){};
    virtual ~VehicleControl(){};
    virtual void Arm(bool value) = 0;
    virtual bool IsArmed() = 0;
    virtual void SetMode(std::string)=0;
//    virtual void SetWayPoint(const sensor_msgs::NavSatFix& location){}; // for global
//    virtual void SetWayPoint(double lat, double lng, double alt){}; // overload for global
    virtual void SetTarget(geometry_msgs::Pose&)=0; 
    virtual void SetTarget(double lat, double lng, double alt) = 0;
    virtual int GetDistanceToWP() = 0;
//    virtual sensor_msgs::NavSatFix GetLocation()=0;
    virtual void SetRTL() = 0; //makes the drone go home immediately when called
    virtual void StartMission() = 0;//{ mission_mode = active; };
    virtual void PauseMission() = 0;//{ mission_mode = paused; };
    virtual void ResumeMission() = 0;//{ mission_mode = active; };
    virtual void StopMission() = 0;//{ mission_mode = stopped; };
    virtual MissionMode GetMissionMode(){ return mission_mode; };
    virtual float GetMissionProgress() { return -1; };
    virtual float GetBattery() { return -1; };
    virtual std::string GetMode() { return  mode; };
    
    virtual void SetMission(std::vector<geometry_msgs::Pose> waypoints_list) = 0;
    
    bool RecievedHeartbeat() { return heartbeat_recieved; };
    virtual bool MissionComplete()
    { 
        if(mission_mode == stopped){
            return true;
        }
        else if(mission_mode == MissionMode::invalid)
        {
            return false;
        }
        
        return mission_completed;
    };
    
    virtual Position getPosition() = 0;
    
    int currentWaypoint()
    {
        return cur_waypoint;
    }
    
protected:

    virtual void Run()=0;
    virtual void RunLocal()=0;
    virtual void RunGlobal()=0;
    
    int cur_waypoint = 0;
    
    ros::Timer run_timer;
    MissionMode mission_mode = stopped;
    std::string mode; // flight mode or related
    
    ros::Publisher                  pub_heartbeat;
    bool                            connection_dropped = false,
                                    heartbeat_recieved = true,
                                    mission_completed = false;
    ros::Timer                      timer_heartbeat_uav,
                                    timer_heartbeat_gcs;
    std_msgs::Int32                 gcs_heartbeat,
                                    uav_heartbeat;
    
};

}
#endif /* VEHICLECONTROL_H */

