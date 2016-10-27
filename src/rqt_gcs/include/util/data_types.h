/* 
 * File:   data_types.h
 * Author: n8
 *
 * Created on September 13, 2016, 3:57 PM
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

namespace rqt_gcs
{
    
#define VEHICLE_TYPE_MAX 1000
    
enum VehicleType
{   // the id space for each vehicle type  
    invalid_low = 999,
    ugv =                     VEHICLE_TYPE_MAX, //1000
    quad_rotor =        ugv + VEHICLE_TYPE_MAX, //2000
    octo_rotor = quad_rotor + VEHICLE_TYPE_MAX, //3000
    vtol =       octo_rotor + VEHICLE_TYPE_MAX, //4000
    humanoid =         vtol + VEHICLE_TYPE_MAX, //5000
    invalid_high = humanoid + VEHICLE_TYPE_MAX
};
    
enum PositionMode
{
    local,
    global
};

enum MissionMode
{
    active,
    paused,
    stopped
};

enum Mode
{
    travel,
    hold,
    scout,
    rtl,
    land,
    disarm,
    idle,
    null
};

typedef struct State_
{
    int battery;
    int mission_progress;
    bool armed;
} State;

typedef struct GroundState_
{
    float ground_speed, // ground_speed and heading belong to all vehicle types
          heading;
} GroundState;

typedef struct FlightState_ : GroundState
{   
    float roll,
          pitch,
          yaw,
          altitude,
          vertical_speed,
          horizontal_speed,
          air_speed;
} FlightState;

typedef struct VehicleInfo_
{
    int id;
    VehicleType v_type;
    State state;
} VehicleInfo;

typedef struct UGVInfo_ : VehicleInfo
{
    GroundState state;
} UGVInfo;

typedef struct UAVInfo_ : VehicleInfo 
{
    FlightState flight_state;
} UAVInfo;

}

#endif /* DATA_TYPES_H */

