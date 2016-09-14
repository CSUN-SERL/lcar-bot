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

typedef enum VehicleType_
{
    invalid = -1,
    ugv = 1000,      //corresponds to the vehicle id range for this vehicle type
    quad_rotor = 2000,
    octo_rotor = 3000,
    vtol = 4000,
    humanoid = 5000
} VehicleType;
    
typedef enum PositionMode_
{
    local,
    global
} PositionMode;

typedef enum MissionMode_
{
    active,
    paused,
    stopped
} MissionMode;

typedef enum Mode_
{
    travel,
    hold,
    scout,
    rtl,
    land,
    disarm,
    idle,
    null
} Mode;

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

