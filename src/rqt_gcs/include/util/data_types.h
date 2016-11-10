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
    
typedef enum VehicleType_
{   // the id space for each vehicle type  
    invalid_low = VEHICLE_TYPE_MAX - 1,
    ugv =                     VEHICLE_TYPE_MAX, //1000
    quad_rotor =        ugv + VEHICLE_TYPE_MAX, //2000
    octo_rotor = quad_rotor + VEHICLE_TYPE_MAX, //3000
    vtol =       octo_rotor + VEHICLE_TYPE_MAX, //4000
    humanoid =         vtol + VEHICLE_TYPE_MAX, //5000
    invalid_high = humanoid + VEHICLE_TYPE_MAX
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
    stopped,
    invalid
} MissionMode;

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
    float battery;
    int mission_progress;
    std::string mode;
    bool armed;
} State;
typedef boost::shared_ptr<State> StatePtr;

typedef struct GroundState_
{
    float ground_speed, // ground_speed and heading belong to all vehicle types
          heading;
} GroundState;
typedef boost::shared_ptr<GroundState> GroundStatePtr;

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
typedef boost::shared_ptr<FlightState> FlightStatePtr;

typedef struct VehicleInfo_
{
    int id;
    int /*VehicleType*/ v_type;
    State state;
} VehicleInfo;
typedef boost::shared_ptr<VehicleInfo> VehicleInfoPtr;

typedef struct UGVInfo_ : VehicleInfo
{
    GroundState ground_state;
} UGVInfo;
typedef boost::shared_ptr<UGVInfo> UGVInfoPtr;

typedef struct UAVInfo_ : VehicleInfo 
{
    FlightState flight_state;
} UAVInfo;
typedef boost::shared_ptr<UAVInfo> UAVInfoPtr;

}

#endif /* DATA_TYPES_H */

