/* 
 * File:   data_types.h
 * Author: n8
 *
 * Created on September 13, 2016, 3:57 PM
 */

#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <boost/shared_ptr.hpp>

namespace gcs
{
    
//todo ADD FLIGHTMODE STRINGS

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
    disarm, //remove disarm
    idle,
    null
};

typedef struct State_
{
    float battery;
    float mission_progress;
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

