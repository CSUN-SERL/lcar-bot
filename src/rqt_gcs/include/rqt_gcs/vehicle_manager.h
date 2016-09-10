
/* 
 * File:   vehicle_manager.h
 * Author: n8
 *
 * Created on September 8, 2016, 11:41 AM
 */

#ifndef VEHICLEMANAGER_H
#define VEHICLEMANAGER_H

#include "vehicle_control.h"


namespace rqt_gcs
{

enum VehicleType
{
    invalid = -1,
    ugv,
    quadrotor_light,
    quadrotor_med,
    octorotor
};

class VehicleManager
{
public:
    VehicleManager();
    virtual ~VehicleManager();
private:
    
};

}

#endif /* VEHICLEMANAGER_H */

