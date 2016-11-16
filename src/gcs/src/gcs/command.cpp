
/* 
 * File:   Voce.cpp
 * Author: serl
 * 
 * Created on November 11, 2016, 11:36 AM
 */

#include "util/command.h"
#include "util/data_types.h"
namespace gcs
{
Command::Command() {  
}

Command::~Command() {
}

int Command::IdFromVehicleString(QString v_type)
{
    int number;
    sscanf(v_type.toStdString().c_str(), "%d", &number);
    
    Qt::CaseSensitivity cs = Qt::CaseSensitivity::CaseInsensitive;
    return v_type.contains("ugv", cs)  ? VehicleType::ugv + number :
           v_type.contains("quad", cs) ? VehicleType::quad_rotor + number :
           v_type.contains("octo", cs) ? VehicleType::octo_rotor + number :
           v_type.contains("vtol", cs) ? VehicleType::vtol + number :
                                         VehicleType::invalid_low;
}
}