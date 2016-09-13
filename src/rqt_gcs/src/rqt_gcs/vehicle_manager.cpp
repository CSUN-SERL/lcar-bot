/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include "rqt_gcs/vehicle_manager.h"

namespace rqt_gcs
{

VehicleManager::VehicleManager()
{int index = 0;
//    while (index < vehicles->size() && id < vehicles->at(index)->GetId())
//        index++;
}

VehicleManager::~VehicleManager()
{
}

void VehicleManager::AddUGV(int id)
{
    
    this->AddVehicleByType(VehicleType::ugv, id);
    NUM_UGV++;
}

void VehicleManager::AddQuadRotor(int id)
{
    this->AddQuadRotor(VehicleType::quad_rotor, id);
    NUM_QUAD++;
}

void VehicleManager::AddOctoRotor(int id)
{
    this->AddVehicleByType(VehicleType::octo_rotor, id);
    NUM_OCTO++;
}

void VehicleManager::AddVTOL(int id)
{
    this->AddVehicleByType(VehicleType::vtol, id);
    NUM_VTOL++;
}

void VehicleManager::AddVehicleByType(VehicleType type, int id)
{
    id += (int)type; // map the id into the correct space for its vehicle type
    
    QVector<VehicleControl*> * vehicles = &db[type];
//    int index = 0;
//    while (index < vehicles->size() && id > vehicles->at(index)->GetId())
//        index++;
    
//    vehicles->insert(vehicles->begin()+id, new)
}

}