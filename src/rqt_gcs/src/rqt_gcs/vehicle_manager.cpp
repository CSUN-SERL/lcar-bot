/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include "rqt_gcs_no_gui/vehicle_manager.h"

namespace rqt_gcs
{

VehicleManager::VehicleManager(QObject *parent)
{
}

VehicleManager::~VehicleManager()
{
}

void VehicleManager::AddUGV(int id)
{
    //todo add UGVControl
    //this->AddVehicleByType(VehicleType::ugv, id);
    NUM_UGV++;
}

void VehicleManager::AddQuadRotor(int id)
{   
    Q_ASSERT(0 < id && id < (int) VehicleType::quad_rotor);
    id += (int) VehicleType::quad_rotor; // map to quad_rotor id space
    UAVControl * uav = new UAVControl(id);
    this->AddVehicleByType(VehicleType::quad_rotor, uav);
    NUM_QUAD++;
}

void VehicleManager::AddOctoRotor(int id)
{
    Q_ASSERT(0 < id && id < (int) VehicleType::octo_rotor);
    id += (int) VehicleType::octo_rotor; // map to octo_rotor id space
    UAVControl * uav = new UAVControl(id);
    this->AddVehicleByType(VehicleType::octo_rotor, uav);
    NUM_OCTO++;
}

void VehicleManager::AddVTOL(int id)
{
    // todo add VTOLControl class
    //this->AddVehicleByType(VehicleType::vtol, id);
    NUM_VTOL++;
}

void VehicleManager::AddVehicleByType(VehicleType v_type, VehicleControl* vehicle)
{
    Q_ASSERT(v_type != VehicleType::invalid);
    QVector<VehicleControl*> * vehicles = &db[v_type];
    
    int index = 0;
    int size = vehicles->size();
    while (index < size && vehicle->id > vehicles->at(index)->id)
        index++;
    
    vehicles->insert(vehicles->begin()+index, vehicle);
}

}