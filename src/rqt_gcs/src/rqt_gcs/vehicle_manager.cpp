/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include "rqt_gcs/vehicle_manager.h"
#include "vehicle/uav_control.h"

namespace rqt_gcs
{
    
//public: 
    
VehicleManager::VehicleManager(QObject *parent):
QObject(parent)
{
}

VehicleManager::~VehicleManager()
{
}

//all public Add* functions assume that the id passed to them are between 0 and 99.
//the functions themselves map the id to the appropriate space
void VehicleManager::AddUGV(int id)
{
    //todo add UGVControl
    //this->AddVehicleByType(VehicleType::ugv, id);
    NUM_UGV++;
}

void VehicleManager::AddQuadRotor(int id)
{   
    id += VehicleType::quad_rotor; // map to quad_rotor id space
    Q_ASSERT(VehicleType::quad_rotor < id && id < VehicleType::quad_rotor + VEHICLE_TYPE_MAX);
    VehicleControl *v = new UAVControl(id);
    db.insert(id, v);
    NUM_QUAD++;
}

void VehicleManager::AddOctoRotor(int id)
{
    id += VehicleType::octo_rotor; // map to octo_rotor id space
    Q_ASSERT(VehicleType::octo_rotor <= id && id < VehicleType::octo_rotor + VEHICLE_TYPE_MAX);
    VehicleControl *v = new UAVControl(id);
    db.insert(id, v);
    NUM_OCTO++;
}

void VehicleManager::AddVTOL(int id)
{
    // todo add VTOLControl class
    NUM_VTOL++;
}

// like the public Add* functions, these expect id to be between 0 and 99
void VehicleManager::DeleteUGV(int id)
{
//    UGVControl* ugv = (UGVControl*)EraseVehicleFromDB(id + VehicleType::ugv);
//    if(ugv != nullptr)
//    {
//        delete ugv;
//        NUM_UGV--;
//    }
}

void VehicleManager::DeleteQuadRotor(int id)
{
    UAVControl* uav= (UAVControl*)this->EraseVehicleFromDB(id + VehicleType::quad_rotor);
    if(uav != nullptr)
    {
        delete uav;
        NUM_QUAD--;
    }
}

void VehicleManager::DeleteOctoRotor(int id)
{
    UAVControl* uav= (UAVControl*)this->EraseVehicleFromDB(id + VehicleType::quad_rotor);
    if(uav != nullptr)
    {
        delete uav;
        NUM_OCTO--;
    }
}

void VehicleManager::DeleteVTOL(int id)
{
//    VTOLControl * vtol = (UAVControl*)this->EraseVehicleFromDB(id + VehicleType::vtol);
    NUM_VTOL--;
}

QString VehicleManager::VehicleString(int id)
{
    if(id > VehicleType::humanoid)
        return "humanoid";
    else if(id > VehicleType::vtol)
        return "VTOL";
    else if(id > VehicleType::octo_rotor)
        return "octo-rotor";
    else if(id > VehicleType::quad_rotor)
        return "quad-rotor";
    else if(id > VehicleType::ugv)
        return "UGV";
    else
        return QString::null; 
}

//private:

VehicleControl* VehicleManager::EraseVehicleFromDB(int id)
{          
    // there should only be one vehicle with this id
    VehicleControl* vehicle = nullptr;
    QMap<int, VehicleControl*>::iterator it = db.find(id);
    if(it != db.end())
    {
        vehicle = it.value();
        db.erase(it);
    }
    else
    {
        int v_type = (id / VEHICLE_TYPE_MAX) * VEHICLE_TYPE_MAX; // remove singular 
        ROS_ERROR_STREAM("tried to delete non existent " 
                << VehicleString(v_type).toStdString()
                << " with id: "  
                << (id % VEHICLE_TYPE_MAX) );
    }
    return vehicle;
}

}