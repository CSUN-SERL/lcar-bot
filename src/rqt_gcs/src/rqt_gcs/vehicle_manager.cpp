/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include <QStringBuilder>

#include "rqt_gcs/vehicle_manager.h"
#include "vehicle/uav_control.h"
#include "lcar_msgs/InitResponse.h"

namespace rqt_gcs
{
    
//public://///////////////////////////////////////////////////////////////////// 
    
VehicleManager::VehicleManager(QObject *parent):
    QObject(parent),
    UGV_ID(0),
    QUAD_ID(0),
    OCTO_ID(0),
    VTOL_ID(0)
{   
    srv_init_request = nh.advertiseService("vehicle/init/request", &VehicleManager::OnVehicleInitRequested, this);
    pub_init_response = nh.advertise<lcar_msgs::InitResponse>("vehicle/init/response", 2);
}

VehicleManager::~VehicleManager()
{
}

void VehicleManager::AddVehicle(int id)
{
    if(id <= VehicleType::invalid_low || VehicleType::invalid_high <= id)
    {
        emit NotifyOperator("tried to add Vehicle with invalid id: " % QString::number(id));
        ROS_ERROR_STREAM("tried to add Vehicle with invalid id: " << id);
        return;
    }
    int v_type = this->VehicleTypeFromId(id);
    VehicleControl* vehicle = nullptr;
    
    //todo uncomment these after their classes are added to the system
//    if(v_type == VehicleType::ugv)
//        this->AddVehicle(new UGVControl(id), db[v_type], NUM_UGV);
    if(v_type == VehicleType::quad_rotor) // quad-rotor
        vehicle = new UAVControl(id);
    if(v_type == VehicleType::octo_rotor) // octo-rotor
        vehicle = new UAVControl(id);
//    if(v_type == VehicleType::vtol) // vtol
//        vehicle = new VTOLControl(id);
//    if(v_type == VehicleControl::humanoid) // humanoid
//        vehicle = new NAOControl(id);
    
    if(vehicle != nullptr)
    {
        db[v_type].insert(id, vehicle);
        ROS_INFO_STREAM("Added vehicle of type: "
                        << this->VehicleStringFromId(id).toStdString()
                        << " with id: " << id <<  " to database.");
    }
    else
    {
        ROS_ERROR_STREAM("Tried to add vehicle of invalid type: "
                         << this->VehicleStringFromId(id).toStdString());
    }
}

void VehicleManager::DeleteVehicle(int v_id)
{          
    // there should only be one vehicle with this id
    int v_type = this->VehicleTypeFromId(v_id);
    
    ROS_ASSERT(db.find(v_type) != db.end());
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    QMap<int, VehicleControl*>::Iterator it = v_db->find(v_id);
    
    if(it != v_db->end())
    {
        VehicleControl* vehicle = it.value();
        v_db->erase(it);
        delete vehicle;
    }
    else
    {
        ROS_ERROR_STREAM("Tried to delete non existent " 
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: "  
                        << (v_id));
    }
    
}

void VehicleManager::SetWaypoint(std::string v_string, const sensor_msgs::NavSatFix& location) 
{
    int v_id = this->IdfromVehicleString(QString(v_string.c_str()));
    if(v_id == VehicleType::invalid_low)
    {
        ROS_ERROR_STREAM("Cannot set waypoint: " << v_string  
                         << " is not a recognized vehicle");
    }
    else
       this->SetWaypoint(v_id, location);
}

void VehicleManager::SetWaypoint(int v_id, const sensor_msgs::NavSatFix& location)
{
    int v_type = this->VehicleTypeFromId(v_id);
    
    ROS_ASSERT(db.find(v_type) != db.end());
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    
    if(v_type == VehicleType::ugv)
        ROS_WARN_STREAM("ignoring altitude for ugv");

    QMap<int, VehicleControl*>::ConstIterator it = v_db->find(v_id);
    if(it != v_db->end())
    {   
        VehicleControl * vc = it.value();
        vc->SetWayPoint(location);
    }
    else
        ROS_ERROR_STREAM("Cannot set waypoint: No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

int VehicleManager::NumVehicles()
{
    return this->NumUGVs() +
           this->NumQuadRotors() +
           this->NumOctoRotors() +
           this->NumVTOLs();
}

int VehicleManager::NumUGVs()
{
    return db[VehicleType::ugv].size();
}

int VehicleManager::NumQuadRotors()
{
    return db[VehicleType::quad_rotor].size();
}

int VehicleManager::NumOctoRotors()
{
    return db[VehicleType::octo_rotor].size();
}

int VehicleManager::NumVTOLs()
{
    return db[VehicleType::vtol].size();
}

const QMap<int, QString>& VehicleManager::GetInitRequests()
{
    return init_requests;
}

QString VehicleManager::VehicleStringFromId(int id)
{
    int v_type = this->VehicleTypeFromId(id);

    switch(v_type)
    {
        case VehicleType::ugv:        return "ugv";
        case VehicleType::quad_rotor: return "quad-rotor";
        case VehicleType::octo_rotor: return "octo-rotor";
        case VehicleType::humanoid:   return "humanoid";
        default:                      return QString();  
    }
}

int VehicleManager::GenerateId(const QString& machine_name)
{
    Qt::CaseSensitivity cs = Qt::CaseSensitivity::CaseInsensitive;
    return machine_name.contains("ugv", cs)  ? VehicleType::ugv + UGV_ID++  :
           machine_name.contains("quad", cs) ? VehicleType::quad_rotor + QUAD_ID++ :
           machine_name.contains("octo", cs) ? VehicleType::octo_rotor + OCTO_ID++ :
           machine_name.contains("vtol", cs) ? VehicleType::vtol + VTOL_ID++ :
                                               VehicleType::invalid_low;
}

int VehicleManager::VehicleTypeFromId(int id)
{ 
    return (id / VEHICLE_TYPE_MAX) * VEHICLE_TYPE_MAX;
}


//public slots://///////////////////////////////////////////////////////////////

void VehicleManager::OnOperatorInitResponse(const int vehicle_id)
{
    QMap<int, QString>::Iterator it = init_requests.find(vehicle_id);
    if(it != init_requests.end())
    {
        lcar_msgs::InitResponse res;
        res.vehicle_id = it.key(); // vehicle_id
        res.machine_name = it.value().toStdString(); // machine_name
        pub_init_response.publish(res);
        init_requests.erase(it);
    }
}

//private://////////////////////////////////////////////////////////////////////

bool VehicleManager::OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req, 
                                            lcar_msgs::InitRequest::Response& res)
{
    // check that this vehicle hasn't already requested initialization
    QMap<int, QString>::Iterator it = init_requests.begin();
    for(; it != init_requests.end(); it++)
    {
        if(it.value().toStdString() == req.machine_name)
        {
            res.vehicle_id = VehicleType::invalid_low;
            res.ack = false;
            res.message = "this vehicle already requested initilization";
            return false;
        }
    }
    
    res.vehicle_id = this->GenerateId(QString(req.machine_name.c_str()));
    if(res.vehicle_id != VehicleType::invalid_low)
    {
        res.ack = true;
        init_requests.insert(res.vehicle_id, req.machine_name.c_str());
        res.message = "request for " + req.machine_name + " acknowledged with id: " + std::to_string(res.vehicle_id);
        emit NotifyOperator("Vehicle initialization requested");
        emit AddToInitWidget(QString(req.machine_name.c_str()), res.vehicle_id);

        return true;
    }
    else
    {
        res.message ="ehicle Initilization request: " 
                + req.machine_name + " is not a recognized vehicle type";
        
        ROS_ERROR_STREAM("res.message");
        
        res.ack = false;
        return false;
    }
        
}

int VehicleManager::IdfromVehicleString(QString v_type)
{
    int number;
    sscanf(v_type.toStdString().c_str(), "%d", &number);
    
    Qt::CaseSensitivity cs = Qt::CaseSensitivity::CaseInsensitive;
    return v_type.contains("ugv", cs)  ? VehicleType::ugv + number:
           v_type.contains("quad", cs) ? VehicleType::quad_rotor + number:
           v_type.contains("octo", cs) ? VehicleType::octo_rotor + number:
           v_type.contains("vtol", cs) ? VehicleType::vtol + number:
                                         VehicleType::invalid_low;
}

}