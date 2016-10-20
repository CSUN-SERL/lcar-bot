/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include <QSet>

#include "rqt_gcs/vehicle_manager.h"
#include "vehicle/uav_control.h"
#include "lcar_msgs/InitResponse.h"

namespace rqt_gcs
{
    
//public://///////////////////////////////////////////////////////////////////// 
    
VehicleManager::VehicleManager(QObject *parent):
QObject(parent)
{   
    init_request_server = nh.advertiseService("vehicle/init/request", &VehicleManager::OnVehicleInitRequested, this);
    init_response_client = nh.serviceClient<lcar_msgs::InitResponse>("vehicle/init/response");
}

VehicleManager::~VehicleManager()
{
}

void VehicleManager::AddVehicleById(int id)
{
    if(id <= VehicleType::invalid_low || VehicleType::invalid_high <= id)
    {
        //emit NotifyOperator();
        ROS_ERROR_STREAM ("tried to add Vehicle with invalid id: " << id);
        return;
    }
    int v_type = (id / VEHICLE_TYPE_MAX) * VEHICLE_TYPE_MAX; // remove singles digit
    
    if(v_type == VehicleType::vtol) // vtol
        this->AddVTOL(id);
    else if(v_type == VehicleType::octo_rotor) // octo-rotor
        this->AddOctoRotor(id);
    else if(v_type == VehicleType::quad_rotor) // quad-rotor
        this->AddQuadRotor(id);
    else if(v_type == VehicleType::ugv)
        this->AddUGV(id);
    else if(v_type == VehicleType::humanoid)
    {
        //todo
    }    
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
    Q_ASSERT(VehicleType::quad_rotor < id && id < VehicleType::quad_rotor + VEHICLE_TYPE_MAX);
    VehicleControl *v = new UAVControl(id);
    db.insert(id, v);
    NUM_QUAD++;
}

void VehicleManager::AddOctoRotor(int id)
{
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
    UAVControl* uav = (UAVControl*)this->EraseVehicleFromDB(id + VehicleType::quad_rotor);
    if(uav != nullptr)
    {
        delete uav;
        NUM_QUAD--;
    }
}

void VehicleManager::DeleteOctoRotor(int id)
{
    UAVControl* uav = (UAVControl*)this->EraseVehicleFromDB(id + VehicleType::quad_rotor);
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

QString VehicleManager::TypeStringFromId(int id)
{
    int v_type = (id / VEHICLE_TYPE_MAX) * VEHICLE_TYPE_MAX; // remove singular 
    if(v_type == VehicleType::humanoid)
        return "humanoid";
    if(v_type == VehicleType::vtol)
        return "vtol";
    if(v_type == VehicleType::octo_rotor)
        return "octo-rotor";
    if(v_type == VehicleType::quad_rotor)
        return "quad-rotor";
    if(v_type == VehicleType::ugv)
        return "ugv";
    else
        return QString::null; 
}

int VehicleManager::IdFromMachineName(const QString& machine_name)
{
    Qt::CaseSensitivity cs = Qt::CaseSensitivity::CaseInsensitive;
    if(machine_name.contains("quad", cs))
        return VehicleType::quad_rotor + NUM_QUAD;
    if(machine_name.contains("octo", cs))
        return VehicleType::octo_rotor + NUM_OCTO;
    if(machine_name.contains("vtol", cs))
        return VehicleType::vtol + NUM_VTOL;
    if(machine_name.contains("ugv", cs))
        return VehicleType::ugv + NUM_UGV;
    else
        return -1;
}

const QList<QString> VehicleManager::GetInitRequests()
{
    return init_requests.values();
}

int VehicleManager::NumVehicles()
{
    return NUM_UGV + NUM_QUAD + NUM_OCTO + NUM_VTOL;
}

int VehicleManager::NumUGVs()
{
    return NUM_UGV;
}

int VehicleManager::NumQuadRotors()
{
    return NUM_QUAD;
}

int VehicleManager::NumOctoRotors()
{
    return NUM_OCTO;
}

int VehicleManager::NumVTOLs()
{
    return NUM_VTOL;
}


//public slots://///////////////////////////////////////////////////////////////

void VehicleManager::OnOperatorInitRequested(const QString& machine_name)
{
    QSet<QString>::Iterator it = init_requests.find(machine_name);
    if(it != init_requests.end())
    {
        lcar_msgs::InitResponse res;
        res.request.machine_name = machine_name.toStdString();
        res.request.vehicle_id = this->IdFromMachineName(machine_name);
        if(init_response_client.call(res))
        {
            this->AddVehicleById(res.request.vehicle_id);
            init_requests.erase(it);
        }
        else
        {
            ROS_ERROR_STREAM("Operator initialization request "
                            "for vehicle with id: " 
                            << res.request.vehicle_id
                            << " failed!");
        }
    }
}


//private://////////////////////////////////////////////////////////////////////

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
        ROS_ERROR_STREAM("Tried to delete non existent " 
                        << this->TypeStringFromId(id).toStdString()
                        << " with id: "  
                        << (id));
    }
    
    return vehicle;
}

bool VehicleManager::OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req, 
                                            const lcar_msgs::InitRequest::Response& res)
{
    init_requests.insert(req.machine_name.c_str());
    emit NotifyOperator();
    return true;
}

}