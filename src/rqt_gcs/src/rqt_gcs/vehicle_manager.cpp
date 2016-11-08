/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include <QStringBuilder>
#include <QSettings>

#include "rqt_gcs/vehicle_manager.h"
#include "vehicle/uav_control.h"
#include "lcar_msgs/InitResponse.h"
#include "util/strings.h"

namespace rqt_gcs
{
    
//public://///////////////////////////////////////////////////////////////////// 
    
VehicleManager::VehicleManager(QObject *parent):
    QObject(parent),
    UGV_ID(0),
    QUAD_ID(0),
    OCTO_ID(0),
    VTOL_ID(0),
    it_stereo(nh)
{   
    srv_init_request = nh.advertiseService("vehicle/init/request", 
                                           &VehicleManager::OnVehicleInitRequested, this);
    pub_init_response = nh.advertise<lcar_msgs::InitResponse>("vehicle/init/response", 2);
    
    this->AdvertiseObjectDetection();
}

VehicleManager::~VehicleManager()
{
}

void VehicleManager::AddVehicle(int v_id)
{
    if(v_id <= VehicleType::invalid_low || VehicleType::invalid_high <= v_id)
    {
        emit NotifyOperator("tried to add Vehicle with invalid id: " % QString::number(v_id));
        ROS_ERROR_STREAM("tried to add Vehicle with invalid id: " << v_id);
        return;
    }
    
    int v_type = this->VehicleTypeFromId(v_id);
    VehicleControl* vehicle = nullptr;
    
    //todo uncomment these after their classes are added to the system
//    if(v_type == VehicleType::ugv)
//        vehicle = new UGVControl(id);
    if(v_type == VehicleType::quad_rotor) // quad-rotor
        vehicle = new UAVControl(v_id);
    if(v_type == VehicleType::octo_rotor) // octo-rotor
        vehicle = new UAVControl(v_id);
//    if(v_type == VehicleType::vtol) // vtol
//        vehicle = new VTOLControl(id);
//    if(v_type == VehicleControl::humanoid) // humanoid
//        vehicle = new NAOControl(id);
    
    if(vehicle != nullptr)
    {
        db[v_type].insert(v_id, vehicle);
        ROS_INFO_STREAM("Added vehicle of type: "
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: " << v_id <<  " to database.");
        
        emit AddVehicleWidget(v_id);
    }
    else
        ROS_ERROR_STREAM("Tried to add vehicle of invalid type: "
                         << this->VehicleStringFromId(v_id).toStdString());
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
        
        emit DeleteVehicleWidget(v_id);
    }
    else
        ROS_ERROR_STREAM("Tried to delete non existent " 
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: "  
                        << (v_id));
}

int VehicleManager::NumTotalVehicles()
{
    return this->NumUGVs() +
           this->NumQuadRotors() +
           this->NumOctoRotors() +
           this->NumVTOLs();
}

int VehicleManager::NumVehiclesByType(int v_type)
{
    if(v_type <= VehicleType::invalid_low || v_type >= VehicleType::invalid_high)
        return VehicleType::invalid_low;
    
    return  db[v_type].size();
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
    if(id <= VehicleType::invalid_low || id >= VehicleType::invalid_high)
        return VehicleType::invalid_low;

    return (id / VEHICLE_TYPE_MAX) * VEHICLE_TYPE_MAX;
}

const QMap<int, QString>& VehicleManager::GetInitRequests()
{
    return init_requests;
}

void VehicleManager::SubscribeToImageTopic(QString& topic)
{
    std::string new_topic = topic.toStdString();
    ROS_INFO_STREAM("subscribing to new image topic: " << new_topic);
    sub_stereo = it_stereo.subscribe(new_topic, 30,
                                     &VehicleManager::ImageCallback, this);
}

ObjectDetectionParameters* VehicleManager::GetObjectDetectionParams()
{
    return &od_params;
}

void VehicleManager::AdvertiseObjectDetection()
{
    od_handlers.pub_hit_thresh = nh.advertise<std_msgs::Float64>("/object_detection/hit_threshold", 5);
    od_handlers.pub_step_size = nh.advertise<std_msgs::Int32>("/object_detection/step_size", 5);
    od_handlers.pub_padding = nh.advertise<std_msgs::Int32>("/object_detection/padding", 5);
    od_handlers.pub_scale_factor = nh.advertise<std_msgs::Float64>("/object_detection/scale_factor", 5);
    od_handlers.pub_mean_shift = nh.advertise<std_msgs::Int32>("/object_detection/mean_shift_grouping", 5);
    od_handlers.sub_od_request = nh.subscribe("/object_detection/param_request", 5,
                                              &VehicleManager::ReceivedObjectDetectionRequest, this);
}

void VehicleManager::PublishHitThreshold(double thresh)
{
    std_msgs::Float64 msg;
    msg.data = thresh;
    od_handlers.pub_hit_thresh.publish(msg);
}

void VehicleManager::PublishStepSize(int step)
{
    std_msgs::Int32 msg;
    msg.data = step;
    od_handlers.pub_step_size.publish(msg);
}

void VehicleManager::PublishPadding(int padding)
{
    std_msgs::Int32 msg;
    msg.data = padding;
    od_handlers.pub_padding.publish(msg);
}

void VehicleManager::PublishScaleFactor(double scale)
{
    std_msgs::Float64 msg;
    msg.data = scale;
    od_handlers.pub_scale_factor.publish(msg);
}

void VehicleManager::PublishMeanShift(bool on)
{
    std_msgs::Int32 msg;
    msg.data = on;
    od_handlers.pub_mean_shift.publish(msg);
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
        this->AddVehicle(res.vehicle_id); 
        init_requests.erase(it);
    }
}

void VehicleManager::SetWaypoint(int v_id, const sensor_msgs::NavSatFix& location)
{
    int v_type = this->VehicleTypeFromId(v_id);
    
    ROS_ASSERT(db.find(v_type) != db.end());
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    
    if(v_type == VehicleType::ugv)
        ROS_WARN_STREAM("ignoring altitude for ugv");
        // note this is ignored inside of the UGVControl code, to be added

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

void VehicleManager::SetWaypoint(std::string v_string, const sensor_msgs::NavSatFix& location) 
{
    int v_id = this->IdfromVehicleString(QString(v_string.c_str()));
    if(v_id != VehicleType::invalid_low)
        this->SetWaypoint(v_id, location);
    else
        ROS_ERROR_STREAM("Cannot set waypoint: " << v_string  
                         << " is not a recognized vehicle");
}

void VehicleManager::Arm(int v_id, bool value)
{
    int v_type = this->VehicleTypeFromId(v_id);
    
    ROS_ASSERT(db.find(v_type) != db.end());
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    auto it = v_db->find(v_id);
    if(it != v_db->end())
    {
        VehicleControl* vc = it.value();
        vc->Arm(value);
    }
    else
        ROS_ERROR_STREAM("Cannot arm vehicle: No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

void VehicleManager::Arm(std::string v_string, bool value)
{
    int v_id = this->IdfromVehicleString(QString(v_string.c_str()));
    if(v_id != VehicleType::invalid_low)
        this->Arm(v_id, value);
    else
        ROS_ERROR_STREAM("Cannot Arm vehicle: " << v_string  
                         << " is not a recognized vehicle");
}

void VehicleManager::SetFlightMode(int v_id, std::string mode)
{
    int v_type = this->VehicleTypeFromId(v_id);
    
    ROS_ASSERT(db.find(v_type) != db.end());
    ROS_ASSERT(v_type == VehicleType::quad_rotor ||
               v_type == VehicleType::octo_rotor ||
               v_type == VehicleType::vtol);
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    auto it = v_db->find(v_id);
    if(it != v_db->end())
    {
        UAVControl* vc = static_cast<UAVControl*>(it.value());
        vc->SetMode(mode);
    }
    else
        ROS_ERROR_STREAM("Cannot set Flight mode for UAV. No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

void VehicleManager::SetFlightMode(std::string v_string, std::string mode)
{
    int v_id = this->IdfromVehicleString(QString(v_string.c_str()));
    if(v_id != VehicleType::invalid_low)
        this->SetFlightMode(v_id, mode);
    else
        ROS_ERROR_STREAM("Cannot set flight mode for UAV: " << v_string  
                         << " is not a recognized UAV");
}


UGVInfoPtr VehicleManager::GetUGVInfo(int ugv_id)
{
    int v_type = this->VehicleTypeFromId(ugv_id);
    
    ROS_ASSERT(v_type == VehicleType::ugv);
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    UGVControl *uav = nullptr;
    UGVInfoPtr ptr;
    
    auto it = v_db->find(ugv_id);
    if(it != v_db->end())
    {
        ptr = boost::make_shared<UGVInfo>();
        //todo fill this out when UGVControl is finalized
//      ptr->id = ugv_id;
//      ptr->v_type = v_type;
    }
    else
        ROS_ERROR_STREAM("Cannot retrieve UGV info for UGV: no such " 
                << this->VehicleStringFromId(ugv_id).toStdString() << " with id: " 
                << ugv_id);
    
    return ptr;
}

UAVInfoPtr VehicleManager::GetUAVInfo(int uav_id)
{
    int v_type = this->VehicleTypeFromId(uav_id);
    
    ROS_ASSERT(v_type == VehicleType::quad_rotor ||
               v_type == VehicleType::octo_rotor ||
               v_type == VehicleType::vtol);
   
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    UAVControl *uav = nullptr;
    UAVInfoPtr ptr;
    
    auto it = v_db->find(uav_id);
    if(it != v_db->end())
    {
        ptr = boost::make_shared<UAVInfo>();
        uav = static_cast<UAVControl*>(it.value());
        ptr->id = uav_id;
        ptr->v_type = v_type;
        ptr->flight_state = uav->GetFlightState();
        ptr->state.battery = uav->GetBatteryState().percentage;
        ptr->state.armed = uav->GetState().armed;
        ptr->state.mission_progress = uav->GetMissionProgress();
    }
    else
        ROS_ERROR_STREAM("Cannot retrieve UAV info for UAV: no such " 
                << this->VehicleStringFromId(uav_id).toStdString() << " with id: " 
                << uav_id);
    
    return ptr;
}

void VehicleManager::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    QPixmap img = img::rosImgToQpixmap(msg);
    emit NewImageFrame(img);
}

void VehicleManager::ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg)
{
    ROS_INFO_STREAM("received object detection paramater request");
    this->PublishHitThreshold(od_params.hit_thresh);
    this->PublishStepSize(od_params.step_size);
    this->PublishPadding(od_params.padding);
    this->PublishScaleFactor(od_params.scale_factor);
    this->PublishMeanShift(od_params.mean_shift);
}

//private://////////////////////////////////////////////////////////////////////

// SettingsWidget and QSettings related stuff
void VehicleManager::InitSettings()
{
    QSettings settings(COMPANY, APPLICATION);
    
    settings.beginGroup("object_detection_tab");

    QString params = "tuning_paramaters";
    od_params.hit_thresh = settings.value(params % "/hit_threshold", od_params.hit_thresh).toDouble();
    od_params.step_size = settings.value(params % "/step_size", od_params.step_size).toInt();
    od_params.padding = settings.value(params % "/padding", od_params.padding).toInt();
    od_params.scale_factor = settings.value(params % "/scale_factor", od_params.scale_factor).toDouble();
    od_params.mean_shift = settings.value(params % "/mean_shift_grouping", od_params.mean_shift).toBool();

    settings.endGroup();
}

bool VehicleManager::OnVehicleInitRequested(lcar_msgs::InitRequest::Request& req, 
                                            lcar_msgs::InitRequest::Response& res)
{
    // check that this vehicle hasn't already requested initialization
    for(auto it = init_requests.begin(); it != init_requests.end(); it++)
    {
        if(it.value().toStdString() == req.machine_name)
        {
            res.vehicle_id = VehicleType::invalid_low;
            res.ack = false;
            res.message = req.machine_name + " already requested initilization";
            return false;
        }
    }
    
    res.vehicle_id = this->GenerateId(QString(req.machine_name.c_str()));
    if(res.vehicle_id != VehicleType::invalid_low)
    {
        res.ack = true;
        init_requests.insert(res.vehicle_id, req.machine_name.c_str());
        res.message = "request for " + req.machine_name + " acknowledged with id: " 
                    + std::to_string(res.vehicle_id);
        
        emit NotifyOperator("Vehicle initialization requested");
        emit AddToInitWidget(QString(req.machine_name.c_str()), res.vehicle_id);

        return true;
    }
    else
    {
        res.message = "vehicle Initilization request: " 
                    + req.machine_name + " is not a recognized vehicle type";
        
        ROS_ERROR_STREAM(res.message);
        
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