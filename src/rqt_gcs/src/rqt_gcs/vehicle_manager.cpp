/* 
 * File:   vehicle_manager.cpp
 * Author: n8
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include <QSettings>
#include <QTextStream>
#include <QStringBuilder>

#include <ros/package.h>

#include "util/strings.h"
#include "vehicle/uav_control.h"
#include "lcar_msgs/InitResponse.h"
#include "rqt_gcs/vehicle_manager.h"

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
    
    this->InitSettings();
    this->AdvertiseObjectDetection();
}

VehicleManager::~VehicleManager()
{
}

void VehicleManager::AddVehicle(int v_id)
{
    int v_type = this->VehicleTypeFromId(v_id);
    ROS_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleControl* vc = nullptr;
    
    //todo uncomment these after their classes are added to the system
//    if(v_type == VehicleType::ugv)
//        vehicle = new UGVControl(id);
    if(v_type == VehicleType::quad_rotor) // quad-rotor
        vc = new UAVControl(v_id);
    if(v_type == VehicleType::octo_rotor) // octo-rotor
        vc = new UAVControl(v_id);
    if(v_type == VehicleType::vtol) // vtol
        vc = new UAVControl(v_id);
    
    if(vc != nullptr)
    {
        db[v_type].insert(v_id, vc);
        ROS_INFO_STREAM("Added vehicle of type: "
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: " << v_id <<  " to database.");
        
        emit ui_adapter->AddVehicleWidget(v_id);
    }
    else
        ROS_ERROR_STREAM("Tried to add vehicle of invalid type: "
                         << this->VehicleStringFromId(v_id).toStdString());
}

void VehicleManager::DeleteVehicle(int v_id)
{          
    widget_mutex.lock();
    
    int v_type = this->VehicleTypeFromId(v_id);
    
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    QMap<int, VehicleControl*>::Iterator it = v_db->find(v_id);
    if(it != v_db->end())
    {
        emit ui_adapter->DeleteVehicleWidget(v_id);
        VehicleControl *vehicle = it.value();
        v_db->erase(it);

        widget_deleted.wait(&widget_mutex);
        delete vehicle;
    }
    else
        ROS_ERROR_STREAM("Tried to delete non existent " 
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: " << v_id);
    
    widget_mutex.unlock();
}

int VehicleManager::NumTotalVehicles()
{
    return db[VehicleType::ugv].size() +
           db[VehicleType::quad_rotor].size() +
           db[VehicleType::octo_rotor].size() +
           db[VehicleType::vtol].size();
}

int VehicleManager::NumVehiclesByType(int v_type)
{
    if(v_type <= VehicleType::invalid_low || VehicleType::invalid_high <= v_type)
        return VehicleType::invalid_high;
    
    return  db[v_type].size();
}

int VehicleManager::NumUAVs()
{
    return db[VehicleType::quad_rotor].size() +
           db[VehicleType::octo_rotor].size() +
           db[VehicleType::vtol].size();
}

QString VehicleManager::VehicleStringFromId(int v_id)
{
    int v_type = this->VehicleTypeFromId(v_id);
    switch(v_type)
    {
        case VehicleType::ugv:        return "ugv";
        case VehicleType::quad_rotor: return "quad-rotor";
        case VehicleType::octo_rotor: return "octo-rotor";
        case VehicleType::vtol:       return "vtol";
    }
    return QString();
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

void VehicleManager::OnExecutePlay()
{
    //todo
}

void VehicleManager::OnCancelPlay()
{
    //todo
}

void VehicleManager::OnScoutBuilding(int quad_id, QString building)
{
    int v_type = this->VehicleTypeFromId(quad_id);
    Q_ASSERT(v_type == VehicleType::quad_rotor);
    
    QString path = QString(ros::package::getPath("rqt_gcs").c_str()) % "/bulding";
    // can be local or global
    path.append("/" % coordinate_system % "/" % building);
    
    VehicleControl *vc = this->FindVehicle(v_type, quad_id);
    if(vc != nullptr)
    {
        UAVControl* uav = static_cast<UAVControl*> (vc);
        if (coordinate_system == "global")
        {
            lcar_msgs::TargetGlobalPtr target = this->GetTargetGlobal(path);
            if(target)
                uav->ScoutBuilding(*target);
            else
                ROS_ERROR_STREAM("error sending target to uav. no such mission file: "
                        << path.toStdString());
        }
        else
        {
            lcar_msgs::TargetLocalPtr target = this->GetTargetLocal(path);
            if(target)
                uav->ScoutBuilding(*target);
            else
                ROS_ERROR_STREAM("error sending target to uav. no such mission file: "
                        << path.toStdString());
        }
    }
    else
        ROS_ERROR_STREAM("Cannot set waypoint: No such" 
                         << this->VehicleStringFromId(quad_id).toStdString()
                         << "with id: " << quad_id);
}

void VehicleManager::SetWaypoint(int v_id, const sensor_msgs::NavSatFix& location)
{
    VehicleControl *vc = this->FindVehicle(v_id);
    if(vc != nullptr)
        vc->SetWayPoint(location);
    else
        ROS_ERROR_STREAM("Cannot set waypoint: No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

void VehicleManager::Arm(int v_id, bool value)
{
    VehicleControl *vc = this->FindVehicle(v_id);
    if(vc != nullptr)
        vc->Arm(value);
    else
        ROS_ERROR_STREAM("Cannot arm vehicle: No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

void VehicleManager::SetMode(int v_id, std::string mode)
{
    int v_type = this->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type == VehicleType::quad_rotor ||
             v_type == VehicleType::octo_rotor ||
             v_type == VehicleType::vtol);
    
    VehicleControl *vc = this->FindVehicle(v_type, v_id);
    if(vc != nullptr)
        vc->SetMode(mode);
    else
        ROS_ERROR_STREAM("Cannot set Flight mode for UAV. No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

void VehicleManager::SetRTL(int v_id)
{
    VehicleControl *vc = this->FindVehicle(v_id);
    if(vc != nullptr)
        return vc->SetRTL();
    else
        ROS_ERROR_STREAM("Cannot set Flight mode for UAV. No such" 
                         << this->VehicleStringFromId(v_id).toStdString()
                         << "with id: " << v_id);
}

int VehicleManager::IsArmed(int v_id)
{
    VehicleControl *vc = this->FindVehicle(v_id);
    if(vc != nullptr)
        return vc->IsArmed();
    
    ROS_ERROR_STREAM("Cannot get armed state for vehicle. No such" 
                     << this->VehicleStringFromId(v_id).toStdString()
                     << "with id: " << v_id);    
    return -1;
}

std::vector<lcar_msgs::QueryPtr> * VehicleManager::GetUAVDoorQueries(int quad_id)
{
    VehicleControl *vc = this->FindVehicle(quad_id);
    if(vc != nullptr)
    {
        UAVControl *quad = static_cast<UAVControl*> (vc);
        return quad->GetDoorQueries();
    }
    
    ROS_ERROR_STREAM("Cannot get door queries for vehicle. No such" 
                     << this->VehicleStringFromId(quad_id).toStdString()
                     << "with id: " << quad_id);    
    return nullptr;
}

std::vector<lcar_msgs::AccessPointStampedPtr> * VehicleManager::GetUAVAccessPoints(int quad_id)
{
    VehicleControl *vc = this->FindVehicle(quad_id);
    if(vc != nullptr)
    {
        UAVControl *quad = static_cast<UAVControl*> (vc);
        return quad->GetRefAccessPoints();
    }
    
    ROS_ERROR_STREAM("Cannot get door queries for vehicle. No such" 
                     << this->VehicleStringFromId(quad_id).toStdString()
                     << "with id: " << quad_id);    
    return nullptr;
}

FlightState VehicleManager::GetFlightState(int uav_id)
{
    int v_type = this->VehicleTypeFromId(uav_id);
    Q_ASSERT(v_type == VehicleType::quad_rotor ||
             v_type == VehicleType::octo_rotor ||
             v_type == VehicleType::vtol);
    
    VehicleControl *vc = this->FindVehicle(v_type, uav_id);
    if(vc != nullptr)
    {
        UAVControl *uav = static_cast<UAVControl*> (vc);
        return uav->GetFlightState();
    }
    
    ROS_ERROR_STREAM("Cannot retrieve flight state UAV: no such " 
                << this->VehicleStringFromId(uav_id).toStdString() << " with id: " 
                << uav_id);
    
    return FlightState();
}

int VehicleManager::GetDistanceToWP(int v_id)
{
    VehicleControl *vc = this->FindVehicle(v_id);
    if(vc != nullptr)
       return vc->GetDistanceToWP();
    
    ROS_ERROR_STREAM("Cannot retrieve distance to waypoint for vehicle: no such " 
                << this->VehicleStringFromId(v_id).toStdString() << " with id: " 
                << v_id);
    
    return -1;
}

MissionMode VehicleManager::GetMissionMode(int v_id)
{
    VehicleControl *vc = this->FindVehicle(v_id);
    if(vc != nullptr)
        return vc->GetMissionMode();
    
    ROS_ERROR_STREAM("Cannot retrieve MissionMode for vehicle: no such " 
                << this->VehicleStringFromId(v_id).toStdString() << " with id: " 
                << v_id);
    
    return MissionMode::invalid;
}

QMutex* VehicleManager::GetWidgetMutex()
{
    return &widget_mutex;
}

QWaitCondition* VehicleManager::GetWaitCondition()
{
    return &widget_deleted;
}

void VehicleManager::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    QPixmap img = img::rosImgToQpixmap(msg);
    emit ui_adapter->NewImageFrame(img);
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

VehicleControl* VehicleManager::FindVehicle(int v_type, int v_id)
{   
    QMap<int, VehicleControl*> * v_db = &db[v_type];
    QMap<int, VehicleControl*>::Iterator it = v_db->find(v_id);
    if(it != v_db->end()) //found it
        return it.value();
    
    return nullptr; //couldn't find it
}

VehicleControl* VehicleManager::FindVehicle(int v_id)
{
    int v_type = this->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    return this->FindVehicle(v_type, v_id);
}

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

    settings.beginGroup("general_tab");
    
    coordinate_system = settings.value("coordinate_system", "local").toString();
    
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
        
        emit ui_adapter->NotifyOperator("Vehicle initialization requested");
        emit ui_adapter->AddToInitWidget(QString(req.machine_name.c_str()), res.vehicle_id);

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

lcar_msgs::TargetGlobalPtr VehicleManager::GetTargetGlobal(QString target_path)
{
    QFile file(target_path);
    lcar_msgs::TargetGlobalPtr ptr;
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return ptr;

    ptr = boost::make_shared<lcar_msgs::TargetGlobal>();
    QTextStream in(&file);
    
    //todo setup global file read
    
    file.close();
    return ptr;
}

lcar_msgs::TargetLocalPtr VehicleManager::GetTargetLocal(QString target_path)
{
    QFile file(target_path);
    lcar_msgs::TargetLocalPtr ptr;
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return ptr;

    ptr = boost::make_shared<lcar_msgs::TargetLocal>();
    QTextStream in(&file);
    
    QStringList line = in.readLine().split(" ");
    
    ptr->target.position.x = line.at(0).toDouble();
    ptr->target.position.y = line.at(1).toDouble();
    ptr->target.position.z = line.at(2).toDouble();
    ptr->radius = line.at(3).toDouble();
    
    file.close();
    return ptr;
}

}