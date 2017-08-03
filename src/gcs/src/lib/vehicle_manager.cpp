/* 
 * File:   vehicle_manager.cpp
 * Author: n
 * 
 * Created on September 8, 2016, 11:41 AM
 */

#include <QTextStream>
#include <QStringBuilder>

#include <ros/package.h>

#include <std_msgs/Empty.h>
#include <lcar_msgs/InitResponse.h>

#include <vehicle/uav_control.h>

#include <gcs/util/flight_modes.h>
#include <gcs/util/debug.h>
#include <gcs/util/settings.h>

#include <gcs/qt/vehicle_manager.h>
#include <gcs/qt/ui_adapter.h>

#include <vehicle/vehicle_control.h>

namespace gcs
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
    qRegisterMetaType<QVector<gcs::Point>>("QVector<Point>");
    
    srv_init_request = nh.advertiseService("vehicle/init/request", 
                                           &VehicleManager::VehicleInitRequested, this);
    
    srv_world_map = nh.advertiseService("world_map", &VehicleManager::WorldMapRequested, this);
    
    pub_init_response = nh.advertise<lcar_msgs::InitResponse>("vehicle/init/response", 10);
    
    pub_world_map_updated = nh.advertise<std_msgs::Empty>("world_map_updated", 10);
    
    heartbeat_timer = nh.createTimer(ros::Duration(1), &VehicleManager::TimedHeartBeatCheck, this);
    
    _run_timer = nh.createTimer(ros::Duration(0.1), &VehicleManager::runVehicles, this);
    
    this->InitSettings();
    this->AdvertiseObjectDetection();
}

VehicleManager::~VehicleManager()
{
    int v_type = VehicleType::invalid_low + 1;
    for(; v_type < VehicleType::invalid_high; v_type += VEHICLE_TYPE_MAX)
    {
        QMap<int, VehicleControl*> * v_db = &db[v_type];
        for(auto it = v_db->begin(); it != v_db->end(); it++)
        {
            VehicleControl *vc = it.value();
            delete vc;
        }
    }
}

void VehicleManager::ConnectToUIAdapter()
{
    UIAdapter *ui_adapter = UIAdapter::Instance();
    
    //vehicle commands/queries
    connect(ui_adapter, &UIAdapter::Arm,
            this, &VehicleManager::OnArm);
    
    connect(ui_adapter, &UIAdapter::SetWayPoint,
            this, &VehicleManager::OnSetWaypoint);
    
    connect(ui_adapter, &UIAdapter::SetMode,
            this, &VehicleManager::OnSetMode);
    
    connect(ui_adapter, &UIAdapter::ScoutBuilding,
            this, &VehicleManager::OnScoutBuilding);
    
    connect(ui_adapter, &UIAdapter::PauseMission,
            this, &VehicleManager::OnPauseMission);
    
    connect(ui_adapter, &UIAdapter::ResumeMission, 
            this, &VehicleManager::OnResumeMission);
    
    connect(ui_adapter, &UIAdapter::CancelMission,
            this, &VehicleManager::OnCancelPlay);
    
    connect(ui_adapter, &UIAdapter::ExecutePlay,
            this, &VehicleManager::OnExecutePlay);
    
    connect(ui_adapter, &UIAdapter::PausePlay,
            this, &VehicleManager::OnPausePlay);
    
    connect(ui_adapter, &UIAdapter::ResumePlay,
            this, &VehicleManager::OnResumePlay);
    
    connect(ui_adapter, &UIAdapter::CancelPlay,
            this, &VehicleManager::OnCancelPlay);
    
    //machine learning stuff from settings widget
    connect(ui_adapter, &UIAdapter::SetMachineLearningMode,
            this, &VehicleManager::OnSetMachineLearningMode);
    
    connect(ui_adapter, &UIAdapter::PublishHitThreshold,
            this, &VehicleManager::OnPublishHitThreshold);
    
    connect(ui_adapter, &UIAdapter::PublishStepSize,
            this, &VehicleManager::OnPublishStepSize);
    
    connect(ui_adapter, &UIAdapter::PublishPadding,
            this, &VehicleManager::OnPublishPadding);
    
    connect(ui_adapter, &UIAdapter::PublishScaleFactor,
            this, &VehicleManager::OnPublishScaleFactor);
    
    connect(ui_adapter, &UIAdapter::PublishMeanShift,
            this, &VehicleManager::OnPublishMeanShift);
        
    
    //database commands from ui
    connect(ui_adapter, &UIAdapter::AddVehicle,
            this, &VehicleManager::OnOperatorAddVehicle);
    
//    connect(ui_adapter, &UIAdapter::DeleteVehicle,
//            this, &VehicleManager::OnOperatorDeleteVehicle);
    
    
    //coordinate system from settings widget
    connect(ui_adapter, &UIAdapter::SetCoordinateSystem,
            this, &VehicleManager::OnSetCoordinateSystem);
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

float VehicleManager::GetMissionProgress(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        return vc->GetMissionProgress();
    else 
        return -1;
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
    od_handlers.sub_od_request = nh.subscribe("/object_detection/param_request", 1000,
                                              &VehicleManager::ReceivedObjectDetectionRequest, this);
}


QString VehicleManager::VehicleStringFromId(int v_id)
{
    int v_type = VehicleManager::VehicleTypeFromId(v_id);
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

int VehicleManager::VehicleTypeFromId(int v_id)
{   
    if(v_id <= VehicleType::invalid_low || VehicleType::invalid_high <= v_id)
        return VehicleType::invalid_low;

    return (v_id / VEHICLE_TYPE_MAX) * VEHICLE_TYPE_MAX;
}

int VehicleManager::VehicleIndexFromId(int v_id)
{
    if(v_id <= VehicleType::invalid_low || v_id >= VehicleType::invalid_high)
        return VehicleType::invalid_low;
    
    return v_id - VehicleManager::VehicleTypeFromId(v_id);
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

//public slots://///////////////////////////////////////////////////////////////

void VehicleManager::OnOperatorAddVehicle(const int vehicle_id)
{
    QMap<int, QString>::Iterator it = init_requests.find(vehicle_id);
    if(it != init_requests.end())
    {
        lcar_msgs::InitResponse res;
        res.vehicle_id = it.key(); // vehicle_id
        res.machine_name = it.value().toStdString(); // machine_name
        
        pub_init_response.publish(res);
        this->AddVehiclePrivate(res.vehicle_id); 
        init_requests.erase(it);
    }
}

void VehicleManager::OnOperatorDeleteVehicle(int v_id)
{          
    //widget_mutex.lock();
    
    int v_type = this->VehicleTypeFromId(v_id);
    
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    QMap<int, VehicleControl*> *v_db = &db[v_type];
    QMap<int, VehicleControl*>::Iterator it = v_db->find(v_id);
    if(it != v_db->end())
    {
        emit UIAdapter::Instance()->vehicleDeleted(v_id);
        //widget_deleted.wait(&widget_mutex);
        
        VehicleControl *vc = it.value();
        v_db->erase(it);
        delete vc;
    }
    else
        ROS_ERROR_STREAM("Tried to delete non existent " 
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: " << v_id);
    
    //widget_mutex.unlock();
}

void VehicleManager::OnScoutBuilding(int quad_id, int building)
{
    int v_type = this->VehicleTypeFromId(quad_id);
    Q_ASSERT(v_type == VehicleType::quad_rotor);
    
    QString path = ros::package::getPath("gcs").c_str();
    path.append("/buildings/" % coordinate_system % "/building" % QString::number(building) % ".txt");
    
    VehicleControl *vc = this->FindVehicle(v_type, quad_id);
    if(vc != nullptr)
    {
        UAVControl* uav = dynamic_cast<UAVControl*> (vc);
        if(coordinate_system == "global")
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

void VehicleManager::OnPauseMission(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
    {
        MissionMode m = vc->GetMissionMode();
        if(m == MissionMode::active)
            vc->PauseMission();
        else
            ROS_WARN_STREAM("Cannot pause non existent mission for "
                    << this->VehicleStringFromId(v_id).toStdString()
                    << "with id: " << v_id);
    }
    else
        ROS_ERROR_STREAM("Cannot pause mission: No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

void VehicleManager::OnResumeMission(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
    {
        MissionMode m = vc->GetMissionMode();
        if(m == MissionMode::paused)
            vc->ResumeMission();
        else
            ROS_WARN_STREAM("Cannot resume non existent mission for "
                    << this->VehicleStringFromId(v_id).toStdString()
                    << "with id: " << v_id);
    }
    else
        ROS_ERROR_STREAM("Cannot pause mission: No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

void VehicleManager::OnCancelMission(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        vc->StopMission();
    else
        ROS_ERROR_STREAM("Cannot cancel mission: No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

void VehicleManager::OnExecutePlay()
{
    //todo
}

void VehicleManager::OnPausePlay()
{
    //todo
}

void VehicleManager::OnResumePlay()
{
    //todo
}

void VehicleManager::OnCancelPlay()
{
    //todo
}

void VehicleManager::OnSetMachineLearningMode(bool on)
{
    QMap<int, VehicleControl*> *v_db = &db[VehicleType::quad_rotor];   
    for(auto it = v_db->begin(); it != v_db->end(); it++)
    {
        UAVControl *uav = dynamic_cast<UAVControl*>(it.value());
        uav->SetOnlineMode(on);
    }
}
 

void VehicleManager::OnPublishHitThreshold(double thresh)
{
    std_msgs::Float64 msg;
    msg.data = thresh;
    od_handlers.pub_hit_thresh.publish(msg);
}

void VehicleManager::OnPublishStepSize(int step)
{
    std_msgs::Int32 msg;
    msg.data = step;
    od_handlers.pub_step_size.publish(msg);
}

void VehicleManager::OnPublishPadding(int padding)
{
    std_msgs::Int32 msg;
    msg.data = padding;
    od_handlers.pub_padding.publish(msg);
}

void VehicleManager::OnPublishScaleFactor(double scale)
{
    std_msgs::Float64 msg;
    msg.data = scale;
    od_handlers.pub_scale_factor.publish(msg);
}

void VehicleManager::OnPublishMeanShift(bool on)
{
    std_msgs::Int32 msg;
    msg.data = on;
    od_handlers.pub_mean_shift.publish(msg);
}

void VehicleManager::OnSetCoordinateSystem(QString new_system)
{
    if(new_system == "global" || new_system == "local")
    {
        if(coordinate_system != new_system)
            coordinate_system = new_system;
        
        ROS_INFO_STREAM("Coordinate system set to " << new_system.toStdString());
    }
    else
        ROS_ERROR_STREAM("Cooridinate system invalid: " << new_system.toStdString());
}

void VehicleManager::OnLocalCoordinatesUpdated(const QVector<Point>& vector)
{
    world_map = vector;
    pub_world_map_updated.publish(std_msgs::Empty());
    
    ROS_INFO_STREAM("coordinates updated:");
    
    if(world_map.length() > 0)
    {
        ROS_INFO_STREAM(world_map.at(0).x << " " << world_map.at(0).y << " " << world_map.at(0).z);
    }
}

void VehicleManager::OnSetWaypoint(int v_id, double lat, double lng, double alt)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        vc->SetTarget(lat, lng, alt);
    else
        ROS_ERROR_STREAM("Cannot set waypoint: No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

void VehicleManager::OnArm(int v_id, bool value)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        vc->Arm(value);
    else
        ROS_ERROR_STREAM("Cannot arm vehicle: No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

void VehicleManager::OnSetMode(int v_id, QString mode)
{
    int v_type = this->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type == VehicleType::quad_rotor ||
             v_type == VehicleType::octo_rotor ||
             v_type == VehicleType::vtol);
    
    VehicleControl *vc = this->FindVehicle(v_type, v_id);
    if(vc != nullptr)
        vc->SetMode(mode.toStdString());
    else
        ROS_ERROR_STREAM("Cannot set Flight mode for UAV. No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

void VehicleManager::OnSetRTL(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        vc->SetRTL();
    else
        ROS_ERROR_STREAM("Cannot set Flight mode for UAV. No such" 
                << this->VehicleStringFromId(v_id).toStdString()
                << "with id: " << v_id);
}

int VehicleManager::IsArmed(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        return vc->IsArmed();
    
    ROS_ERROR_STREAM("Cannot get armed state for vehicle. No such" 
            << this->VehicleStringFromId(v_id).toStdString()
            << "with id: " << v_id);    
    
    return -1;
}

std::vector<lcar_msgs::QueryPtr> * VehicleManager::GetUAVDoorQueries(int quad_id)
{
    VehicleControl *vc = this->GetVehicle(quad_id);
    if(vc != nullptr)
    {
        UAVControl *quad = dynamic_cast<UAVControl*>(vc);
        if(quad)   
            return quad->GetDoorQueries();
    }
    
    ROS_ERROR_STREAM("Cannot get door queries for vehicle. No such" 
            << this->VehicleStringFromId(quad_id).toStdString()
            << "with id: " << quad_id);    
    
    return nullptr;
}

std::vector<lcar_msgs::AccessPointStampedPtr> * VehicleManager::GetUAVAccessPoints(int quad_id)
{
    VehicleControl *vc = this->GetVehicle(quad_id);
    if(vc != nullptr)
    {
        UAVControl *quad = dynamic_cast<UAVControl*>(vc);
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
        UAVControl *uav = dynamic_cast<UAVControl*> (vc);
        return uav->GetFlightState();
    }
    
    ROS_ERROR_STREAM("Cannot retrieve flight state UAV: no such " 
            << this->VehicleStringFromId(uav_id).toStdString() << " with id: " 
            << uav_id);
    
    return FlightState();
}

StatePtr VehicleManager::GetState(int v_id)
{
    StatePtr ptr;
    VehicleControl * vc = this->GetVehicle(v_id);
    if(vc != nullptr)
    {
        ptr = boost::make_shared<State>();
        ptr->armed = vc->IsArmed();
        ptr->battery = vc->GetBattery();
        ptr->mission_progress = vc->GetMissionProgress();
        ptr->mode = vc->GetMode();
    }
    
    return ptr;
}

int VehicleManager::GetDistanceToWP(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
       return vc->GetDistanceToWP();
    
    ROS_ERROR_STREAM("Cannot retrieve distance to waypoint for vehicle: no such " 
            << this->VehicleStringFromId(v_id).toStdString() << " with id: " 
            << v_id);
    
    return -1;
}

MissionMode VehicleManager::GetMissionMode(int v_id)
{
    VehicleControl *vc = this->GetVehicle(v_id);
    if(vc != nullptr)
        return vc->GetMissionMode();
    
    ROS_ERROR_STREAM("Cannot retrieve MissionMode for vehicle: no such " 
            << this->VehicleStringFromId(v_id).toStdString() << " with id: " 
            << v_id);
    
    return MissionMode::invalid;
}

//QMutex* VehicleManager::GetWidgetMutex()
//{
//    return &widget_mutex;
//}
//
//QWaitCondition* VehicleManager::GetWaitCondition()
//{
//    return &widget_deleted;
//}

//private://////////////////////////////////////////////////////////////////////

void VehicleManager::emitVehicleAdded(VehicleControl * vehicle)
{
    int v_type = VehicleTypeFromId(vehicle->id);
    switch(v_type)
    {
        case VehicleType::ugv:
            emit ugvAdded(vehicle->id);
            break;
        case VehicleType::quad_rotor:
            emit quadRotorAdded(vehicle->id);
            break;
        case VehicleType::octo_rotor:
            emit octoRotorAdded(vehicle->id);
            break;
        case VehicleType::vtol:
            emit vtolAdded(vehicle->id);
            break;
    }
}

bool VehicleManager::WorldMapRequested(lcar_msgs::WorldMap::Request& req, lcar_msgs::WorldMap::Response& res)
{
    res.data.reserve(world_map.size () * 3);
    for(const Point& p : world_map)
    {
        res.data.push_back(p.x);
        res.data.push_back(p.y);
        res.data.push_back(p.z);
    }
    return true;
}

bool VehicleManager::VehicleInitRequested(lcar_msgs::InitRequest::Request& req, 
                                            lcar_msgs::InitRequest::Response& res)
{
    // check that this vehicle hasn't already requested initialization
    for(auto it = init_requests.begin(); it != init_requests.end(); it++)
    {
        if(it.value().toStdString() == req.machine_name)
        {
            res.vehicle_id = VehicleType::invalid_low;
            res.ack = false;
            res.message = req.machine_name + " already requested initialization";
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
        
        emit UIAdapter::Instance()->NotifyOperator("Vehicle initialization requested");
        emit UIAdapter::Instance()->AddToInitWidget(QString(req.machine_name.c_str()), res.vehicle_id);

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

void VehicleManager::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    QPixmap img = image_conversions::rosImgToQpixmap(msg);
    emit UIAdapter::Instance()->NewImageFrame(img);
}

void VehicleManager::ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg)
{
    ROS_INFO_STREAM("received object detection paramater request");
    this->OnPublishHitThreshold(od_params.hit_thresh);
    this->OnPublishStepSize(od_params.step_size);
    this->OnPublishPadding(od_params.padding);
    this->OnPublishScaleFactor(od_params.scale_factor);
    this->OnPublishMeanShift(od_params.mean_shift);
}

void VehicleManager::TimedHeartBeatCheck(const ros::TimerEvent& e)
{
    int v_type = VehicleType::invalid_low + 1;
    for(; v_type < VehicleType::invalid_high; v_type += VEHICLE_TYPE_MAX)
    {
        QMap<int, VehicleControl*> *v_db = &db[v_type];   
        for(auto it = v_db->constBegin(); it != v_db->constEnd(); it++)
        {
            int v_id = it.key();
            VehicleControl* vc = it.value();
            emit UIAdapter::Instance()->SetVehicleWidgetEnabled(v_id, vc->RecievedHeartbeat());
        }
    }
}

void VehicleManager::AddVehiclePrivate(int v_id)
{
    int v_type = this->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleControl* vc;
    switch(v_type)
    {
        //todo uncomment this after UGVControl is added to the system
//        case        VehicleType::ugv: 
//                                    vc = new UGVControl(v_d);
//                                    break;
        case VehicleType::quad_rotor:
        case VehicleType::octo_rotor: 
        case       VehicleType::vtol: 
            vc = new UAVControl(v_id);
            break;
                                      
        default: vc = nullptr;
    }
    
    if(vc != nullptr)
    {
        db[v_type].insert(v_id, vc);
        ROS_INFO_STREAM("Added vehicle of type: "
                        << this->VehicleStringFromId(v_id).toStdString()
                        << " with id: " << v_id <<  " to database.");
        
        emit UIAdapter::Instance()->vehicleAdded(v_id);
        //emitVehicleAdded(vc);
    }
    else
        ROS_ERROR_STREAM("Tried to add vehicle of invalid type: "
                         << this->VehicleStringFromId(v_id).toStdString());
}

VehicleControl* VehicleManager::FindVehicle(int v_type, int v_id)
{   
    const QMap<int, VehicleControl*>& v_db = db.value(v_type);
    return v_db.value(v_id, nullptr);
}

VehicleControl* VehicleManager::GetVehicle(int v_id)
{
    int v_type = this->VehicleTypeFromId(v_id);
    
    return this->FindVehicle(v_type, v_id);
}

// SettingsWidget and QSettings related stuff
void VehicleManager::InitSettings()
{
    Settings settings;
    od_params = settings.GetObjectDetectionParameters();
    coordinate_system = settings.GetCoordinateSystem();
    world_map = settings.GetCoordinateSystemArray();
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

  void VehicleManager::runVehicles(const ros::TimerEvent& e)
  {
      for(const auto& outer : db)
      {
          for(const auto& vehicle : outer)
          {
              UAVControl* uav = dynamic_cast<UAVControl*>(vehicle);
              if(uav)
                  uav->Run();
          }
      }
  }

}