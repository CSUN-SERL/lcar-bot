
/* 
 * File:   TrialManager.cpp
 * Author: n8
 * 
 * Created on July 29, 2017, 10:51 PM
 */

#include <QMap>
#include <QTimer>

#include <gcs/qt/ui_adapter.h>
#include <gcs/qt/trial_manager.h>
#include <gcs/util/trial_loader.h>
#include <gcs/qt/building.h>
#include <gcs/util/debug.h>
#include <gcs/util/image_conversions.h>

//#include <vehicle/vehicle_control.h>
#include <vehicle/uav_control.h>

#include <angles/angles.h>
#include <tf/tf.h>

#include <gcs/util/settings.h>
#include <QtCore/qdatetime.h>

namespace gcs
{

std::vector<geometry_msgs::Pose> getWaypointList(const TrialLoader& loader)
{
    auto wp_info_list = loader.getWaypointInfoList();
    std::vector<geometry_msgs::Pose> waypoints;
    
    for(const auto& wp: wp_info_list)
    {
        geometry_msgs::Pose pose;
        pose.position.x = wp->x;
        pose.position.y = wp->y;
        pose.position.z = wp->z;

        double yaw_angle = angles::normalize_angle_positive(angles::from_degrees(wp->yaw));
        quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle), pose.orientation);
        
        waypoints.push_back(pose);
    }
    
    return waypoints;
}
    
TrialManager::TrialManager(QObject * parent) :
QObject(parent),
_timer(new QTimer(this)),
_seconds_timer(new QTimer(this))
{
    QObject::connect(_timer, &QTimer::timeout,
                     this, &TrialManager::update);
    
    QObject::connect(_timer, &QTimer::timeout,
                    this, &TrialManager::fakeQuery);

    connectToUIAdapter();
}

void TrialManager::reset()
{
    _cur_trial = -1;
    _cur_condition = TrialLoader::Null;
    _conditions_used = 0;
    _user_id = -1;
    _trial_running = false;
    _cur_b_id;
    
    emit sigReset();
}

void TrialManager::setCurrentVehicle(VehicleControl * vehicle)
{
    //_vehicle = vehicle;
    _uav = dynamic_cast<UAVControl*>(vehicle);
    Q_ASSERT(_uav);
}

void TrialManager::setTrialStartCondition(TrialLoader::Condition c)
{    
    blockSignals(true);
    reset();
    blockSignals(false);
    setTrial(c, 1);
}

void TrialManager::setTrial(TrialLoader::Condition c, int trial)
{   
    if(_cur_condition != c || _cur_trial != trial)
    {
        _loader.load(c, trial);
        
        auto buildings = _loader.getBuildings();

        for(auto it = buildings.constBegin(); it != buildings.constEnd(); ++it)
        {
            std::shared_ptr<Building> b = *it;
            QObject::connect(b.get(), &Building::foundByChanged,
                            this, &TrialManager::accessPointFound);
        }
        
        _cur_condition = c;
        _cur_trial = trial;
        
        emit trialChanged();
    }
}


void TrialManager::nextTrial()
{
    _loader.reset();
    if(_cur_trial < MAX_TRIALS)
    {
        setTrial(_cur_condition, _cur_trial + 1);
    }
    else if(_conditions_used < MAX_CONDITIONS)
    {
        TrialLoader::Condition c = _cur_condition == TrialLoader::Predictable ?
                TrialLoader::UnPredictable :
                TrialLoader::Predictable;
        
        _conditions_used++;
        
        setTrial(c, 1);
    }
    else
    {
        reset();
    }
}

bool TrialManager::startTrial()
{    
    if(isValid() && _uav)
    {
        auto waypoints = getWaypointList(_loader);
        
        //qCDebug(lcar_bot) << "WP LIST SIZE:" << waypoints.size();
        
        _uav->SetMission(waypoints);
        _uav->StartMission();
        _uav->EnableOffboard();
           
        _timer->start(33);
        _seconds_timer->start(3000);
          
        _trial_running = true;
    }
    else
    {
        qCDebug(lcar_bot) << "TriaflManager::setTrial: CAN'T START EMPTY TRIAL";
        _trial_running = false;
    }
    
    return _trial_running;
}

void TrialManager::endTrial()  
{   
    _timer->stop();
    _seconds_timer->stop();
    _trial_running = false;
    emit trialEnded();
//    _vehicle->StopMission();
//    _vehicle->SetMission({});
}

void TrialManager::setUserID(int user_id)
{
    _user_id = user_id;
}

void TrialManager::exportTrialData()
{
    //todo fuuuuuuuck
    Settings s;
}

void TrialManager::update()
{   
    auto waypoints = _loader.getWaypointInfoList();
    auto buildings = _loader.getBuildings();
    
    int cur_wp = _uav->currentWaypoint();
    
    if(cur_wp >= _loader.getWaypointInfoList().size())
    {
        exportTrialData();
        endTrial();
        return;
    }
    
    auto wp = waypoints.at(cur_wp);
    
    if (_cur_b_id != wp->building_id && wp->isBuildingWP())
    {
        _cur_b_id = wp->building_id;
        emit currentBuildingChanged();
    }
}

void TrialManager::fakeQuery()
{
    auto building = _loader.getBuildings().value(_cur_b_id, nullptr);
    auto wp = _loader.getWaypointInfoList().value(_uav->currentWaypoint(), nullptr);
    
    if(!building)
        return;
    
    if(!wp)
        return;
    
    if(!_uav->canQuery())
        return;

    int wall = Building::targetYawToWall(wp->yaw);
    
    if(wall == -1)
        return;

    auto door_queries = building->doorPrompts();
    auto window_queries = building->windowPrompts();
    auto doors = building->doors();

    
    int count = building->queryCountForWall(wall, Building::qDoor);
    Q_ASSERT(count != -1);
    bool query = count < building->maxQueriesPerWall(Building::qDoor);

    if(query && door_queries[wall] == 1)
    {
        if(_cur_image == nullptr)
        {
            sensor_msgs::Image image;
            QImage p = queryImage(Building::qDoor);
            image_conversions::qImgToRosImg(QImage(p), image);
            _uav->fakeQuery(image, building->getID());
        }
        else
        {
            _uav->fakeQuery(_cur_image, building->getID());
        }
        
        building->wallQueried(wall, Building::qDoor);
    }
    
    count = building->queryCountForWall(wall, Building::qWindow);
    Q_ASSERT(count != -1);
    query = count < building->maxQueriesPerWall(Building::qWindow);
     
    if(query && window_queries[wall] == 1)
    {
        if(_cur_image == nullptr)
        {
            sensor_msgs::Image image;
            QImage p = queryImage(Building::qWindow);
            image_conversions::qImgToRosImg(QImage(p), image);
            _uav->fakeQuery(image, building->getID());
        }
        else
        {
            _uav->fakeQuery(_cur_image, building->getID());
        }
        
        building->wallQueried(wall, Building::qWindow);
    }
}

void TrialManager::connectToUIAdapter()
{
    UIAdapter * uia = UIAdapter::Instance();
    
    QObject::connect(uia, &UIAdapter::NewImageFrame,
                    this, &TrialManager::newImage);
    
    QObject::connect(uia, &UIAdapter::DeleteVehicle,
                    this, &TrialManager::deleteVehicle);
}

QImage TrialManager::queryImage(int q_type)
{
    switch (q_type)
    {
        case Building::qDoor:
            return QImage(":/Resources/door_purple.jpg");
        case Building::qWindow:
            return QImage(":/Resources/window_purple.jpg");
        case Building::qNull:
        default:
            break;
    }
    
    return QImage();
}

void TrialManager::newImage()
{
    _cur_image = UIAdapter::Instance()->_cur_image;
}

void TrialManager::deleteVehicle(int v_id)
{
    if(_uav->id == v_id)
        _uav = nullptr;
}

}
