
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
#include <gcs/util/building.h>
#include <gcs/util/debug.h>

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
_timer(new QTimer(this))
{
    QObject::connect(_timer, &QTimer::timeout,
                     this, &TrialManager::checkCurrentBuildingChange);
    

    QObject::connect(UIAdapter::Instance(), &UIAdapter::DeleteVehicle,
                    this, [=](int v_id)
    {
        if(_uav->id == v_id)
            _uav = nullptr;
    });
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

void TrialManager::checkCurrentBuildingChange()
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

}
