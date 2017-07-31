
/* 
 * File:   TrialManager.cpp
 * Author: n8
 * 
 * Created on July 29, 2017, 10:51 PM
 */

#include <QMap>
#include <QTimer>

#include <gcs/qt/trial_manager.h>
#include <gcs/util/trial_loader.h>
#include <gcs/util/building.h>
#include <gcs/util/debug.h>

#include <vehicle/vehicle_control.h>

#include <angles/angles.h>
#include <tf/tf.h>

#include <gcs/util/settings.h>

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

        double yaw_angle = angles::normalize_angle_positive(angles::from_degrees(0));
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
                     this, &TrialManager::checkEndTrial);
}

void TrialManager::reset()
{
    _cur_trial = -1;
    _cur_condition = TrialLoader::Null;
    _conditions_used = 0;
    _user_id = -1;
    
    emit sigReset();
}

void TrialManager::setCurrentVehicle(VehicleControl * vehicle)
{
    _vehicle = vehicle;
}

void TrialManager::setTrialStartCondition(TrialLoader::Condition c)
{    
    reset();
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
        // done
        emit finished();
    }
}

void TrialManager::startTrial()
{
    Q_ASSERT(_loader.isValid());
    Q_ASSERT(_vehicle);
    
    if(_loader.isValid() && _vehicle)
    {
        auto waypoints = getWaypointList(_loader);
        
        _vehicle->SetMission(waypoints);
        _vehicle->StartMission();
        
        _timer->start(1000);
    }
    else
    {
        qCDebug(lcar_bot) << "TrialManager::setTrial: CAN'T START EMPTY TRIAL";
    }
}

void TrialManager::endTrial()
{   
    _timer->stop();
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

void TrialManager::checkEndTrial()
{
    if(_vehicle->currentWaypoint() >= _loader.getWaypointInfoList().size())
    {
        exportTrialData();
        endTrial();
    }
}

}
