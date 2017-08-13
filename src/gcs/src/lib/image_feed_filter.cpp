
/* 
 * File:   ImageFeedFilter.cpp
 * Author: serl
 * 
 * Created on July 28, 2017, 10:15 AM
 */

#include <QKeyEvent>

#include <gcs/qt/image_feed_filter.h>
#include <gcs/qt/gcs_main_window.h>
#include <gcs/qt/trial_manager.h>
#include <gcs/qt/ui_adapter.h>

#include <gcs/qt/building.h>

#include <vehicle/uav_control.h>

namespace gcs
{

ImageFeedFilter::ImageFeedFilter(GCSMainWindow * main_window, QObject * parent) :
QObject(parent),
_main_window(main_window)
{
    QObject::connect(UIAdapter::Instance(), &UIAdapter::DeleteVehicle,
                    this, [this](int id)
                    {
                        if(_uav->id == id)
                            _uav == nullptr;
                    });
}


void ImageFeedFilter::setCurrentVehicle(VehicleControl* vehicle)
{
    _uav = dynamic_cast<UAVControl*>(vehicle);
    Q_ASSERT(_uav);
}

void ImageFeedFilter::setTrialManager(TrialManager * trial_manager)
{
    _trial_manager = trial_manager;
    
    QObject::connect(_trial_manager, &TrialManager::sigReset,
                    this, [this]()
    {
        _cur_building = nullptr;
    });
    
    QObject::connect(_trial_manager, &TrialManager::trialEnded,
                    this, [this]()
    {
        _cur_building = nullptr;
    });
    
    QObject::connect(_trial_manager, &TrialManager::currentBuildingChanged, 
                    this, [this]() 
    {
        _cur_building = _trial_manager->currentBuilding();
    });
}

bool ImageFeedFilter::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::KeyPress)
    {
        QKeyEvent * key_event = dynamic_cast<QKeyEvent*>(event);
        if(key_event->key() == Qt::Key_Space)
        {
            _space_down = true;
            _main_window->setImageFeedVisible(true);
            
            if(!_uav)
                return false;
            
            if(!_trial_manager->isValid())
                return false;
            
            if(_uav->MissionComplete())
                return false;
            
            auto waypoints = _trial_manager->getWaypointInfoList();
            int cur_wp = _uav->currentWaypoint();
            
            if(cur_wp >= waypoints.size())
                return false;
            
            auto wp = waypoints[cur_wp];
            
            /**
             * // todo don't get the wall from target yaw.
             * rather, calculate from actual vehicle position and angle
             */
            int wall = Building::targetYawToWall(wp->yaw);
            
            if(_cur_building)
            {
                _cur_building->spaceDown(wall);
                
                // todo same as above
                if(_uav->canQuery())
                {
                    if(_cur_building->wallHasDoor(wall))
                    {
                        // vehicle queries take precedence over operator
                        if(_cur_building->foundBy(wall) != Building::fVehicle)
                            _cur_building->setFoundBy(wall, Building::fOperator);
                        
                        _cur_building->setFoundByTentative(wall, Building::fOperator);
                    }
                }
            }
        }
        
    }
    else if(event->type() == QEvent::KeyRelease)
    {
        QKeyEvent * key_event = dynamic_cast<QKeyEvent*>(event);
        if(key_event->key() == Qt::Key_Space)
        {
            _space_down = false;
            _main_window->setImageFeedVisible(false);

            if(_cur_building)
            {
                _cur_building->spaceUp();
            }
        }
    }
    
    return false;
}

}