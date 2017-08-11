
/* 
 * File:   TrialManager.h
 * Author: n8
 *
 * Created on July 29, 2017, 10:51 PM
 */

#ifndef TRIALMANAGER_H
#define TRIALMANAGER_H

#include <memory>

#include <QObject>
#include <QMap>

#include <gcs/util/trial_loader.h>
#include <gcs/util/debug.h>

#include <sensor_msgs/Image.h>

class QTimer;

namespace gcs
{

class VehicleControl;
class UAVControl;
class TrialLoader;    
class Building;

class TrialManager : public QObject
{
    Q_OBJECT
public:
    
    TrialManager(QObject * parent);
    void reset();
    
    void setCurrentVehicle(VehicleControl * vehicle);
    
    void setTrialStartCondition(TrialLoader::Condition c);
    
    bool startTrial();
    void nextTrial();
    
    void setUserID(int user_id);
    void exportTrialData();
    
    std::shared_ptr<Building> currentBuilding()
    {
        if(_cur_b_id == -1)
            return nullptr;
        
        return _loader.getBuildings().value(_cur_b_id);
    }
    
    int currentTrial()
    {
        return _cur_trial;
    }
    
    TrialLoader::Condition currentCondition()
    {
        return _cur_condition;
    }
      
    const QMap<int, std::shared_ptr<Building> >& getBuildings()
    {
        return _loader.getBuildings();
    }
    
    const QList< std::shared_ptr<WaypointInfo> >& getWaypointInfoList()
    {
        return _loader.getWaypointInfoList();
    }
    
    bool isValid()
    {
        return _loader.isValid() &&
               _cur_condition != TrialLoader::Null &&
               _cur_trial > 0 && _cur_trial <= MAX_TRIALS && 
               _conditions_used <= MAX_CONDITIONS;
    }
    
    bool isRunning()
    {
        return _trial_running;
    }
    
     void endTrial();
    
signals:
    void trialChanged();
    void trialEnded();
    void sigReset();
    void currentBuildingChanged();
    
private:    
    void update();
    void fakeQuery();
    void setTrial(TrialLoader::Condition c, int trial);
    
    void connectToUIAdapter();
    void newImage();
    void deleteVehicle(int v_id);
    
    QImage queryImage(int q_type);
    
private:
    Q_DISABLE_COPY(TrialManager)
            
    TrialLoader _loader;
    //VehicleControl * _vehicle = nullptr;
    
    UAVControl* _uav = nullptr;
    
    QTimer * _timer;
    QTimer * _seconds_timer;
    
    int _user_id;
    
    bool _trial_running = false;
    
    TrialLoader::Condition _cur_condition;
    int _conditions_used = 0;
    int _cur_trial = -1;
    
    int MAX_TRIALS = 4;
    int MAX_CONDITIONS = 1;
    
//    QMap<int, std::shared_ptr<gcs::Building>> _waypoint_to_building;
    int _cur_b_id = -1;
    
    sensor_msgs::ImageConstPtr _cur_image;
    
    //_total_missed
};

}
#endif /* TRIALMANAGER_H */

