
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

#include <gcs/qt/building.h>

#include <gcs/util/trial_loader.h>
#include <gcs/util/debug.h>

#include <sensor_msgs/Image.h>

class QTimer;

namespace gcs
{

class DistractionContainerWidget;
class ImageFeedFilter;
class VehicleControl;
class UAVControl;
class TrialLoader;    
class Building;

class TrialManager : public QObject
{
    Q_OBJECT
public:
    
    TrialManager(ImageFeedFilter * filter, QObject * parent);
    void reset(bool overwrite_user_id = true);

    void setDistractionWidget(DistractionContainerWidget * widget);
    void setCurrentVehicle(VehicleControl * vehicle);
    
    void setTrialStartCondition(TrialLoader::Condition c);
    
    bool startTrial();
    void nextTrial();
    
    void setUserID(const QString& user_id);
    void exportTrialData();
    
    std::shared_ptr<Building> currentBuilding()
    {
        if( _cur_b_id == -1)
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
    
    const QList< std::shared_ptr<WaypointInfo> >& getWaypoints()
    {
        return _loader.getWaypoints();
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
    void accessPointFound(BuildingID id, FoundBy found_by);

private:    
    void update();
    void fakeQuery();
    void setTrial(TrialLoader::Condition c, int trial);
    
    bool wallInRange(Wall target);
    void connectToUIAdapter();
    void newImage();
    void deleteVehicle(int v_id);
    
    QImage queryImage(int q_type);
    
private:
    Q_DISABLE_COPY(TrialManager)
            
    TrialLoader _loader;

    UAVControl* _uav = nullptr;
    ImageFeedFilter * _image_feed_filter;
    DistractionContainerWidget * _distraction_widget;

    QTimer * _timer;
    
    QString _user_id;
    
    bool _trial_running = false;
    
    TrialLoader::Condition _cur_condition;
    int _conditions_used = 0;
    int _cur_trial = -1;
    
    int MAX_TRIALS = 2;
    int MAX_CONDITIONS = 1;
    
    int _cur_b_id = -1;
    
    sensor_msgs::ImageConstPtr _cur_image;
    
    //_total_missed
};

}
#endif /* TRIALMANAGER_H */

