
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

#include <gcs/util/trial_loader.h>

class QTimer;

namespace gcs
{

class VehicleControl;
class TrialLoader;    
class Building;

class TrialManager : public QObject
{
    Q_OBJECT
public:
    TrialManager(QObject * parent);
    
    void setCurrentVehicle(VehicleControl * vehicle);
    
    void setTrial(TrialLoader::Condition c, int trial);
    
    void startTrial();
    void endTrial();
    
    void setUserID(int user_id);
    void exportTrialData();
    
private:
    void checkEndTrial();
    
private:
    Q_DISABLE_COPY(TrialManager)
            
    TrialLoader _loader;
    VehicleControl * _vehicle = nullptr;
    
    QTimer * _timer;
    
    QList< std::shared_ptr<Building> > _buildings;
    QList< std::shared_ptr<WaypointInfo> > _waypoints;
    
    int _user_id;
};

}
#endif /* TRIALMANAGER_H */

