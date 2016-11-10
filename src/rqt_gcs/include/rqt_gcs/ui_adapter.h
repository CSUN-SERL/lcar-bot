
/* 
 * File:   ui_adapter.h
 * Author: n8
 *
 * Created on November 9, 2016, 6:15 PM
 */

#ifndef UIADAPTER_H
#define UIADAPTER_H

#include <QObject>

namespace rqt_gcs
{

class UIAdapter : public QObject
{
    Q_OBJECT
public:
    UIAdapter();
    virtual ~UIAdapter();
    
signals:

    // backend -> ui
    void NotifyOperator(QString message);
    
    // ui -> backend
    void Arm(int v_id, bool value);
    void SetWayPoint(int v_id, sensor_msgs::NavSatFix waypoint);
    void SetMode(int v_id, QString mode);
    
    void ScoutBuilding(int quad_id, QString building);
    void PauseMission(int v_id);
    void ResumeMission(int v_id);
    void CancelMission(int v_id);
    
    void ExecutePlay(QString play);
    void PausePlay();
    void ResumePlay();
    void CancelPlay();
    
    void SetOnlineMode(bool on);
    
    
    // GUI specific
    void NewImageFrame(QPixmap img);
    void AddToInitWidget(QString machine_name, int vehicle_id);
    void AddVehicleWidget(int v_id);
    void DeleteVehicleWidget(int v_id);
    
    // VOICE UI specific
    
    
private:

};

}
#endif /* UIADAPTER_H */

