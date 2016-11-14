
/* 
 * File:   ui_adapter.h
 * Author: n8
 *
 * Created on November 9, 2016, 6:15 PM
 */

#ifndef UIADAPTER_H
#define UIADAPTER_H

#include <QObject>
#include <QPixmap>

#include <sensor_msgs/NavSatFix.h>

namespace rqt_gcs
{
    
class UIAdapter : public QObject
{
    Q_OBJECT
public:
    UIAdapter();
    virtual ~UIAdapter();
    
    static UIAdapter* Instance() ;
    
signals:

    // any ui -> backend
        //vehicle commands
        void Arm(int v_id, bool value);
        void SetWayPoint(int v_id, int lat, int lng, int alt=0);
        void SetMode(int v_id, QString mode);
        void SetRTL(int v_id);
        
        void ScoutBuilding(int quad_id, int num);
        void PauseMission(int v_id);
        void ResumeMission(int v_id);
        void CancelMission(int v_id);

        void ExecutePlay(int play);
        void PausePlay();
        void ResumePlay();
        void CancelPlay();
   
        //object detection settings for quad-rotors
        void SetMachineLearningMode(bool online);
        void PublishHitThreshold(double hit_thresh);
        void PublishStepSize(int step_size);
        void PublishPadding(int padding);
        void PublishScaleFactor(double scale);
        void PublishMeanShift(bool mean_shift);

        //database commands (and potentially voice commands)
        void AddVehicle(int v_id);
        void DeleteVehicle(int v_id);
    
        //a necessary evil for determing the way vehicles navigate. global(outdoors) or global(indoors)
        void SetCoordinateSystem(QString system);
    
    // backend -> any ui
    void NotifyOperator(QString message);

    // backend -> GCSMainWindow
    void NewImageFrame(QPixmap img);
    void AddVehicleWidget(int v_id);
    void DeleteVehicleWidget(int v_id);
    void SetVehicleWidgetEnabled(int v_id, bool enabled);

    // backend -> VehicleInitWidget
    void AddToInitWidget(QString machine_name, int v_id);
    
    
    // backend -> VOCE
    //nothing here yet
    
    
private:
    static UIAdapter *instance;
};

}
#endif /* UIADAPTER_H */

