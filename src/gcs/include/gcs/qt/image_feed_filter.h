
/* 
 * File:   ImageFeedFilter.h
 * Author: serl
 *
 * Created on July 28, 2017, 10:15 AM
 */

#ifndef IMAGEFEEDFILTER_H
#define IMAGEFEEDFILTER_H

#include <memory>

#include <QObject>

namespace gcs
{

class GCSMainWindow;
class Building;
class UAVControl;
class VehicleControl;
class TrialManager;
    
class ImageFeedFilter : public QObject
{
    Q_OBJECT
public:
    ImageFeedFilter(GCSMainWindow * main_window, QObject * parent = nullptr);
    
    void setCurrentBuilding(const std::shared_ptr<Building>& building);
    void setCurrentVehicle(VehicleControl* vehicle);
    void setTrialManager(TrialManager * trial_manager);
    
    int targetYawToWall(int yaw)
    {
        switch(yaw)
        {
            case 0: 
                return 2;
            case 90:
                return 3;
            case 180:
                return 0;
            case 270:
                return 1;
            default:
                break;
        }
        return -1;
    }
protected:
    bool eventFilter(QObject *obj, QEvent *event);
    
private:
    GCSMainWindow * _main_window;
    std::shared_ptr<Building> _cur_building;
    UAVControl* _uav = nullptr;
    TrialManager * _trial_manager;
};

}
#endif /* IMAGEFEEDFILTER_H */

