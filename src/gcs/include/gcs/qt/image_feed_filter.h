
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

    void setCurrentVehicle(VehicleControl* vehicle);
    void setTrialManager(TrialManager * trial_manager);

    bool spaceDown()
    {
        return _space_down;
    }

protected:
    bool eventFilter(QObject *obj, QEvent *event);
    
private:
    GCSMainWindow * _main_window;
    std::shared_ptr<Building> _cur_building;
    UAVControl* _uav = nullptr;
    TrialManager * _trial_manager;

    bool _space_down = false;
    int  _space_count;
};

}
#endif /* IMAGEFEEDFILTER_H */

