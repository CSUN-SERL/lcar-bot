
#ifndef _ACCESSPOINTS_H
#define _ACCESSPOINTS_H

#include <QTimer>
#include <QSignalMapper>

#include "ui_AccessPoints.h"
#include "vehicle/uav_control.h"

namespace rqt_gcs
{
    
class AccessPoints : public QWidget
{
    Q_OBJECT
public:
    AccessPoints();
    virtual ~AccessPoints();
    void ClearAccessPoints();
    void UpdateAccessPoints();
    static void SaveUavAccessPoints(UAVControl * uav, QString ap_type);
    void SetUAV(UAVControl* uav);
    
public slots:
    void OnDeleteAccessPoint(QWidget* w);
    
private:
    Ui::AccessPoints widget;
    QTimer* timer;
    int num_access_points_last;
    
    UAVControl* uav;
};

}
#endif /* _ACCESSPOINTS_H */
