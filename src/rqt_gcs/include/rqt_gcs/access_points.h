
#ifndef _ACCESSPOINTS_H
#define _ACCESSPOINTS_H

#include "ui_AccessPoints.h"
#include "vehicle/uav_control.h"
#include <QTimer>
#include <QSignalMapper>

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
    Ui::AccessPoints widget_;
    QVector<QWidget*> ap_widgets;
    QSignalMapper* mapper;
    int num_access_points_last;
    QTimer* timer;
    
    UAVControl* uav;
};

}
#endif /* _ACCESSPOINTS_H */
