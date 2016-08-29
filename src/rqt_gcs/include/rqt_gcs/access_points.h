
#ifndef _ACCESSPOINTS_H
#define _ACCESSPOINTS_H

#include "ui_AccessPoints.h"
#include "rqt_gcs/simple_control.h"
#include "qt5/QtCore/qtimer.h"
#include <QSignalMapper>

namespace rqt_gcs
{
    
class AccessPoints : public QWidget
{
    Q_OBJECT
public:
    AccessPoints();
    virtual ~AccessPoints();
    void clearAccessPoints();
    void updateAccessPoints();
    static void saveUavAccessPoints(SimpleControl * uav, QString ap_type);
    void deleteAccessPoint(QWidget* w);
    void setUav(SimpleControl* uav);
    
private:
    Ui::AccessPoints widget_;
    QVector<QWidget*> ap_widgets;
    QSignalMapper* mapper;
    int num_access_points_last;
    QTimer* timer;
    
    SimpleControl* uav;
};

}
#endif /* _ACCESSPOINTS_H */
