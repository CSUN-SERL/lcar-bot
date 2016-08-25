
#ifndef _ACCESSPOINTS_H
#define _ACCESSPOINTS_H

#include "qt5/QtCore/qtimer.h"
#include <QStringBuilder>
#include <QSignalMapper>
#include <QDir>

#include "ui_AccessPoints.h"
#include "ui_AccessPointStats.h"
#include "ui_ImageView.h"

#include "rqt_gcs/simple_control.h"
#include "rqt_gcs/image_handler.h"


namespace rqt_gcs{

class AccessPoints : public QWidget
{
    Q_OBJECT
public:
    AccessPoints();
    virtual ~AccessPoints();
    void updateAccessPoints();
    void clearAccessPoints();
    static void saveUavAccessPoints(SimpleControl * uav, QString& ap_type);
    void deleteAccessPoint(QWidget* w);
    void setUav(SimpleControl* uav);
    
private:
    Ui::AccessPoints widget_;
    QVector<QWidget*> ap_widgets;
    QSignalMapper* mapper;
    int num_access_points_last;
    QTimer* timer;
    SimpleControl* uav;
   // ImageHandler img_handler;

    
    bool saveImage(QString& path, QString& file, QImage* image);

};

}
#endif /* _ACCESSPOINTS_H */
