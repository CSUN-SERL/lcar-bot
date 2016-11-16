
/* 
 * File:   AccessPointWidget.h
 * Author: n8
 *
 * Created on October 6, 2016, 2:54 PM
 */

#ifndef _ACCESSPOINTWIDGET_H
#define _ACCESSPOINTWIDGET_H

#include <QPixmap>

#include "gcs/my_q_widget.h"
#include "ui_AccessPointWidget.h"

namespace gcs
{

class AccessPointWidget : public MyQWidget {
    Q_OBJECT
public:
    AccessPointWidget(QWidget *parent = 0);
    virtual ~AccessPointWidget();
    
    void SetName(QString name);
    void SetLatitude(double lat);
    void SetLongitude(double lng);
    void SetHeading(double heading);
    void SetAltitude(double altitude);
    void SetCaptureTime(double time);
    void SetImage(QPixmap& img);
    const QPushButton* Button();
    
    
private:
    Ui::AccessPointWidget widget;
};

}

#endif /* _ACCESSPOINTWIDGET_H */