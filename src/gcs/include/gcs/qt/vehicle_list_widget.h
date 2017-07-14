/* 
 * File:   VehicleListWidget.h
 * Author: n8
 *
 * Created on September 17, 2016, 7:20 PM
 */

#ifndef _VEHICLELISTWIDGET_H
#define _VEHICLELISTWIDGET_H

#include "ui_VehicleListWidget.h"
#include <gcs/qt/my_q_widget.h>

class QTimer;

namespace gcs
{
   
class VehicleControl;
    
class VehicleWidget : public MyQWidget
{
    Q_OBJECT
public:
    VehicleWidget(QWidget * parent = 0);
    virtual ~VehicleWidget();

    void SetNumber(int id);
    void SetBattery(int battery);
    void SetCondition(const QString& cond); 
    void SetName(const QString name);
    void SetVehicle(VehicleControl * vc);
    void SetUpdateTimer(QTimer * timer);
    
    void SetId(int id);
    int Id();
    
    void SetButtonEnabled(bool enable);
    bool IsButtonEnabled();
    const QPushButton* Button();
    
private slots:
    void timedUpdate();
    
private:
    Ui::VehicleListWidget _widget;
    VehicleControl * _vc;
    int v_id; //vehicle_id;
    
};

}
#endif /* _VEHICLELISTWIDGET_H */
