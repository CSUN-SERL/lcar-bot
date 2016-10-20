
/* 
 * File:   vehicle_init_widget.h
 * Author: n8
 *
 * Created on October 19, 2016, 4:04 PM
 */

#ifndef _VEHICLEINITWIDGET_H
#define _VEHICLEINITWIDGET_H

#include "ui_VehicleInitWidget.h"

class VehicleInitWidget : public QWidget
{
    Q_OBJECT
public:
    VehicleInitWidget();
    virtual ~VehicleInitWidget();
    
signals:
    void AddVehicle(QString machine_name);
    
private:
    Ui::VehicleInitWidget widget;
};

#endif /* _VEHICLEINITWIDGET_H */
