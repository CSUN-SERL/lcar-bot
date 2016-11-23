
/* 
 * File:   vehicle_init_widget.h
 * Author: n8
 *
 * Created on October 19, 2016, 4:04 PM
 */

#ifndef _VEHICLEINITWIDGET_H
#define _VEHICLEINITWIDGET_H

#include "ui_VehicleInitWidget.h"
#include "qt/vehicle_manager.h"

namespace gcs
{

class VehicleInitWidget : public QWidget
{
    Q_OBJECT
public:
    VehicleInitWidget(VehicleManager *vm);
    virtual ~VehicleInitWidget();

public slots:
    void OnAddVehicleBtnClicked();
    void OnAddInitRequest(QString machine_name, int vehicle_id);
    
private:
    void DisplayInitRequests();
    
    Ui::VehicleInitWidget widget;\
    VehicleManager *vm;
};

}
#endif /* _VEHICLEINITWIDGET_H */
