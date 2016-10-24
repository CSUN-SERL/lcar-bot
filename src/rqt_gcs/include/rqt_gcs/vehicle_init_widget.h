
/* 
 * File:   vehicle_init_widget.h
 * Author: n8
 *
 * Created on October 19, 2016, 4:04 PM
 */

#ifndef _VEHICLEINITWIDGET_H
#define _VEHICLEINITWIDGET_H

#include "ui_VehicleInitWidget.h"
#include "rqt_gcs/vehicle_manager.h"

namespace rqt_gcs
{

class VehicleInitWidget : public QWidget
{
    Q_OBJECT
public:
    VehicleInitWidget(VehicleManager *vm);
    virtual ~VehicleInitWidget();
    
signals:
    void AddVehicleToDb(int machine_name);

public slots:
    void OnAddVehicleBtnClicked();
    void OnRemoveInitRequest(int vehicle_id);
    
private:
    void DisplayVehicleInitRequests();
    
    Ui::VehicleInitWidget widget;
    VehicleManager *vm;
};

}
#endif /* _VEHICLEINITWIDGET_H */
