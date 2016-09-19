/* 
 * File:   VehicleListWidget.h
 * Author: n8
 *
 * Created on September 17, 2016, 7:20 PM
 */

#ifndef _VEHICLELISTWIDGET_H
#define _VEHICLELISTWIDGET_H

#include "rqt_gcs/my_q_widget.h"
#include "ui_VehicleListWidget.h"

namespace rqt_gcs
{

class VehicleListWidget : public MyQWidget
{
    Q_OBJECT
public:
    VehicleListWidget(QWidget * parent = 0);
    virtual ~VehicleListWidget();
    
    void SetNumber(int id);
    void SetBattery(int battery);
    void SetCondition(const QString& cond); 
    void SetName(const QString name);
    
    bool ToggleButton(bool enable);
    bool IsButtonEnabled();
    const QPushButton* Button();
    
private:
    Ui::VehicleListWidget widget;
};

}
#endif /* _VEHICLELISTWIDGET_H */
