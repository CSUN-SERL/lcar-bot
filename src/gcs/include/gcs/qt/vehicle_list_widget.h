/* 
 * File:   VehicleListWidget.h
 * Author: n8
 *
 * Created on September 17, 2016, 7:20 PM
 */

#ifndef _VEHICLELISTWIDGET_H
#define _VEHICLELISTWIDGET_H

#include <QPointer>

#include "ui_VehicleListWidget.h"
#include <gcs/qt/my_q_widget.h>

class QMenu;
class QTimer;

namespace gcs
{
   
class VehicleControl;
class TrialManager;
class GCSMainWindow;
    
class VehicleWidget : public MyQWidget
{
    Q_OBJECT
public:
    VehicleWidget(GCSMainWindow* main_window, TrialManager * trial_manager, QWidget * parent = 0);
    virtual ~VehicleWidget();

    void SetNumber(int id);
    void SetBattery(float battery);
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
    void contextMenuRequested(QPoint pos);
    void deleteActionTriggererd();
    
private:
     void createMenu();
    
private:
    Ui::VehicleListWidget _widget;
    VehicleControl * _vc;
    int v_id; //vehicle_id;
    
    QPointer<QMenu> _menu;
    
    GCSMainWindow* _main_window;
    TrialManager * _trial_manager;
    
};

}
#endif /* _VEHICLELISTWIDGET_H */