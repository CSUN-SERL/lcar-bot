
/*
 * File:   VehicleListWidget.cpp
 * Author: n8
 *
 * Created on September 17, 2016, 7:20 PM
 */

#include <QStringBuilder>
#include <QTimer>
#include <QMenu>

#include <gcs/qt/gcs_main_window.h>
#include <gcs/qt/trial_manager.h>
#include <gcs/qt/vehicle_list_widget.h>
#include <gcs/qt/ui_adapter.h>
#include <vehicle/vehicle_control.h>


namespace gcs
{

//public////////////////////////////////////////////////////////////////////////
VehicleWidget::VehicleWidget(GCSMainWindow* main_window, TrialManager * trial_manager, QWidget * parent):
    MyQWidget(parent),
    _main_window(main_window),
    _trial_manager(trial_manager)
{
    setContextMenuPolicy(Qt::CustomContextMenu);
    _widget.setupUi(this);
    
    QObject::connect(this, &VehicleWidget::customContextMenuRequested,
                     this, &VehicleWidget::contextMenuRequested);
}

VehicleWidget::~VehicleWidget() 
{
}

void VehicleWidget::SetNumber(int id)
{
    _widget.VehicleSelectButton->setText(QString::number(id));
}

void VehicleWidget::SetBattery(float battery)
{
    if (battery < 0)
        battery = 0;
    _widget.VehicleBatteryLine->setText(QString::number(battery) % QChar('%'));
}

void VehicleWidget::SetCondition(const QString& cond)
{
    _widget.VehicleConditionLine->setText(cond);
}

void VehicleWidget::SetName(const QString name)
{
    _widget.VehicleNameLine->setText(name);
}

void VehicleWidget::SetVehicle(VehicleControl * vc)
{
    _vc = vc;
}

void VehicleWidget::SetUpdateTimer(QTimer * timer)
{
    QObject::connect(timer, &QTimer::timeout,
                    this, &VehicleWidget::timedUpdate);
}

void VehicleWidget::SetId(int id)
{
    this->v_id = id;
}

int VehicleWidget::Id()
{
    return v_id;
}

void VehicleWidget::SetButtonEnabled(bool enable)
{
//    widget.VehicleSelectButton->setEnabled(enable); 
    QString style_sheet = enable ?
          "background-color: rgb(64, 89, 140); color: rgb(240, 240, 240);" :
          "background-color: rgb(80, 90, 110); color: rgb(150, 150, 150);" ;

    _widget.VehicleSelectButton->setStyleSheet(style_sheet);
}

bool VehicleWidget::IsButtonEnabled()
{
    return _widget.VehicleSelectButton->isEnabled();
}

const QPushButton* VehicleWidget::Button()
{
    return _widget.VehicleSelectButton;
}

void VehicleWidget::timedUpdate()
{
    if(_vc)
    {
        SetBattery(_vc->GetBattery() * 100);
        SetCondition(_vc->GetMode().c_str());
    }
}

void VehicleWidget::contextMenuRequested(QPoint pos)
{
    if(_trial_manager->isRunning())
        return;
    
    if(_menu == nullptr)
        createMenu();
    
    _menu->popup(mapToGlobal(pos));
}

void VehicleWidget::createMenu()
{
    if(_menu != nullptr)
        return;
    
    _menu = new QMenu(this);
    
    QAction * action = _menu->addAction("Delete Vehicle");
    
    QObject::connect(action, &QAction::triggered,
                    this, &VehicleWidget::deleteActionTriggererd);
}

void VehicleWidget::deleteActionTriggererd()
{
    _vc = nullptr;
    _main_window->deleteVehicle(v_id);
}

}