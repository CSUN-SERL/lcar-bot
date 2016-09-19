
/*
 * File:   VehicleListWidget.cpp
 * Author: n8
 *
 * Created on September 17, 2016, 7:20 PM
 */


#include <QStringBuilder>
#include "rqt_gcs/vehicle_list_widget.h"

namespace rqt_gcs
{

VehicleListWidget::VehicleListWidget(QWidget * parent):
MyQWidget(parent)
{
    widget.setupUi(this);
}

VehicleListWidget::~VehicleListWidget() 
{
}

void VehicleListWidget::SetNumber(int id)
{
    widget.VehicleSelectButton->setText(QString::number(id));
}

void VehicleListWidget::SetBattery(int battery)
{
    if (battery < 0)
        battery = 0;
    widget.VehicleBatteryLine->setText(QString::number(battery) % "%");
}

void VehicleListWidget::SetCondition(const QString& cond)
{
    widget.VehicleConditionLine->setText(cond);
}

void VehicleListWidget::SetName(const QString name)
{
    widget.VehicleNameLine->setText(name);
}

bool VehicleListWidget::ToggleButton(bool enable)
{
    widget.VehicleSelectButton->setEnabled(enable); 
    QString style_sheet = enable ?
          "background-color: rgb(64, 89, 140); color: rgb(240, 240, 240);" :
          "background-color: rgb(80, 90, 110); color: rgb(150, 150, 150);" ;

    widget.VehicleSelectButton->setStyleSheet(style_sheet);
}

bool VehicleListWidget::IsButtonEnabled()
{
    return widget.VehicleSelectButton->isEnabled();
}

const QPushButton* VehicleListWidget::Button()
{
    return widget.VehicleSelectButton;
}

}