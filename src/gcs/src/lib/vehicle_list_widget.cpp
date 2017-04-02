
/*
 * File:   VehicleListWidget.cpp
 * Author: n8
 *
 * Created on September 17, 2016, 7:20 PM
 */


#include <QStringBuilder>
#include <gcs/qt/vehicle_list_widget.h>

namespace gcs
{

//public////////////////////////////////////////////////////////////////////////
VehicleWidget::VehicleWidget(QWidget * parent):
    MyQWidget(parent)
{
    widget.setupUi(this);
}

VehicleWidget::~VehicleWidget() 
{
}

void VehicleWidget::SetNumber(int id)
{
    widget.VehicleSelectButton->setText(QString::number(id));
}

void VehicleWidget::SetBattery(int battery)
{
    if (battery < 0)
        battery = 0;
    widget.VehicleBatteryLine->setText(QString::number(battery) % QChar('%'));
}

void VehicleWidget::SetCondition(const QString& cond)
{
    widget.VehicleConditionLine->setText(cond);
}

void VehicleWidget::SetName(const QString name)
{
    widget.VehicleNameLine->setText(name);
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

    widget.VehicleSelectButton->setStyleSheet(style_sheet);
}

bool VehicleWidget::IsButtonEnabled()
{
    return widget.VehicleSelectButton->isEnabled();
}

const QPushButton* VehicleWidget::Button()
{
    return widget.VehicleSelectButton;
}

}