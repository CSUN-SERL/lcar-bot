
/*
 * File:   vehicle_init_widget.cpp
 * Author: n8
 *
 * Created on October 19, 2016, 4:04 PM
 */

#include "rqt_gcs/vehicle_init_widget.h"

VehicleInitWidget::VehicleInitWidget()
{
    this->setAttribute(Qt::WA_DeleteOnClose);
    
    widget.setupUi(this);
    
}

VehicleInitWidget::~VehicleInitWidget() 
{
}
