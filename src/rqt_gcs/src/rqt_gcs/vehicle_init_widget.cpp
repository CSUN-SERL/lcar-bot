
/*
 * File:   vehicle_init_widget.cpp
 * Author: n8
 *
 * Created on October 19, 2016, 4:04 PM
 */

#include "rqt_gcs/vehicle_init_widget.h"
#include "rqt_gcs/vehicle_manager.h"
#include "ui_VehicleInitWidget.h"

namespace rqt_gcs
{

//public:///////////////////////////////////////////////////////////////////////
    
VehicleInitWidget::VehicleInitWidget(VehicleManager *vm):
vm(vm)
{
    this->setAttribute(Qt::WA_DeleteOnClose);
    
    widget.setupUi(this);
    widget.view->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    connect(this, &VehicleInitWidget::AddVehicleToDb,
            vm, &VehicleManager::OnOperatorInitRequested);
    
    connect(widget.btn_close, &QPushButton::clicked,
            this, [=](){ this->close(); } );
            
            connect(vm, &VehicleManager::RemoveInitRequest,
                    this, &VehicleInitWidget::OnRemoveInitRequest);

    this->DisplayVehicleInitRequests();
}

VehicleInitWidget::~VehicleInitWidget() 
{
    vm = nullptr;
}

//public slots://///////////////////////////////////////////////////////////////

void VehicleInitWidget::OnAddVehicleBtnClicked()
{
    QList<QTableWidgetItem*> items = widget.view->selectedItems();
    int size = items.size();
    for(int i = 0; i < size; i++)
    {
        if(items[i]->column() == 2)
            emit AddVehicleToDb(items[i]->text().toInt());
    }
}

void VehicleInitWidget::OnRemoveInitRequest(int vehicle_id)
{
    int rows = widget.view->rowCount();
    int row = 0;
    while(row < rows && widget.view->item(row, 2))
        row++;
    
    widget.view->removeRow(row);
}

//private://////////////////////////////////////////////////////////////////////

void VehicleInitWidget::DisplayVehicleInitRequests()
{
    QMap<int, QString> requests = vm->GetInitRequests();
    QMap<int, QString>::Iterator it = requests.begin();
    for(; it != requests.end(); it++)
    {
        int id = it.key();
        QString machine_name = it.value();
        QString v_type = vm->VehicleTypeFromName(machine_name);
        
        int row = widget.view->rowCount();
        widget.view->insertRow(row);

        widget.view->setItem(row, 0, new QTableWidgetItem(machine_name));
        widget.view->setItem(row, 1, new QTableWidgetItem(v_type));
        widget.view->setItem(row, 2, new QTableWidgetItem(id));
    }
}



}