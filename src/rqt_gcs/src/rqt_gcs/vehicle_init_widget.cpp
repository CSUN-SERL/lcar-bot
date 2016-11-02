
/*
 * File:   vehicle_init_widget.cpp
 * Author: n8
 *
 * Created on October 19, 2016, 4:04 PM
 */

#include "rqt_gcs/vehicle_init_widget.h"
#include "rqt_gcs/vehicle_manager.h"
#include "ui_VehicleInitWidget.h"

#include "util/debug.h"

namespace rqt_gcs
{

//public:///////////////////////////////////////////////////////////////////////
    
VehicleInitWidget::VehicleInitWidget(VehicleManager *vm):
vm(vm)
{
    this->setAttribute(Qt::WA_DeleteOnClose);
    
    widget.setupUi(this);
    widget.view->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    connect(widget.btn_add, &QPushButton::clicked,
            this, &VehicleInitWidget::OnAddVehicleBtnClicked);
    
    connect(widget.btn_close, &QPushButton::clicked,
            this, [=](){ this->close(); } );
            
    connect(this, &VehicleInitWidget::AddVehicleToDb,
            vm, &VehicleManager::OnOperatorInitRequested);
    
    connect(vm, &VehicleManager::RemoveInitRequest,
            this, &VehicleInitWidget::OnRemoveInitRequest);
    
    connect(vm, &VehicleManager::AddToInitWidget, 
            this, &VehicleInitWidget::OnAddInitRequest);

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
    while(row < rows && widget.view->item(row, 2)->data(0).toInt() != vehicle_id)
        row++;
    
    widget.view->removeRow(row);
}

void VehicleInitWidget::OnAddInitRequest(QString machine_name, int vehicle_id)
{
    qCDebug(lcar_bot) << "vehicle_id: " << vehicle_id;
    QString v_type = vm->VehicleStringFromMachineName(machine_name);

    int row = widget.view->rowCount();
    widget.view->insertRow(row); // rowCount goes up by 1

    widget.view->setItem(row, 0, new QTableWidgetItem(machine_name));
    widget.view->setItem(row, 1, new QTableWidgetItem(v_type));
    widget.view->setItem(row, 2, new QTableWidgetItem(QString::number(vehicle_id)));
}

//private://////////////////////////////////////////////////////////////////////

void VehicleInitWidget::DisplayVehicleInitRequests()
{
    const QMap<int, QString> requests = vm->GetInitRequests();
    QMap<int, QString>::ConstIterator it = requests.begin();
    for(; it != requests.end(); it++)
        this->OnAddInitRequest(it.value(),it.key());
}



}