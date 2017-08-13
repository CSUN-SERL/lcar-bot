/*
 * File:   query_widget.cpp
 * Author: n8
 *
 * Created on September 26, 2016, 7:20 PM
 */

#include <QPixmap>

#include <gcs/qt/query_widget.h>
#include <gcs/qt/trial_manager.h>
#include <gcs/qt/building.h>

namespace gcs
{

QueryWidget::QueryWidget(TrialManager * trial_manager, BuildingID building_id, Wall wall) :
_trial_manager(trial_manager),
_building_id(building_id),
_wall(wall)
{
    widget.setupUi(this);
    
    QObject::connect(widget.yesButton, &QPushButton::clicked,
                     this, &QueryWidget::accept);
    
    QObject::connect(widget.rejectButton, &QPushButton::clicked,
                     this, &QueryWidget::reject);
}

QueryWidget::~QueryWidget()
{
}

void QueryWidget::SetImage(const QPixmap& img)
{
    int w = widget.image_frame->width();
    int h = widget.image_frame->height();
    widget.image_frame->setPixmap(img.scaled(w, h, Qt::KeepAspectRatio));
}

void QueryWidget::accept()
{
    answered(Building::aYes);
}

void QueryWidget::reject()
{
    answered(Building::aNo);
}
        

void QueryWidget::answered(PromptAnswer answer)
{
    auto buildings = _trial_manager->getBuildings();
    if(buildings.isEmpty() ||
       _building_id < 0 || _building_id >= buildings.size())
    {
        emit queryAnswered(_building_id, Building::aNull);
        return;
    }
    
    std::shared_ptr<Building> b = buildings[_building_id];
    if(b == nullptr)
    {
        emit queryAnswered(_building_id, Building::aNull);
        return;
    }

    //only set found by if the answer is yes
    if(answer == Building::aYes)
        b->setFoundBy(_wall, Building::fVehicle);
    
    emit queryAnswered(_building_id, answer);
}

}