
/*
 * File:   TrialIdWidget.cpp
 * Author: n8
 *
 * Created on July 30, 2017, 12:14 AM
 */

#include <QIntValidator>

#include <gcs/qt/user_id_widget.h>
#include <gcs/qt/trial_manager.h>

namespace gcs
{

UserIdWidget::UserIdWidget(TrialManager* trial_manager):
_trial_manager(trial_manager)
{
    widget.setupUi(this);
    auto validator = new QIntValidator(this);
    validator->setBottom(0);
    validator->setTop(10000);
    widget.id_line->setValidator(validator);
    
    QObject::connect(widget.btn_ok, &QPushButton::clicked,
                    this, &UserIdWidget::okClicked);
    
    QObject::connect(widget.btn_cancel, &QPushButton::clicked,
                    this, &UserIdWidget::cancelClicked);
}

UserIdWidget::~UserIdWidget() 
{
}

void UserIdWidget::okClicked()
{
    QString s = widget.id_line->text();
    if(s.isEmpty())
        return;
    
    _trial_manager->setUserID(s.toInt());
    
    TrialLoader::Condition c = s.endsWith("1") ?
        TrialLoader::Predictable :
        TrialLoader::UnPredictable;
    
    
    //todo set the trial based on this info
    //_trial_manager->setTrial

    close();
}

void UserIdWidget::cancelClicked()
{
    close();
}

}