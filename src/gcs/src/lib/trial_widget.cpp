
/*
 * File:   trial_widget.cpp
 * Author: n8
 *
 * Created on July 30, 2017, 12:14 AM
 */

#include <QIntValidator>

#include <gcs/qt/trial_widget.h>
#include <gcs/qt/trial_manager.h>

namespace gcs
{

TrialWidget::TrialWidget(TrialManager* trial_manager):
_trial_manager(trial_manager)
{
    widget.setupUi(this);
    auto validator = new QIntValidator(this);
    validator->setBottom(0);
    validator->setTop(100000);
    widget.id_line->setValidator(validator);
    
    QObject::connect(_trial_manager, &TrialManager::trialChanged,
                    this, &TrialWidget::trialChanged);
    
    QObject::connect(_trial_manager, &TrialManager::sigReset,
                    this, [this]() { setViewState(NewUser); });
    
    QObject::connect(widget.btn_ok, &QPushButton::clicked,
                    this, &TrialWidget::okClicked);
    
    QObject::connect(widget.btn_cancel, &QPushButton::clicked,
                    this, &TrialWidget::cancelClicked);
    
    QObject::connect(widget.btn_reset, &QPushButton::clicked,
                    this, &TrialWidget::resetClicked);
    
    QObject::connect(widget.btn_export, &QPushButton::clicked,
                    _trial_manager, &TrialManager::exportTrialData);
    
    setViewState(NewUser);
}

TrialWidget::~TrialWidget() 
{
}

void TrialWidget::enableExport(bool enable)
{
    widget.btn_export->setVisible(enable);
}

void TrialWidget::okClicked()
{
    QString s = widget.id_line->text();
    if(s.isEmpty())
        return;
    
    if(s.length() < 4)
        return;
        
    if(!s.endsWith("1") && !s.endsWith("2"))
        return;
    
    qCDebug(lcar_bot) << "current trial" << _trial_manager->currentTrial();
    
    qCDebug(lcar_bot) << "current condition" << _trial_manager->currentCondition();
    
    setViewState(NextTrial);
    
    if(_trial_manager->isValid())
    {
        _trial_manager->nextTrial();
    }
    else
    {
        _trial_manager->setUserID(s);
        TrialLoader::Condition c = s.endsWith("1") ?
            TrialLoader::Predictable :
            TrialLoader::UnPredictable;

        _trial_manager->setTrialStartCondition(c);
    }
}

void TrialWidget::resetClicked()
{
    setViewState(ViewState::NewUser);
    _trial_manager->reset();
}

void TrialWidget::cancelClicked()
{
    close();
}

void TrialWidget::trialChanged()
{
    if(_trial_manager->isValid())
    {
        TrialLoader::Condition c = _trial_manager->currentCondition();
        int trial = _trial_manager->currentTrial();
        QString s = c == TrialLoader::Predictable?
            "Predictable":
            "Unpredictable";

        widget.lbl_condition->setText(s);
        widget.lbl_trial->setText(QString::number(trial));

        setViewState(NextTrial);
    }
    else
    {
        setViewState(NewUser);
    }
}

void TrialWidget::setViewState(ViewState state)
{
    switch(state)
    {
        case NewUser:
            widget.id_line->setText(QString());
            widget.btn_reset->hide();
            widget.btn_ok->setText("Load Trial 1");
            widget.lbl_condition->setText("n/a");
            widget.lbl_trial->setText("n/a");
            widget.btn_ok->setMinimumWidth(65);
            widget.btn_export->hide();
            break;
        case NextTrial:
            widget.btn_ok->setText("Next Trial");
            widget.btn_ok->setMinimumWidth(50);
            widget.btn_reset->show();
            break;
        case Null:
            widget.id_line->setText(QString());
            widget.lbl_condition->setText("n/a");
            widget.lbl_trial->setText("n/a");
            widget.btn_reset->hide();
        default:
            break;
    }
}

}