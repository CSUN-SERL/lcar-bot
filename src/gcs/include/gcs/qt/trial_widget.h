
/* 
 * File:   TrialIdWidget.h
 * Author: n8
 *
 * Created on July 30, 2017, 12:14 AM
 */

#ifndef _TRIALIDWIDGET_H
#define _TRIALIDWIDGET_H

#include "ui_TrialWidget.h"

namespace gcs
{

class TrialManager;
    
class TrialWidget : public QWidget
{
    Q_OBJECT
    enum ViewState
    {
        Null,
        NewUser,
        NextTrial,
    };

public:
    TrialWidget(TrialManager* trial_manager);
    virtual ~TrialWidget();
    
private slots:
    void okClicked();
    void cancelClicked();
    void resetClicked();
    
    void trialChanged();
    
    void setViewState(ViewState state);
    
private:    
    Ui::TrialWidget widget;
    
    TrialManager * _trial_manager;
};

}

#endif /* _TRIALIDWIDGET_H */
