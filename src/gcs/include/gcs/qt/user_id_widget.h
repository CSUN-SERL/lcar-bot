
/* 
 * File:   TrialIdWidget.h
 * Author: n8
 *
 * Created on July 30, 2017, 12:14 AM
 */

#ifndef _TRIALIDWIDGET_H
#define _TRIALIDWIDGET_H

#include "ui_UserIdWidget.h"

namespace gcs
{

class TrialManager;
    
class UserIdWidget : public QDialog 
{
    Q_OBJECT
public:
    UserIdWidget(TrialManager* trial_manager);
    virtual ~UserIdWidget();
    
private slots:
    void okClicked();
    void cancelClicked();
    
private:
    Ui::UserIdWidget widget;
    
    TrialManager * _trial_manager;
};

}

#endif /* _TRIALIDWIDGET_H */
