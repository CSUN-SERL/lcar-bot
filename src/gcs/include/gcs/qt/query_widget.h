/* 
 * File:   query_widget.h
 * Author: n8
 *
 * Created on September 26, 2016, 7:20 PM
 */

#ifndef _QUERYWIDGET_H
#define _QUERYWIDGET_H


#include "ui_QueryWidget.h"
#include <gcs/qt/my_q_widget.h>
#include <gcs/qt/building.h>

namespace gcs
{

class TrialManager;
    
class QueryWidget : public MyQWidget
{   
    Q_OBJECT
public:
    QueryWidget(TrialManager * trial_manager, int building_id);
    virtual ~QueryWidget();
    void SetImage(const QPixmap& img);
    
private slots:
    void accept();
    void reject();
    
signals:
    void queryAnswered(BuildingID building_id, PromptAnswer answer);
    
private:
    void answered(PromptAnswer answer);

private:
    Ui::QueryWidget widget;
    TrialManager * _trial_manager;
    int _building_id;
};

}
#endif /* _QUERYWIDGET_H */
