/* 
 * File:   query_widget.h
 * Author: n8
 *
 * Created on September 26, 2016, 7:20 PM
 */

#ifndef _QUERYWIDGET_H
#define _QUERYWIDGET_H

#include <QPixmap>
#include "gcs/my_q_widget.h"
#include "ui_QueryWidget.h"

namespace gcs
{

class QueryWidget : public MyQWidget
{
    Q_OBJECT
public:
    QueryWidget();
    virtual ~QueryWidget();
    void SetImage(const QPixmap& img);
    const QPushButton* YesButton();
    const QPushButton* RejectButton();
private:
    Ui::QueryWidget widget;
};

}
#endif /* _QUERYWIDGET_H */
