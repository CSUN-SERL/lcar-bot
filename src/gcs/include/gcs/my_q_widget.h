
/* 
 * File:   my_q_widget.h
 * Author: n8
 *
 * Created on September 17, 2016, 11:16 PM
 */

#ifndef MYQWIDGET_H
#define MYQWIDGET_H

#include <QStyleOption>
#include <QPainter>
#include <QWidget>


class MyQWidget : public QWidget
{
    
    public:
        MyQWidget(QWidget *parent = 0):QWidget(parent){}
        
        virtual void paintEvent(QPaintEvent *pe)
        {                                                                                                                                        
            QStyleOption o;                                                                                                                                                                  
            o.initFrom(this);                                                                                                                                                                
            QPainter p(this);                                                                                                                                                                
            style()->drawPrimitive(
              QStyle::PE_Widget, &o, &p, this);                                                                                                                         
        };
};

#endif /* MYQWIDGET_H */

