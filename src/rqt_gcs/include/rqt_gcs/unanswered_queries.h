/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   UnansweredQueries.h
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#ifndef _UNANSWEREDQUERIES_H
#define _UNANSWEREDQUERIES_H

#include "ui_UnansweredQueries.h"
#include "simple_gcs.h"

namespace rqt_gcs
{

class UnansweredQueries : public QWidget
{
    Q_OBJECT
public:
    UnansweredQueries(SimpleGCS *);
    virtual ~UnansweredQueries();
    void addQuery(cv::Mat&);
    void addAllQueries();
    void parseDirectory
    void removeQuery(std::string);
    void removeAllQueries();
    
private:
    Ui::UnansweredQueries widget;
    
    SimpleGCS * gcs;
};


}// namespace rqt_gcs
#endif /* _UNANSWEREDQUERIES_H */
