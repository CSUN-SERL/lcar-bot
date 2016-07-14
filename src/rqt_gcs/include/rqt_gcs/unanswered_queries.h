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
#include <QDir>
#include "simple_gcs.h"

namespace rqt_gcs
{
    
class UnansweredQueries : public QWidget
{
    Q_OBJECT
public:
    struct QueryStat
    {
        int uav_id;
        QImage image; 
    };
    
    UnansweredQueries(SimpleGCS *);
    virtual ~UnansweredQueries();
    void addQuery(QueryStat *);
    void addUnansweredQueriesFromDisk();
    void parseDirectory();
    void removeQuery(std::string);
    void removeAllQueries();
    
private:
    Ui::UnansweredQueries widget;
    
    SimpleGCS * gcs;
    std::map <std::string, std::vector<QueryStat*> > queries_map;
    std::vector<QWidget *> query_widgets;

    QSignalMapper accept_mapper;
    QSignalMapper reject_mapper;
    
    int idFromDir(QString);
    answerQuery(QWidget*);
    
signals:
    saveImage(std::string path, std::string file, cv::Mat);

public slots:
    acceptQuery(QWidget*);
    rejectQuery(QWidget*);
};




}// namespace rqt_gcs
#endif /* _UNANSWEREDQUERIES_H */
