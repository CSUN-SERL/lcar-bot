/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   unanswered_queries.h
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#ifndef _UNANSWEREDQUERIES_H
#define _UNANSWEREDQUERIES_H


#include <rqt_gcs/simple_gcs.h>

#include "ui_UnansweredQueries.h"
#include "ui_PictureMsg.h"

#include <QSignalMapper>

#include <QDir>
#include <QDirIterator>
#include <QUrl>

namespace rqt_gcs
{
    
class SimpleGCS;
    
class UnansweredQueries : public QWidget
{
Q_OBJECT
public:

    struct QueryStat
    {
        int uav_id;
        QImage * image;
        QString image_file_path;
    };

    UnansweredQueries(SimpleGCS*);
    virtual ~UnansweredQueries();
    void addQuery(QueryStat*, QString);

    void addUnansweredQueriesFromDisk();
    void parseDirectory();
    void removeQuery(std::string);
    void removeAllQueries();
    int numImagesInDir(QString);
    int uavIdFromDir(QString);
    void saveImage(QString, QString, QImage *);
    
private:
    Ui::UnansweredQueries widget;

    SimpleGCS * gcs;
    QMap <QString, QVector<QueryStat*> > queries_map;
    QVector<QWidget *> query_widgets;

    QSignalMapper * accept_mapper;
    QSignalMapper * reject_mapper;

    void answerQuery(QWidget*, QString, bool);
    
public slots:
    void acceptQuery(QWidget*);
    void rejectQuery(QWidget*);
};

}// namespace rqt_gcs
#endif /* _UNANSWEREDQUERIES_H */
