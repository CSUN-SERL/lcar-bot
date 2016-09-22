
#ifndef _UNANSWEREDQUERIES_H
#define _UNANSWEREDQUERIES_H


#include <rqt_gcs/gcs.h>
#include "ui_UnansweredQueries.h"
#include <QSignalMapper>

namespace rqt_gcs
{
    
class GCS;
    
class UnansweredQueries : public QWidget
{
    Q_OBJECT
public:

    struct QueryStat
    {
        int uav_id;
        QImage original_img;
        QImage framed_img;
        QString og_img_file_path;
        QString fr_img_file_path;
        
        ~QueryStat() { }
    };
    
    UnansweredQueries(GCS*);
    virtual ~UnansweredQueries();
    
    void addQueryWidget(QueryStat*, QString&);
    void addUnansweredQueriesFromDisk();
    int uavIdFromDir(QString&);
    
private:
    Ui::UnansweredQueries widget;
    QMap<QString, QVBoxLayout*> layout_by_ap_type;
    QMap <QString, QVector<QueryStat*> > queries_map;
    QVector<QString> ap_types = {"door", "window", "hole"};
        
    GCS * gcs;

    void answerQuery(QWidget*, QString, bool);
    
public slots:
    void acceptQuery(QWidget*);
    void rejectQuery(QWidget*);
};

}// namespace rqt_gcs
#endif /* _UNANSWEREDQUERIES_H */
