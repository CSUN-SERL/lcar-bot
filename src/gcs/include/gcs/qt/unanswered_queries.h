
#ifndef _UNANSWEREDQUERIES_H
#define _UNANSWEREDQUERIES_H

#include "ui_UnansweredQueries.h"
#include <gcs/qt/gcs_main_window.h>

namespace gcs
{
    
class GCSMainWindow;
    
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
    
    UnansweredQueries(QString& img_dir);
    virtual ~UnansweredQueries();
    
    void addQueryWidget(QueryStat*, QString&);
    void addUnansweredQueriesFromDisk();
    int uavIdFromDir(QString&);
    
       
public slots:
    void acceptQuery(QWidget*);
    void rejectQuery(QWidget*);
    void OnUpdateImageRootDir(QString new_dir);
    
private:
    Ui::UnansweredQueries widget;
    QMap<QString, QVBoxLayout*> layout_by_ap_type;
    QMap <QString, QVector<QueryStat*> > queries_map;
    QVector<QString> ap_types = {"door", "window", "hole"};
    QString image_root_dir;

    void answerQuery(QWidget*, QString, bool);
};

}// namespace gcs
#endif /* _UNANSWEREDQUERIES_H */
