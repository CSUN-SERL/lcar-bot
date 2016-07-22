/*
 * File:   UnansweredQueries.cpp
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#include "rqt_gcs/unanswered_queries.h"

//Q_LOGGING_CATEGORY(rqt_gcsUnansweredQueries, "rqt_gcs.unanswered_queries")

namespace rqt_gcs
{
    
UnansweredQueries::UnansweredQueries(SimpleGCS * sgcs) :
gcs(sgcs)
{
    widget.setupUi(this);

    accept_mapper = new QSignalMapper(this);
    reject_mapper = new QSignalMapper(this);
    connect(accept_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(acceptQuery(QWidget*)));
    connect(reject_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(rejectQuery(QWidget*)));

    addUnansweredQueriesFromDisk();
}

UnansweredQueries::~UnansweredQueries()
{
    gcs = nullptr;
}

int UnansweredQueries::uavIdFromDirectory(QString s)
{  
    int start = s.indexOf("/uav_");
    QString temp = s.mid(start+5, 3);
    int stop = temp .indexOf('/');
    return temp.mid(0, stop).toInt();
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path_root = gcs->image_root_path_ + "/queries/unanswered";         
    QVector<QString> ap_types = {"door", "window", "hole"};
    for(int i = 0; i < ap_types.size(); i++)
    {
        QString path = path_root + "/" + ap_types[i];
        QDirIterator it(path, QDirIterator::Subdirectories);
        for(; it.hasNext(); it.next())
        {
            if(it.fileName().contains(".jpg"))
            {
                QString file_path = it.filePath();
                QImage * image = new QImage(file_path);
                QueryStat * stat = new QueryStat();
                stat->image = image;
                stat->uav_id = uavIdFromDirectory(file_path);
                stat->image_file_path = file_path;
                addQuery(stat, ap_types[i]);
            }

        }
    }
}

void UnansweredQueries::addQuery(QueryStat* stat, QString ap_type)
{
    queries_map[ap_type].push_back(stat);

    //create the widget
    QWidget * pmWidget = new QWidget();
    Ui::PictureMsgWidget pmUiWidget;
    pmUiWidget.setupUi(pmWidget);

    QWidget * imgWidget = new QWidget();
    Ui::ImageViewWidget imgUiWidget;
    imgUiWidget.setupUi(imgWidget);

    // take care of the image
    imgUiWidget.image_frame->setImage(*stat->image);
    widget.verticalLayout->addWidget(pmWidget);
    
    query_widgets.push_back(pmWidget);

    // handle button clicks
    int index = query_widgets.size() - 1;
    accept_mapper->setMapping(pmUiWidget.yesButton, query_widgets[index]);
    connect(pmUiWidget.yesButton, SIGNAL(clicked()), accept_mapper, SLOT(map()));

    reject_mapper->setMapping(pmUiWidget.rejectButton, query_widgets[index]);
    connect(pmUiWidget.rejectButton, SIGNAL(clicked()), accept_mapper, SLOT(map()));
}

void UnansweredQueries::acceptQuery(QWidget* w)
{
    answerQuery(w, "door", true);
}

void UnansweredQueries::rejectQuery(QWidget* w)
{
    answerQuery(w, "door", false);
}

void UnansweredQueries::answerQuery(QWidget* w, QString ap_type, bool accepted)
{
    int index = widget.queriesLayout->indexOf(w);
    QueryStat* stat = queries_map[ap_type][index];
    QImage * image = stat->image;

    SimpleControl* uav;
    int i = 0;
    while(i < gcs->active_uavs.size())
    {
        if(gcs->active_uavs[i]->id == stat->uav_id)
        {
            uav = gcs->active_uavs[i];
            break;
        }
    }

    QString path = gcs->image_root_path_ + "/queries";
    QString file;
    if(accepted)
    {
        path.append("/accepted/" + ap_type + "/uav_" + QString::number(uav->id));
        file = "img_" + QString::number(uav->accepted_images++) + ".jpg";
    }
    else
    {
        path = path + "/rejected/" + ap_type + "/uav_" + QString::number(uav->id);
        file = "img_" + QString::number(uav->rejected_images++) + ".jpg";
    }

    saveImage(path, file, image);

    QDir image_file(stat->image_file_path);
    image_file.remove(stat->image_file_path);
}

void UnansweredQueries::saveImage(QString path, QString file, QImage* image)
{
    QDir dir(path);
    if(!dir.exists())
        dir.mkdir(path);

    QString full_path = dir.currentPath().append("/" + file);
    image->save(full_path, "jpg", -1);
}



}

