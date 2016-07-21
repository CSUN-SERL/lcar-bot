/*
 * File:   UnansweredQueries.cpp
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#include "rqt_gcs/unanswered_queries.h"

namespace rqt_gcs
{
    
UnansweredQueries::UnansweredQueries(SimpleGCS * sgcs) :
gcs(sgcs)
{
    widget.setupUi(this);
 // todo
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
    QStringList list = s.split("_");
    return list.takeLast().toInt();
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path = gcs->image_root_path_;
    QVector<QString> ap_types = {"door", "window", "hole"};
    for(auto const& ap_type : ap_types)
    {
        QDir dir(path +  "/queries"  + "/unanswered/" +  ap_type);
        dir.setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks);
        QFileInfoList list = dir.entryInfoList();
        for(int i = 0; i < list.size(); i++)
        {
            QFileInfo info = list.at(i);
            dir.cd(info.filePath());
            QFileInfoList images_list = dir.entryInfoList();
            for(int j = 0; j < images_list.size(); j++)
            {
                QueryStat * stat = new QueryStat();
                stat->uav_id = uavIdFromDirectory(info.fileName());
                QString image_path = images_list.at(j).absoluteFilePath();
                QUrl url(image_path);
                if(url.isValid())
                {
                    stat->image_file_path = image_path;
                    QImage * image = new QImage();
                    image->load(url.fileName(), "jpg");
                    stat->image = image;
                    addQuery(stat, ap_type);

                }
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

    query_widgets.push_back(pmWidget);

    // take care of the image
    imgUiWidget.image_frame->setImage(*stat->image);
    widget.verticalLayout->addWidget(pmWidget);

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

