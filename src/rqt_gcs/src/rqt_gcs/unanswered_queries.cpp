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
    layout_by_ap_type.insert("door", widget.queriesLayoutDoor); 
    //TODO layout_by_ap_type for window and hole, 
    //and adding the layouts and tab widget container in qtdesigner
    
    accept_mapper = new QSignalMapper(this);
    reject_mapper = new QSignalMapper(this);
    connect(accept_mapper, SIGNAL(mapped(QWidget *)), this, SLOT(acceptQuery(QWidget *)));
    connect(reject_mapper, SIGNAL(mapped(QWidget *)), this, SLOT(rejectQuery(QWidget *)));

    addUnansweredQueriesFromDisk();
}

UnansweredQueries::~UnansweredQueries()
{
    gcs = nullptr;
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path_root = gcs->image_root_path_ + "/queries/unanswered";         
    QVector<QString> ap_types = {"door", "window", "hole"};
    for(int i = 0; i < ap_types.size(); i++)
    {
        QDir path(path_root + "/" + ap_types[i]);
        path.setSorting(QDir::Time);
        QDirIterator it(path ,QDirIterator::Subdirectories);
        for(; it.hasNext(); it.next())
        {
            QString file_name = it.fileName();
            if(file_name.contains(".jpg"))
            {
                QString file_path = it.fileInfo().absoluteFilePath();
                QImage * image = new QImage(file_path);
                QueryStat * stat = new QueryStat();
                stat->uav_id = uavIdFromDir(file_path);
                stat->image = image;
                stat->image_file_path = file_path;
                addQueryWidget(stat, ap_types[i]);
            }
        }
    }
}

void UnansweredQueries::addQueryWidget(QueryStat* stat, QString ap_type)
{   
    //create the widget
    QWidget * pmWidget = new QWidget();
    Ui::PictureMsgWidget pmUiWidget;
    pmUiWidget.setupUi(pmWidget);

    QWidget * imgWidget = new QWidget();
    Ui::ImageViewWidget imgUiWidget;
    imgUiWidget.setupUi(imgWidget);

    // take care of the image
    imgUiWidget.image_frame->setImage(*stat->image);
    pmUiWidget.PictureLayout->addWidget(imgWidget);
    
    QVector<QueryStat*> * ap_vec = &queries_map[ap_type];
    ap_vec->push_back(stat);

    accept_mapper->setMapping(pmUiWidget.yesButton, pmWidget);
    connect(pmUiWidget.yesButton, SIGNAL(clicked()), accept_mapper, SLOT(map()));

    reject_mapper->setMapping(pmUiWidget.rejectButton, pmWidget);
    connect(pmUiWidget.rejectButton, SIGNAL(clicked()), reject_mapper, SLOT(map()));
    
    layout_by_ap_type[ap_type]->addWidget(pmWidget);
    this->resize(pmWidget->width() + 15, this->height());
}

void UnansweredQueries::acceptQuery(QWidget* w)
{
    answerQuery(w, "door", true);
}

void UnansweredQueries::rejectQuery(QWidget* w)
{
    answerQuery(w, "door", false);
}

void UnansweredQueries::answerQuery(QWidget * w, QString ap_type, bool accepted)
{
    int index = layout_by_ap_type[ap_type]->indexOf(w);
    
    QueryStat * stat = queries_map[ap_type][index];
    QImage * image = stat->image;

    SimpleControl * uav = nullptr;
    for(int i = 0; i < gcs->active_uavs.size(); i++)
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
        path.append("/accepted/" + ap_type + "/uav_" + QString::number(stat->uav_id));
        int num_images = numImagesInDir(path);
        file = "img_" + QString::number(num_images) + ".jpg";
        if(uav != nullptr)
            uav->accepted_images++;
    }
    else
    {
        path.append("/rejected/" + ap_type + "/uav_" + QString::number(stat->uav_id));
        int num_images = numImagesInDir(path);
        file = "img_" + QString::number(num_images) + ".jpg";
        if(uav != nullptr)
            uav->rejected_images++;
    }

    saveImage(path, file, image);
    QDir().remove(stat->image_file_path);
    
    layout_by_ap_type[ap_type]->removeWidget(w);
    QVector<QueryStat*> * ap_vector = &queries_map[ap_type];
    ap_vector->erase(ap_vector->begin()+index); // removes w from vector
    
    delete stat; 
    delete w;
}

int UnansweredQueries::numImagesInDir(QString dir_path)
{
    QDir dir(dir_path);
    dir.setNameFilters(QStringList()<<"*.jpg");
    return dir.entryList().size();
}

int UnansweredQueries::uavIdFromDir(QString s)
{  
    int start = s.indexOf("/uav_");
    QString temp = s.mid(start+5, 3);
    int stop = temp .indexOf('/');
    return temp.mid(0, stop).toInt();
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

