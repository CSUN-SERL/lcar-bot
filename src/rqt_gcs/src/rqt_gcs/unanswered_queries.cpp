/*
 * File:   UnansweredQueries.cpp
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#include "rqt_gcs/unanswered_queries.h"
#include "util/image.h"
#include "util/debug.h"

#include "ui_PictureMsg.h"

#include <QDir>
#include <QDirIterator>
#include <QUrl>

namespace rqt_gcs
{
    
UnansweredQueries::UnansweredQueries(GCS * sgcs) :
gcs(sgcs)
{
    widget.setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);

    layout_by_ap_type.insert("door", widget.queriesLayoutDoor); 
    //TODO layout_by_ap_type for window and hole, 
    //and adding the layouts and tab widget container in qtdesigner

    addUnansweredQueriesFromDisk();
}

UnansweredQueries::~UnansweredQueries()
{
    gcs = nullptr;
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path_root = image_util::image_root_dir_ % "/queries/unanswered";         
    for(int i = 0; i < ap_types.size(); i++)
    {
        QDir path(path_root % "/" % ap_types[i]);
        path.setNameFilters(QStringList()<<"uav_*");
        QFileInfoList uav_list = path.entryInfoList();
        int num_uavs = uav_list.size();
        for(int u = 0; u < num_uavs; u++)
        {
            QDir uav_path(uav_list[u].canonicalFilePath()); // <path>/uav_x
            uav_path.setNameFilters(QStringList()<<"*.jpg");
            QFileInfoList img_list = uav_path.entryInfoList(); 
            int num_imgs = img_list.size();
            for(int j = 0; j < num_imgs; j++)
            {
                QString file_name = img_list[j].fileName();
                int img_num = image_util::imgNumFromFile(file_name);
                if(img_num % 2 == 0)
                {
                    QString img_path = img_list[j].canonicalFilePath(); // <path>/uav_x/img_x.jpg
                    QueryStat * stat = new QueryStat();
                    stat->uav_id = uavIdFromDir(img_path);
                    stat->original_img = QImage(img_path);
                    QString base_path = image_util::getImgBasePath(img_path);

                    QString framed_img_path = base_path % "/img_" 
                            % QString::number(img_num+1) % ".jpg";
                    stat->framed_img = QImage(framed_img_path);

                    stat->og_img_file_path = img_path;
                    stat->fr_img_file_path = framed_img_path;
                    addQueryWidget(stat, ap_types[i]);
                }
            }
        }
    }
}

void UnansweredQueries::addQueryWidget(QueryStat* stat, QString& ap_type)
{   
    //create the widget
    QWidget * pm_widget = new QWidget(this);
    Ui::PictureMsgWidget pm_ui;
    pm_ui.setupUi(pm_widget);

    // take care of the image
    int w = pm_ui.image_frame->width();
    int h = pm_ui.image_frame->height();
    pm_ui.image_frame->setPixmap(QPixmap::fromImage(stat->framed_img).scaled(w,h));
    
    QVector<QueryStat*> * ap_vec = &queries_map[ap_type];
    ap_vec->push_back(stat);

    connect(pm_ui.yesButton, &QPushButton::clicked, 
            this, [=](){ acceptQuery(pm_widget); } );
    connect(pm_ui.rejectButton, &QPushButton::clicked, 
            this, [=](){ rejectQuery(pm_widget); } );
     
    layout_by_ap_type[ap_type]->addWidget(pm_widget);
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
    QImage img = stat->original_img;

    UAVControl * uav = nullptr;

    if(gcs->uav_db.count(stat->uav_id) > 0)
        uav = gcs->uav_db[stat->uav_id];
    
    QString path = image_util::image_root_dir_ % "/queries";
    QString file;
    if(accepted)
    {
        path.append("/accepted/" % ap_type % "/uav_" % QString::number(stat->uav_id));
        int num_images = image_util::numImagesInDir(path);
        file = "img_" % QString::number(num_images) % ".jpg";
        if(uav != nullptr)
            uav->accepted_images++;
    }
    else
    {
        path.append("/rejected/" % ap_type % "/uav_" % QString::number(stat->uav_id));
        int num_images = image_util::numImagesInDir(path);
        file = "img_" % QString::number(num_images) % ".jpg";
        if(uav != nullptr)
            uav->rejected_images++;
    }

    if(image_util::saveImage(path, file, img))
    {
        QDir dir;
        dir.remove(stat->og_img_file_path);
        dir.remove(stat->fr_img_file_path);
    }
    
    QVector<QueryStat*> * ap_vector = &queries_map[ap_type];
    ap_vector->erase(ap_vector->begin()+index); // removes w from vector
    
    delete stat; 
    delete w;
}

int UnansweredQueries::uavIdFromDir(QString& dir)
{  
    int start = dir.indexOf("/uav_");
    QString temp = dir.mid(start+5, 4);
    int stop = temp.indexOf('/');
    return temp.mid(0, stop).toInt();
}

}

