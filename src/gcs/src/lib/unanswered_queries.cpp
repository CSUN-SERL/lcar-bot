/*
 * File:   UnansweredQueries.cpp
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#include <QDir>
#include <QDirIterator>
#include <QUrl>

#include <gcs/qt/unanswered_queries.h>
#include <gcs/qt/ui_adapter.h>
#include <gcs/qt/query_widget.h>
#include <gcs/util/image_conversions.h>
#include <gcs/util/debug.h>
#include <gcs/util/flight_modes.h>

namespace gcs
{
    
UnansweredQueries::UnansweredQueries(QString& img_dir) :
    image_root_dir(img_dir)
{
    widget.setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);

    layout_by_ap_type.insert("door", widget.queriesLayoutDoor); 
    //TODO layout_by_ap_type for window and hole, 
    //and adding the layouts and tab widget container in qtdesigner
    
    connect(UIAdapter::Instance(), &UIAdapter::SetImageRootDir,
            this, &UnansweredQueries::OnUpdateImageRootDir);
    
    this->addUnansweredQueriesFromDisk();
}

UnansweredQueries::~UnansweredQueries()
{
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path_root = image_root_dir % "/queries/unanswered";         
    for(int i = 0; i < ap_types.size(); i++)
    {
        QDir path(path_root % "/" % ap_types[i]);
        path.setNameFilters(QStringList()<<"*.jpg");
        QFileInfoList img_list = path.entryInfoList(); 
        int num_imgs = img_list.size();
        if(num_imgs > 200) // more than 200 and the gui starts to hang
            num_imgs = 200;
        for(int j = 0; j < num_imgs; j++)
        {
            QString file_name = img_list[j].fileName();
            int img_num = image_conversions::imgNumFromFile(file_name);
            if(img_num % 2 == 0)
            {
                QString img_path = img_list[j].canonicalFilePath(); //<path_root>/queries/unanswered/<access_point_type>img_x.jpg
                QueryStat * stat = new QueryStat();
                stat->uav_id = uavIdFromDir(img_path);
                stat->original_img = QImage(img_path);
                QString base_path = image_conversions::getImgBasePath(img_path);

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

void UnansweredQueries::addQueryWidget(QueryStat* stat, QString& ap_type)
{   
    QVector<QueryStat*> *ap_vec = &queries_map[ap_type];
    ap_vec->push_back(stat);
    
    //create the widget
    QueryWidget *qw = new QueryWidget();
    qw->SetImage(QPixmap::fromImage(stat->framed_img));

    connect(qw->YesButton(), &QPushButton::clicked,
            this, [=](){acceptQuery(qw); });
    connect(qw->RejectButton(), &QPushButton::clicked,
            this, [=](){rejectQuery(qw); });

    layout_by_ap_type[ap_type]->addWidget(qw);
}

void UnansweredQueries::acceptQuery(QWidget *w)
{
    answerQuery(w, "door", true);
}

void UnansweredQueries::rejectQuery(QWidget *w)
{
    answerQuery(w, "door", false);
}

void UnansweredQueries::OnUpdateImageRootDir(QString new_dir)
{
    image_root_dir = new_dir;
}

void UnansweredQueries::answerQuery(QWidget *w, QString ap_type, bool accepted)
{
    int index = layout_by_ap_type[ap_type]->indexOf(w);
    
    QueryStat *stat = queries_map[ap_type][index];
    QImage img = stat->original_img;
    
    QString path = image_root_dir % "/queries";
    if(accepted)
        path.append("/accepted/" % ap_type);
    else 
        path.append("/rejeceted/" % ap_type);
                
    int num_images = image_conversions::numImagesInDir(path);
    QString file = "img_" % QString::number(num_images) % ".jpg";

    if(image_conversions::saveImage(path, file, img))
    {
        QDir dir;
        dir.remove(stat->og_img_file_path);
        dir.remove(stat->fr_img_file_path);
    }
    
    QVector<QueryStat*> *ap_vector = &queries_map[ap_type];
    ap_vector->erase(ap_vector->begin()+index); // removes stat from vector
    
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

