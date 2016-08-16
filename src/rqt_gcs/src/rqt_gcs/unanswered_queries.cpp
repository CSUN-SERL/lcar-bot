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
    this->setAttribute(Qt::WA_DeleteOnClose);
    
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

void UnansweredQueries::closeEvent(QCloseEvent* event)
{
    gcs->widgets_.unanswered_queries_ = nullptr;
    event->accept();
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path_root = gcs->image_root_dir_ % "/queries/unanswered";         
    QVector<QString> ap_types = {"door", "window", "hole"};
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
                int img_num = imgNumFromFile(file_name);
                if(img_num % 2 == 0)
                {
                    QString img_path = img_list[j].canonicalFilePath(); // <path>/uav_x/img_x.jpg
                    QueryStat * stat = new QueryStat();
                    stat->uav_id = uavIdFromDir(img_path);
                    stat->original_img = new QImage(img_path);
                    QString base_path = getImgBasePath(img_path);

                    QString framed_img_path = base_path % "/img_" 
                            % QString::number(img_num+1) % ".jpg";
                    stat->framed_img = new QImage(framed_img_path);

                    stat->og_img_file_path = img_path;
                    stat->fr_img_file_path = framed_img_path;
                    addQueryWidget(stat, ap_types[i]);
                }
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
    imgUiWidget.image_frame->setImage(*stat->framed_img);
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
    std::cout << "index " << index << "\n";
    
    QueryStat * stat = queries_map[ap_type][index];
    QImage * img = stat->original_img;

    SimpleControl * uav = nullptr;

    if(gcs->uav_db.count(stat->uav_id) > 0)
        uav = gcs->uav_db[stat->uav_id]->uav;
    
    QString path = gcs->image_root_dir_ % "/queries";
    QString file;
    if(accepted)
    {
        path.append("/accepted/" % ap_type % "/uav_" % QString::number(stat->uav_id));
        int num_images = numImagesInDir(path);
        file = "img_" % QString::number(num_images) % ".jpg";
        if(uav != nullptr)
            uav->accepted_images++;
    }
    else
    {
        path.append("/rejected/" % ap_type % "/uav_" % QString::number(stat->uav_id));
        int num_images = numImagesInDir(path);
        file = "img_" % QString::number(num_images) % ".jpg";
        if(uav != nullptr)
            uav->rejected_images++;
    }

    if(!saveImage(path, file, img))
        std::cout << "couldnt save image to path: " << path.append("/"%file).toStdString()<< "\n";
    
    QDir dir;
    dir.remove(stat->og_img_file_path);
    dir.remove(stat->fr_img_file_path);
    
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

int UnansweredQueries::uavIdFromDir(QString dir)
{  
    int start = dir.indexOf("/uav_");
    QString temp = dir.mid(start+5, 4);
    int stop = temp.indexOf('/');
    return temp.mid(0, stop).toInt();
}

int UnansweredQueries::imgNumFromFile(QString file)
{
    int start = file.indexOf("img_");
    QString temp = file.mid(start+4,4);
    int stop = temp.indexOf(".");
    return temp.mid(0, stop).toInt();
}

QString UnansweredQueries::getImgBasePath(QString file_path)
{
    int start = file_path.indexOf("/img_");
    return file_path.mid(0,start);
}

bool UnansweredQueries::saveImage(QString path, QString file, QImage* image)
{
    QDir dir(path);
    if(!dir.exists())
        dir.mkdir(path);

    QString full_path = dir.canonicalPath().append("/" % file);
    std::cout << "ful_path in saveImage(): " << full_path.toStdString() << "\n";
    return image->save(full_path, "jpg", -1);
}

}

