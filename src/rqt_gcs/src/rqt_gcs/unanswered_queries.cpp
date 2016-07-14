/*
 * File:   UnansweredQueries.cpp
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#include "rqt_gcs/unanswered_queries.h"

namespace rqt_gcs
{

namespace fs = boost::filesystem;
    
UnansweredQueries::UnansweredQueries(SimpleGCS * sgcs) :
gcs(sgcs)
{
    widget.setupUi(this);
 // todo   
   // connect(accept_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(acceptQuery(QWidget*)));
   // connect(reject_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(rejectQuery(QWidget*)));
    
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
    QString path = gcs->image_root_path_ + "/queries/unanswered";
    QVector<QString> ap_types = {"door","window", "hole"};
    for(auto const& ap_type : ap_types)
    {
        QDir dir (path + "/" + ap_type);
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
                QUrl url(images_list.at(j).filePath());
                if(url.isValid())
                {
                    QImage * image = new QImage();
                    image->load(url.fileName(), "jpg");
                    stat->image = image;
                    addQuery(stat, ap_type.toStdString());
                }
            }
        }
    }

}

void UnansweredQueries::addQuery(QueryStat* stat, std::string ap_type)
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
    accept_mapper.setMapping(pmUiWidget.yesButton, query_widgets[index]);
    //connect(pmUiWidget.yesButton, SIGNAL(clicked()), accept_mapper, SLOT(map()));

    reject_mapper.setMapping(pmUiWidget.rejectButton, query_widgets[index]);
    //connect(pmUiWidget.rejectButton, SIGNAL(clicked()), accept_mapper, SLOT(map()));
}

void UnansweredQueries::acceptQuery(QWidget* w)
{
    answerQuery(w, "door", true);
}

void UnansweredQueries::rejectQuery(QWidget* w)
{
    answerQuery(w, "door", false);
}

void UnansweredQueries::answerQuery(QWidget* w, std::string ap_type, bool accepted)
{
    
        int index = widget.queriesLayout->indexOf(w);
        
//
//        std::string path = gcs->image_root_path_.toStdString() + "/queries";
//        std::string file;
//        if(accepted)
//        {
//            path += "/accepted/" + ap_type + "/uav_" + ap_type;
//            file = "img_" + std::to_string(accepted_images++) + ".jpg";
//        }
//        else
//        {
//            path += "/rejected/" + ap_type + "/uav_" + std::to_string(uav->id);
//            file = "img_" + std::to_string(uav->rejected_images++) + ".jpg";
//        }
//        
//        //dont change my colorspace! (rgb8)
//        saveImage(path, file,
//            cv_bridge::toCvCopy((sensor_msgs::Image)door->original_picture, "rgb8")->image);

}

}