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
//    connect(accept_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(acceptQuery(QWidget*)));
//    connect(reject_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(rejectQuery(QWidget*)));
    
}

UnansweredQueries::~UnansweredQueries()
{
    gcs = nullptr;
}

int UnansweredQueries::idFromDir(QString s)
{
    QStringList list = s.split("_");
    return list.takeLast().toInt();
}

void UnansweredQueries::addUnansweredQueriesFromDisk()
{
    QString path = gcs->image_root_path_ + "/queries/unanswered";
    QVector<QString> ap_types(3);
    ap_types[0] = "door";
    ap_types[1] = "window";
    ap_types[2] = "hole"; 
    for(auto const& entry : ap_types)
    {
        QDir dir (path + "/" + entry);
        dir.setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks);
        QFileInfoList list = dir.entryInfoList();
        for(int i = 0; i < list.size(); i++)
        {
            QFileInfo info = list.at(i);
            dir.cd(info.filePath());
            QFileInfoList images_list = dir.entryInfoList();
            for(int j = 0; j < images_list.size(); j++)
            {
//                QueryStat*  stat = new QueryStat();
//                stat->uav_id = idFromDir(info.fileName());
//                QUrl url(images_list.at(j));
//                if(url)
//                QImage * image ()
                // todo: figure out directory traversal and adding Qimages from hard drive
            }
        }
    }

}

void UnansweredQueries::addQuery(QueryStat*)
{
    // todo : change this to use QueryStat
    //retrieve Query msg for door image
//    QImage image(img.data,
//                 img.cols,
//                 img.rows, 
//                 QImage::Format_RGB888);
//
//    queries_map[ap_type].push_back(&image);
//    
//    //create the widget
//    QWidget * pmWidget = new QWidget();
//    Ui::PictureMsgWidget pmUiWidget;
//    pmUiWidget.setupUi(pmWidget);
//
//    QWidget * imgWidget = new QWidget();
//    Ui::ImageViewWidget imgUiWidget;
//    imgUiWidget.setupUi(imgWidget);
//
//    query_widgets.push_back(pmWidget);
//    
//    // take care of the image
//    imgUiWidget.image_frame->setImage(image);
//    widget.verticalLayout->addWidget(pmWidget);
//    
//    // handle button clicks
//    int index = query_widgets.size() - 1;
//    accept_mapper->setMapping(pmUiWidget.yesButton, query_widgets[index]);
//    connect(pmUiWidget.yesButton, SIGNAL(clicked()),
//            accept_mapper, SLOT(map()));
//
//    reject_mapper->setMapping(pmUiWidget.rejectButton, query_widgets[index]);
//    connect(pmUiWidget.rejectButton, SIGNAL(clicked()),
//            accept_mapper, SLOT(map()));
//    
}

}