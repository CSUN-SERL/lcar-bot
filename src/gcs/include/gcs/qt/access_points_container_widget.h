
#ifndef _ACCESSPOINTS_H
#define _ACCESSPOINTS_H

#include <QTimer>
#include <QSignalMapper>

#include "ui_AccessPointsContainerWidget.h"

#include <lcar_msgs/AccessPointStamped.h>

namespace gcs
{
    
class AccessPointsContainerWidget : public QWidget
{
    Q_OBJECT
public:
    AccessPointsContainerWidget(QString& img_dir);
    virtual ~AccessPointsContainerWidget();
    void ClearAccessPoints();
    void UpdateAccessPoints();
    void SetUAVAccessPointsAndId(std::vector<lcar_msgs::AccessPointStampedPtr> *ap_vec, int v_id = -1);
    static void SaveUavAccessPoints(std::vector<lcar_msgs::AccessPointStampedPtr> *ap_vector, int id, QString ap_type, QString img_dir);
    
public slots:
    void OnDeleteAccessPoint(QWidget* w);
    void OnUpdateImageRootDir(QString new_dir);
    
private:
    Ui::AccessPointsContainerWidget widget;
    QTimer* timer;
    int num_access_points_last;
    QString image_root_dir;
    
    std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vec;
};

}
#endif /* _ACCESSPOINTS_H */
