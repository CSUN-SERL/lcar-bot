
#ifndef _ACCESSPOINTS_H
#define _ACCESSPOINTS_H

#include <QTimer>
#include <QSignalMapper>

#include "ui_AccessPointsContainerWidget.h"
#include "lcar_msgs/AccessPointStamped.h"

namespace gcs
{
    
class AccessPointsContainerWidget : public QWidget
{
    Q_OBJECT
public:
    AccessPointsContainerWidget();
    virtual ~AccessPointsContainerWidget();
    void ClearAccessPoints();
    void UpdateAccessPoints();
    static void SaveUavAccessPoints(std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vector, int id, QString ap_type);
    void SetUAVAccessPointsAndId(std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vec, int id = -1);
    
public slots:
    void OnDeleteAccessPoint(QWidget* w);
    
private:
    Ui::AccessPointsContainerWidget widget;
    QTimer* timer;
    int num_access_points_last;
    
    std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vec;
};

}
#endif /* _ACCESSPOINTS_H */
