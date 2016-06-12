#ifndef rqt_gcs__SimpleGCS_H
#define rqt_gcs__SimpleGCS_H

#include <ros/ros.h>
#include <ros/common.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_gcs/simple_control.h>
#include <rqt_gcs/settings_widget.h>
#include <query_msgs/Door.h>
#include <query_msgs/Target.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>

#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

#include <ui_gcs.h>
#include <ui_MissionCancel.h>
#include <ui_MissionProgress.h>
#include <ui_MissionSelect.h>
#include <ui_UavQuestion.h>
#include <ui_UavStat.h>
#include <ui_QuadStats.h>
#include <ui_UAVCondition.h>
#include <ui_WidgetMain.h>
#include <ui_PFDWidget_custom.h>
#include <ui_ImageView.h>
#include <ui_MissionConfirm.h>
#include <ui_AccessPointsMenu.h>
#include <ui_AccessPointStats.h>
#include <ui_PictureMsg.h>

#include <QWidget>
#include <QLabel>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QMainWindow>
#include <QSignalMapper>

#define NUM_UAV 2 //Total number of UAV's in the system



namespace rqt_gcs{



  class SimpleGCS: public rqt_gui_cpp::Plugin
  {
  Q_OBJECT
  public:
    SimpleGCS();
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::ServiceServer server;
    query_msgs::Door msg;

    image_transport::ImageTransport it_stereo{nh};
    void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);


  protected slots:
    virtual void TimedUpdate();
    virtual void ExecutePlay();
    virtual void CancelPlay();
    virtual void ScoutBuilding();
    virtual void StopQuad();
    virtual void ChangeFlightMode();
    virtual void RefreshAccessPointsMenu();
    virtual void DeleteAccessPoint(QWidget*);
    virtual void QuadSelect(int);
    virtual void ArmSelectedQuad();
    virtual void DisarmSelectedQuad();
    virtual void AcceptDoorQuery(QWidget*);
    virtual void DenyDoorQuery(QWidget*);
    virtual void SettingsClicked();
    virtual void DestroySettingsWidget();


  private:
    void UpdatePFD();
    void UpdateMsgQuery();
    int cur_uav = 0;
    int timeCounter = 0;
    query_msgs::Target GetMission(std::string fileName);
    SimpleControl quadrotors[NUM_UAV] = {};
    std::vector<AccessPoint> * accessPointsVector;
    std::vector<query_msgs::Door> * pictureQueryVector;
    //std::vector<sensor_msgs::Image> * pictureQueryVector;

    cv::Mat conversion_mat_;
    image_transport::Subscriber sub_stereo = it_stereo.subscribe("/UAV1/stereo_cam/left/image_rect", 
                                                                 1, &SimpleGCS::ImageCallback, this);

    Ui::SimpleGCSWidget ui_;
    Ui::MissionProgressWidget mpUi_;
    Ui::UavQuestionWidget uqUi_;
    Ui::UavStatWidget usUi_;
    Ui::ImageViewWidget ivUi_;
    Ui::PFDWidget pfd_ui;
    Ui::centralWidget central_ui_;
    Ui::UAVConditionWidget uavCondWidgetArr[NUM_UAV];
    Ui::AccessPointsMenuWidget apmUi_;
    Ui::PictureMsgWidget pmUi_;

    QWidget* widget_;
    QWidget* missionProgressWidget_;
    QWidget* uavQuestionWidget_;
    QWidget* uavStatWidget_;
    QWidget* imageViewWidget_;
    QWidget* uavListWidgetArr[NUM_UAV];
    QWidget* PFDQWidget;
    QWidget* apmQWidget_;
    QWidget* settings_widget_;

    std::vector<QWidget*> accessPointsQWidgets_;
    std::vector<QWidget*> pictureMsgQWidgets_;


    QLabel* label;
    QTimer* update_timer;

    QString temp_data;
    QString quad_id;
    QString access_point_temp_data;
    QString access_point_id;
    QSignalMapper* signal_mapper;
    QSignalMapper* signal_mapper2;
    QSignalMapper* acceptDoorMapper;
    QSignalMapper* denyDoorMapper;
    
    QSettings *settings_;
  };
} // name space
#endif // my_namespace__my_plugin_H
