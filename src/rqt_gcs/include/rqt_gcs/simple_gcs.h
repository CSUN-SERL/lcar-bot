#ifndef rqt_gcs__SimpleGCS_H
#define rqt_gcs__SimpleGCS_H

#include <ros/ros.h>
#include <ros/common.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_gcs/simple_control.h>
#include <rqt_gcs/settings_widget.h>
#include <lcar_msgs/Door.h>
#include <lcar_msgs/Target.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>


#include <boost/filesystem/operations.hpp>
#include <boost/thread.hpp>

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
#include <QThread>
#include <QMainWindow>
#include <QSignalMapper>
#include <QDesktopWidget>

#define MAX_UAV 100 // the total number of UAV's manageable by our system


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
    lcar_msgs::Door msg;

    image_transport::ImageTransport it_stereo{nh};
    void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void initializeSettings();
    
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
    virtual void QuadSelected(int);
    virtual void ArmSelectedQuad();
    virtual void DisarmSelectedQuad();
    virtual void AcceptDoorQuery(QWidget*);
    virtual void DenyDoorQuery(QWidget*);
    virtual void SettingsClicked();
    virtual void MonitorUavNamespace();
    virtual void MonitorConnection();
    
    //SETTINGS RELATED
    virtual void DestroySettingsWidget();
    virtual void ToggleMachineLearningMode(bool);

  private:
    void UpdatePFD();
    void UpdateMsgQuery();
    lcar_msgs::Target GetMission(std::string fileName);
    void saveImage(bool, std::string, const cv::Mat&);
    void parseUavNamespace(std::map<int,int>&);
    void addUav(int);
    void deleteUav(int);
    void selectQuad(int);
    void initializeThreads();
    
    int cur_uav;
    int timeCounter;
    int NUM_UAV; //Total number of UAV's in the system
    
    
    std::vector<SimpleControl*> quadrotors;
    std::vector<AccessPoint> * accessPointsVector;
    std::vector<lcar_msgs::DoorPtr> * pictureQueryVector;

   
    cv::Mat conversion_mat_;
    image_transport::Subscriber sub_stereo;
    Ui::SimpleGCSWidget ui_;
    Ui::MissionProgressWidget mpUi_;
    Ui::UavQuestionWidget uqUi_;
    Ui::UavStatWidget usUi_;
    Ui::ImageViewWidget ivUi_;
    Ui::PFDWidget pfd_ui;
    Ui::centralWidget central_ui_;
    std::vector<Ui::UAVConditionWidget*> uavCondWidgetArr;
    Ui::AccessPointsMenuWidget apmUi_;
    Ui::PictureMsgWidget pmUi_;

    QWidget* widget_;
    QWidget* missionProgressWidget_;
    QWidget* uavQuestionWidget_;
    QWidget* uavStatWidget_;
    QWidget* imageViewWidget_;
    std::vector<QWidget*> uavListWidgetArr;
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
    QString image_root_path_;
    
    QMutex uav_mutex;
    QTimer  *uav_ns_timer;
    QThread *t_namespace_monitor;
    
    QTimer  *connection_timer;
    QThread *t_connection_monitor;
  };
  
} // rqt_gcs name space
#endif //rqt_gcs__SimpleGCS_H
