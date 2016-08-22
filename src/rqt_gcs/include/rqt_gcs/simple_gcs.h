#ifndef rqt_gcs__SimpleGCS_H
#define rqt_gcs__SimpleGCS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/common.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_gcs/simple_control.h>
#include <rqt_gcs/unanswered_queries.h>
#include <rqt_gcs/settings_widget.h>
#include <lcar_msgs/Door.h>
#include <lcar_msgs/TargetLocal.h>
#include <lcar_msgs/TargetGlobal.h>
#include <std_srvs/Empty.h>
#include <pluginlib/class_list_macros.h>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <fstream>
#include <map>
#include <algorithm>

#include <boost/filesystem/operations.hpp>

#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

#include <ui_gcs.h>
#include <ui_UAVCondition.h>
#include <ui_WidgetMainMap.h>
#include <ui_PFDWidget_custom.h>
#include <ui_ImageView.h>
#include <ui_AccessPointStats.h>
#include <ui_PictureMsg.h>
#include <ui_AccessPointsMenu.h>

//#include <ui_MissionSelect.h>
//#include <ui_MissionCancel.h>
//#include <ui_MissionProgress.h>
//#include <ui_UavStat.h>
//#include <ui_UavQuestion.h>
//#include <ui_QuadStats.h>
//#include <ui_MissionConfirm.h>

#include <QWidget>
#include <QLabel>  
#include <QStringBuilder>
#include <QStringList>
#include <QTimer>
#include <QThread>
#include <QMainWindow>
#include <QMenuBar>
#include <QMap>
#include <QVector>
#include <QSignalMapper>
#include <QDesktopWidget>
#include <QWaitCondition>
#include <QMetaType>
#include <QWebView>
#include <QSettings>
#include <QCloseEvent>

#define MAX_UAV 100 // the total number of UAV's manageable by our system

namespace rqt_gcs{

  enum UavStatus { null = -1, active, deleted, purged };

  class SimpleGCSHelper;
  class UnansweredQueries;
  class SettingsWidget;
  
  class SimpleGCS : public rqt_gui_cpp::Plugin
  {
  Q_OBJECT

  friend class SimpleGCSHelper;
  friend class UnansweredQueries;
  friend class SettingsWidget;

  public:
      
    struct UAV
    {
        UAV(SimpleControl * sc, UavStatus stat)
        {
            uav = sc;
            status = stat;
        }
                
        ~UAV(){ }
        SimpleControl* uav;
        UavStatus status;
    };
      
    SimpleGCS();
    
    ros::NodeHandle nh;
    //ros::Subscriber sub;
    ros::ServiceServer server;
    lcar_msgs::Door msg;
   
    image_transport::ImageTransport it_stereo{nh};
    void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    
    //methods for publishing object detection paramerter updates
    void publishHitThreshold(double thresh);
    void publishStepSize(int step);
    void publishPadding(int padding);
    void publishScaleFactor(double scale);
    void publishMeanShift(bool on);
    
    std::string GetMissionType(std::string file_name);		 
    lcar_msgs::TargetLocal GetMissionLocal(std::string file_name);
    lcar_msgs::TargetGlobal GetMissionGlobal(std::string file_name);
    
  protected slots:
    virtual void TimedUpdate();
    
    //////////// Buttons
    virtual void ExecutePlay();
    virtual void CancelPlay();
    virtual void ScoutBuilding();
    virtual void StopQuad();
    virtual void ChangeFlightMode(int);
    virtual void DeleteAccessPoint(QWidget*);
    virtual void QuadSelected(int);
    virtual void ArmOrDisarmSelectedQuad();
    virtual void PauseOrResumeScout();
//    virtual void DisarmSelectedQuad();
    virtual void AcceptDoorQuery(QWidget *);
    virtual void RejectDoorQuery(QWidget *);
    virtual void ShowAccessPoints();
    virtual void SettingsTriggered();
    virtual void UnansweredQueriesTriggered();
    
    virtual void AddUav(int);
    virtual void DeleteUav(int, UavStatus);
    virtual void PurgeDeletedUavs();
    virtual void UavConnectionToggled(int, int, bool);

    //SETTINGS RELATED
    virtual void DestroySettingsWidget();
    virtual void ToggleMachineLearningMode(bool);

  private:
    
    void initMap();
    void initMenuBar();
    void initSettings();
    void initHelperThread();
      
    void selectQuad(int);  
    void UpdatePFD();
    void clearImageView();
    void saveImage(std::string, std::string, const cv::Mat&);
    
    void updateQueries();
    void clearQueries();
    void saveUavQueries(SimpleControl *, std::string ap_type);
    void answerQuery(QWidget *, std::string ap_type, bool);
    
    void updateAccessPoints();
    void clearAccessPoints();
    void saveUavAccessPoints(SimpleControl *, std::string ap_type);
          
    int cur_uav;
    int timeCounter;
    int NUM_UAV; //Total number of UAV's in the system
    int num_queries_last;
    int num_access_points_last;
    
    struct ObjectDetectionPublishers
    {
        ros::Publisher pub_hit_thresh;
        ros::Publisher pub_step_size;
        ros::Publisher pub_padding;
        ros::Publisher pub_scale_factor;
        ros::Publisher pub_mean_shift;
    } od_pubs;
    
    struct FloatingWidgets 
    {
        SettingsWidget * settings_ = nullptr;
        UnansweredQueries * unanswered_queries_ = nullptr;
    } widgets_;
    

    QVector<SimpleControl*> active_uavs;
    QMap<int, UAV*> uav_db;
    QMap<int, SimpleControl*> deleted_uavs;

    std::vector<AccessPoint> * accessPointVector;
    std::vector<lcar_msgs::DoorPtr> * pictureQueryVector;

    image_transport::Subscriber sub_stereo;
    Ui::SimpleGCSWidget ui_;
    Ui::centralWidget central_ui_;
    std::vector<Ui::UAVConditionWidget*> uavCondWidgetArr;
    Ui::AccessPointsMenuWidget apmUi_;
    Ui::PictureMsgWidget pmUi_;
     
    QMenuBar * menu_bar_;
    
    QMenu * file_menu;
    QAction * start_uav_act;
    QAction * start_uav_group_act;
    QAction * shutdown_uav_act;
    QAction * shutdown_uav_group_act;
    
    QMenu * view_menu;
    QAction * unanswered_queries_act;
    
    QMenu * tools_menu;
    QAction* settings_act;
    
    QMenu * help_menu;
    QAction * about_act;
    QAction * lcar_bot_act;
    QAction * ros_act;
    QAction * qt_act;
    QAction * opencv_act;
            
    QWidget* widget_main_;
    QWidget* missionProgressWidget_;
    QWidget* uavQuestionWidget_;
    QWidget* uavStatWidget_;
    QWidget* imageViewWidget_;
    QVector<QWidget*> uavListWidgetArr_;
    QWidget* PFDQWidget_;
    QWidget* apmQWidget_;

    std::vector<QWidget*> accessPointWidgets_;
    std::vector<QWidget*> pictureQueryWidgets_;

    QTimer* update_timer;

    QString temp_data;
    QString quad_id;
    QString access_point_temp_data;
    QString access_point_id;
    QSignalMapper* quad_select_mapper;
    QSignalMapper* access_point_mapper;
    QSignalMapper* acceptDoorMapper;
    QSignalMapper* denyDoorMapper;

    QSettings* settings_;
    QString image_root_dir_;
    QString rqt_gcs_dir_;

    QThread t_uav_monitor;
    SimpleGCSHelper * uav_monitor;
    QMutex uav_mutex;
    QWaitCondition num_uav_changed;
  };

  class SimpleGCSHelper : public QObject
  {
  Q_OBJECT

  public:
      SimpleGCSHelper(SimpleGCS *);
      ~SimpleGCSHelper();

  public slots:
      void help();

  signals:
      void addUav(int); // uav_id
      void deleteUav(int, UavStatus);
      void toggleUavConnection(int, int, bool);

  private:
      SimpleGCS * gcs;

      int  binarySearch(int, int, int);
      void parseUavNamespace(std::map<int, int>&);

      void monitorUavNamespace();
      void monitorUavConnections();
      void runUavs();

  };

} // rqt_gcs name space
Q_DECLARE_METATYPE(rqt_gcs::UavStatus);
//Q_DECLARE_METATYPE(cv::Mat);
#endif //rqt_gcs__SimpleGCS_H
