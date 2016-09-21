
/*
 * File:   SimpleGCS.h
 * Author: n8
 *
 * Created on September 20, 2016, 1:28 PM
 */

#ifndef _SIMPLEGCS_H
#define _SIMPLEGCS_H

#include <ros/ros.h>
#include <ros/common.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>

#include <QTimer>
#include <QQueue>
#include <QThread>
#include <QMenuBar>
#include <QSettings>
#include <QSignalMapper>
#include <QWaitCondition>
#include <QMutex>

#include "rqt_gcs/vehicle_list_widget.h"
#include "rqt_gcs/unanswered_queries.h"
#include "rqt_gcs/settings_widget.h"
#include "rqt_gcs/access_points.h"
#include "util/data_types.h"
#include "util/debug.h"
#include "util/image.h"
#include "vehicle/uav_control.h"
#include "lcar_msgs/Door.h"
#include "lcar_msgs/TargetLocal.h"
#include "lcar_msgs/TargetGlobal.h"

#include "ui_PictureMsg.h"
#include "ui_SimpleGCS.h"

namespace rqt_gcs
{

#define MAX_UAV 100 // the total number of UAV's manageable by our system

class SimpleGCSHelper;
class UnansweredQueries;
class SettingsWidget;
class AccessPoints;

class SimpleGCS : public QMainWindow
{
    Q_OBJECT

    friend class SimpleGCSHelper;
    friend class UnansweredQueries;
    friend class SettingsWidget;

public:
    SimpleGCS();
    virtual ~SimpleGCS();
    
    
public slots:
    void OnTimedUpdate();

    //////////// Buttons
    void OnExecutePlay();
    void OnCancelPlay();
    void OnScoutBuilding();
    void OnStopScout();
    void OnChangeFlightMode(int);
    void OnUavSelected(QWidget*);
    void OnArmOrDisarmSelectedUav();
    void OnPauseOrResumeScout();
    void OnAcceptDoorQuery(QWidget *);
    void OnRejectDoorQuery(QWidget *);
    void OnAccessPointsTriggered();
    void OnSettingsTriggered();
    void OnUnansweredQueriesTriggered();
    void OnUpdateCameraFeed();

    void OnAddUav(int);
    void OnDeleteUav(int);
    void OnUAVConnectionToggled(int, int, bool);

    //SETTINGS RELATED
    virtual void OnToggleMachineLearningMode(bool);

    signals:
    void NewCameraFeedFrame();
    
private:
    Ui::SimpleGCS widget;

    void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg);

    //methods for publishing object detection paramerter updates
    void PublishHitThreshold(double thresh);
    void PublishStepSize(int step);
    void PublishPadding(int padding);
    void PublishScaleFactor(double scale);
    void PublishMeanShift(bool on);

    std::string GetMissionType(std::string file_name);
    lcar_msgs::TargetLocal GetMissionLocal(std::string file_name);
    lcar_msgs::TargetGlobal GetMissionGlobal(std::string file_name);

    ros::NodeHandle nh;
    ros::ServiceServer server;
    lcar_msgs::Door msg;
    image_transport::ImageTransport it_stereo{nh};
    QQueue<QPixmap> img_q;
    int img_q_max_size;

    void InitMap();
    void InitMenuBar();
    void InitSettings();
    void InitHelperThread();

    void SelectUav(int);
    void UpdateFlightStateWidgets(); // both the PFD and the text based widget
    void UpdateVehicleWidgets();

    void UpdateQueries();
    void ClearQueries();
    void SaveUavQueries(int uav_id, const std::vector<lcar_msgs::DoorPtr> *queries, const QString ap_type);
    void AnswerQuery(QWidget *, QString ap_type, bool);

    void ToggleScoutButtons(bool visible, QString icon_type = "pause");
    void ToggleArmDisarmButton(bool arm);

    int cur_uav;
    int timeCounter;
    int NUM_UAV; //Total number of UAV's in the system
    int num_queries_last;

    struct ObjectDetectionMessageHandlers // publishers and subscribers
    {
        ros::Publisher pub_hit_thresh;
        ros::Publisher pub_step_size;
        ros::Publisher pub_padding;
        ros::Publisher pub_scale_factor;
        ros::Publisher pub_mean_shift;
        ros::Subscriber sub_od_request;
    } od_handlers;

    struct ObjectDetectionParamaters // object detection parameters
    {
        //defaults
        double hit_thresh = 0; // displayed as a decimal
        int step_size = 16;
        int padding = 8;
        double scale_factor = 1.15; // displayed as a decimal
        bool mean_shift = false;
    } od_params;

    struct FloatingWidgets
    {
        SettingsWidget * settings = nullptr;
        UnansweredQueries * unanswered_queries = nullptr;
        AccessPoints* ap_menu = nullptr;
    } fl_widgets_;

    QVector<UAVControl*> active_uavs;
    QMap<int, UAVControl*> uav_db;

    std::vector<lcar_msgs::DoorPtr> * vec_uav_queries_;

    image_transport::Subscriber sub_stereo;
    QVector<VehicleListWidget*> vec_v_widgets;
    std::vector<QWidget*> query_widgets_; // recieves UAVControl::getRefQuerires

    QTimer* update_timer;
    QString temp_data;

    QSettings* settings;

    QThread t_uav_monitor;
    SimpleGCSHelper * uav_monitor;
    QMutex uav_mutex,
           img_mutex;
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
    void deleteUav(int);
    void toggleUavConnection(int, int, bool);

private:
    SimpleGCS * gcs;

    void parseUavNamespace(std::map<int, int>&);

    void monitorUavNamespace();
    void monitorUavConnections();
    void runUavs();
};

}
#endif /* _SIMPLEGCS_H */
