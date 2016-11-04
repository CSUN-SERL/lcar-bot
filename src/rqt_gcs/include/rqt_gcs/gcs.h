
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
#include <QCloseEvent>
#include <QMutex>

#include "rqt_gcs/vehicle_list_widget.h"
#include "rqt_gcs/unanswered_queries.h"
#include "rqt_gcs/settings_widget.h"
#include "rqt_gcs/access_points_container_widget.h"
#include "rqt_gcs/vehicle_init_widget.h"
#include "rqt_gcs/vehicle_manager.h"

#include "util/data_types.h"
#include "util/debug.h"
#include "util/image.h"

#include "vehicle/uav_control.h"

#include "lcar_msgs/Query.h"
#include "lcar_msgs/TargetLocal.h"
#include "lcar_msgs/TargetGlobal.h"

#include "ui_GCS.h"

namespace rqt_gcs
{

#define MAX_UAV 100 // the total number of UAV's manageable by our system

class GCSHelperThread;
class UnansweredQueries;
class SettingsWidget;

class GCS : public QMainWindow
{
    Q_OBJECT

    friend class GCSHelperThread;
    friend class UnansweredQueries;
    friend class SettingsWidget;

public:
    GCS();
    virtual ~GCS();
    
    
public slots:
    void OnTimedUpdate();

    void OnAddVehicleWidget(int v_id);
    void OnDeleteVehicleWidget(int v_id);
    
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
    void OnAddVehicleTriggered();
    void OnUpdateCameraFeed(const QPixmap& img);

    void OnAddUav(int);
    void OnDeleteUav(int);
    void OnUAVConnectionToggled(int, int, bool);

    //SETTINGS RELATED
    virtual void OnToggleMachineLearningMode(bool);
    
    signals:
        void NewCameraFeedFrame(const QPixmap& img);
    
protected:
    void closeEvent(QCloseEvent* event) override;
    
private:

//    void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg);
    VehicleWidget* VehicleWidgetAt(int v_type, int index);
    
    //methods for publishing object detection paramerter updates
    void PublishHitThreshold(double thresh);
    void PublishStepSize(int step);
    void PublishPadding(int padding);
    void PublishScaleFactor(double scale);
    void PublishMeanShift(bool on);

    std::string GetMissionType(std::string file_name);
    lcar_msgs::TargetLocal GetMissionLocal(std::string file_name);
    lcar_msgs::TargetGlobal GetMissionGlobal(std::string file_name);

    void InitMap();
    void InitMenuBar();
    void InitSettings();
    void InitHelperThread();

    void SelectVehicleWidget(int v_type, int index);
    void SelectUav(int);
    void UpdateFlightStateWidgets(); // both the PFD and the text based widget
    void UpdateVehicleWidgets();

    void UpdateQueries();
    void ClearQueries();
    void SaveUavQueries(int uav_id, const std::vector<lcar_msgs::QueryPtr> *queries, const QString ap_type);
    void AnswerQuery(QWidget *, QString ap_type, bool);

    void ToggleScoutButtons(bool visible, QString icon_type = "pause");
    void ToggleArmDisarmButton(bool arm);

    void AdvertiseObjectDetection();
    
    Ui::GCS widget;
    
    //ros::NodeHandle nh;
    ros::ServiceServer server;
    lcar_msgs::Query msg;
//    image_transport::ImageTransport it_stereo{nh};
    
    int cur_vehicle;
    int time_counter;
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
        SettingsWidget *settings = nullptr;
        UnansweredQueries *unanswered_queries = nullptr;
        AccessPointsContainerWidget *ap_menu = nullptr;
        VehicleInitWidget *vehicle_init = nullptr;
    } fl_widgets;

    QVector<UAVControl*> active_uavs;
    QMap<int, UAVControl*> uav_db;
    VehicleManager * vm;
    
    QMap<int/*VehicleType*/, QVBoxLayout*> layout_by_v_type;

    std::vector<lcar_msgs::QueryPtr> *vec_uav_queries_ptr;

    image_transport::Subscriber sub_stereo;

    QTimer *update_timer;
    QString temp_data;

    QSettings *settings;

    GCSHelperThread *thread_uav_monitor;
    QMutex uav_mutex;
    QWaitCondition num_uav_changed;
    
};

class GCSHelperThread : public QThread
{
  Q_OBJECT

public:
    GCSHelperThread(GCS *);
    ~GCSHelperThread();
    virtual void Stop();
    
signals:
    void AddUav(int); // uav_id
    void DeleteUav(int);
    void ToggleUavConnection(int, int, bool);
    
private:
    GCS * gcs;
    
    void ParseUavNamespace(std::map<int, int>&);
    void MonitorUavNamespace();
    void MonitorUavConnections();
    void RunUavs();
    
    virtual void run() override;
};

}
#endif /* _SIMPLEGCS_H */
