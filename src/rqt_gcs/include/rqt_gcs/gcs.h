
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
#include "rqt_gcs/ui_adapter.h"

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
    GCS(UIAdapter * uia);
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
    void OnUavSelected(VehicleWidget *w);
    void OnArmOrDisarmSelectedUav();
    void OnPauseOrResumeScout();
    void OnAcceptDoorQuery(QWidget *);
    void OnRejectDoorQuery(QWidget *);
    void OnAccessPointsTriggered();
    void OnSettingsTriggered();
    void OnUnansweredQueriesTriggered();
    void OnAddVehicleTriggered();
    void OnUpdateCameraFeed(QPixmap img);

//    void OnAddUav(int);
//    void OnDeleteUav(int);
//    void OnUAVConnectionToggled(int, int, bool);

    //SETTINGS RELATED
    virtual void OnToggleMachineLearningMode(bool);
    
protected:
    void closeEvent(QCloseEvent* event) override;
    
private:
    VehicleWidget* VehicleWidgetAt(int v_type, int index);

    std::string GetMissionType(std::string file_name);
    lcar_msgs::TargetLocal GetMissionLocal(std::string file_name);
    lcar_msgs::TargetGlobal GetMissionGlobal(std::string file_name);

    void InitMap();
    void InitMenuBar();
    void InitSettings();
//    void InitHelperThread();

    // new
    void SelectVehicleWidgetById(int v_id);
    //old
//    void SelectUav(int);
    
    void UpdateFlightStateWidgets(); // both the PFD and the text based widget
    void UpdateVehicleWidgets();

    void UpdateQueries();
    void ClearQueries();
    void SaveUavQueries(int uav_id, const std::vector<lcar_msgs::QueryPtr> *queries, const QString ap_type);
    void AnswerQuery(QWidget *, QString ap_type, bool);

    void ToggleScoutButtons(bool visible, QString icon_type = "pause");
    void ToggleArmDisarmButton(bool arm);
    
    Ui::GCS widget;
    UIAdapter *ui_adapter;
    
    int cur_v_id; // the current selected vehicles id
    int time_counter;
    int num_queries_last;

    struct FloatingWidgets
    {
        SettingsWidget *settings = nullptr;
        UnansweredQueries *unanswered_queries = nullptr;
        AccessPointsContainerWidget *ap_menu = nullptr;
        VehicleInitWidget *vehicle_init = nullptr;
    } fl_widgets;

    QMap<int, UAVControl*> uav_db;
    VehicleManager * vm;
    
    QMap<int/*VehicleType*/, QVBoxLayout*> layout_by_v_type;

    image_transport::Subscriber sub_stereo;

    QTimer *update_timer;

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
