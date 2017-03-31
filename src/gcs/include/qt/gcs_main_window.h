
/*
 * File:   Simplegcs_main_window.h
 * Author: n8
 *
 * Created on September 20, 2016, 1:28 PM
 */

#ifndef _SIMPLEGCS_H
#define _SIMPLEGCS_H

#include <QTimer>
#include <QMenuBar>
#include <QSettings>
#include <QCloseEvent>

#include "ui_GCSMainWindow.h"

#include "qt/ui_adapter.h"
#include "qt/vehicle_manager.h"
#include "qt/settings_widget.h"
#include "qt/unanswered_queries.h"
#include "qt/vehicle_list_widget.h"
#include "qt/vehicle_init_widget.h"
#include "qt/access_points_container_widget.h"

#include <vehicle/data_types.h>
#include <vehicle/uav_control.h>

#include <lcar_msgs/Query.h>
#include <lcar_msgs/TargetLocal.h>
#include <lcar_msgs/TargetGlobal.h>

namespace gcs
{

class UnansweredQueries;
class SettingsWidget;

class GCSMainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    GCSMainWindow(VehicleManager * vm);
    virtual ~GCSMainWindow();
    
public slots:
    void OnTimedUpdate();

    void OnAddVehicleWidget(int v_id);
    void OnDeleteVehicleWidget(int v_id);
    void OnSetVehicleWidgetEnabled(int v_id, bool enabled);
    
    // Buttons
    void OnExecutePlay();
    void OnCancelPlay();
    void OnScoutBuilding();
    void OnStopScout();
    void OnChangeFlightMode(int);
    void OnVehicleSelected(VehicleWidget *w);
    void OnArmOrDisarmSelectedUav();
    void OnPauseOrResumeScout();
    void OnAcceptDoorQuery(QWidget *);
    void OnRejectDoorQuery(QWidget *);

    
    void OnUpdateCameraFeed(QPixmap img);
    void OnOperatorNotified(QString msg);

    // SETTINGS RELATED
    virtual void OnToggleMachineLearningMode(bool);
    void OnImageRootDirUpdated(QString new_dir);
    
protected:
    void closeEvent(QCloseEvent* event) override;
    
private:
    VehicleWidget* VehicleWidgetAt(int v_type, int index);

    void InitMap();
    void InitMenuBar();
    void InitSettings();
    void SelectVehicleWidgetById(int v_id);
    void UpdateFlightStateWidgets(); // both the PFD and the text based widget
    void UpdateVehicleWidgets();
    void UpdateQueries();
    void ClearQueries();
    void SaveUavQueries(int uav_id, const std::vector<lcar_msgs::QueryPtr> *queries, const QString ap_type);
    void AnswerQuery(QWidget *, QString ap_type, bool);
    /**
     * \brief controls the behavior and visibility of the Scout Buttons
     * @param mode accepts "scout" (hides the play/pause and stop buttons, making scout visible again. effectively stops a mission),
     *                     "play" (hides the scout button, showing the play button and stop), 
     *                     "pause (hides the scout button, showing the pause button and stop)"
     */
    void ToggleScoutButtons(QString mode = "scout");
    void ToggleArmDisarmButton(QString mode = "Arm");
    void CenterFloatingWidget(QWidget * w);
    void OnAccessPointsTriggered();
    void OnSettingsTriggered();
    void OnUnansweredQueriesTriggered();
    void OnAddVehicleTriggered();

    Ui::GCSMainWindow widget;
    VehicleManager * vm;
    QMap<int/*VehicleType*/, QVBoxLayout*> layout_by_v_type;
    QTimer *update_timer;
    QString image_root_dir;
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
    
};

}
#endif /* _SIMPLEGCS_H */
