
/*
 * File:   Simplegcs_main_window.h
 * Author: n8
 *
 * Created on September 20, 2016, 1:28 PM
 */

#ifndef _SIMPLEGCS_H
#define _SIMPLEGCS_H

#include <QCloseEvent>
#include <QMainWindow>
#include <QPointer>

#include <vehicle/data_types.h>
#include <vehicle/uav_control.h>

#include <lcar_msgs/Query.h>
#include <lcar_msgs/TargetLocal.h>
#include <lcar_msgs/TargetGlobal.h>

class QTimer;
class QVBoxLayout;

namespace Ui
{
    class GCSMainWindow;
}

namespace gcs
{

class VehicleManager;
class ImageFeedFilter;
class TrialManager;

class DistractionContainerWidget;
class UnansweredQueries;
class SettingsWidget;
class AccessPointsContainerWidget;
class VehicleInitWidget;
class VehicleWidget;
class TrialWidget;

class GCSMainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    GCSMainWindow(VehicleManager * vm);
    virtual ~GCSMainWindow();
    void setImageFeedVisible(bool visible);
    
    void deleteVehicle(int v_id);
    
public slots:
    void OnTimedUpdate();

    void OnAddVehicleWidget(int v_id);
    void OnDeleteVehicleWidget(int v_id);
    void OnSetVehicleWidgetEnabled(int v_id, bool enabled);
    
    // Buttons
    void OnExecutePlay();
    void OnCancelPlay();
    void StartTrial();
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

    void InitMenuBar();
    void InitSettings();
    
    void hideNonTrialControls();
    
    void SelectVehicleWidgetById(int v_id);
    void UpdateFlightStateWidgets(); // both the PFD and the text based widget
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
    void OnTrialInfoTriggered();
    void OnDistractionWidgetTriggererd();
    
    void connectToTrialManager();
    void connectToSelf();
    void connectToUiAdapter();
    
    void reset();
    
private:
    Ui::GCSMainWindow* _ui;
    
    VehicleManager * vm;
    ImageFeedFilter * _filter;
    TrialManager * _trial_manager;
    
    QMap<int/*VehicleType*/, QVBoxLayout*> layout_by_v_type;
    QTimer *update_timer;
    QTimer *_seconds_timer;
    QString image_root_dir;
    int cur_v_id; // the current selected vehicles id
    int time_counter;
    int num_queries_last;
    
    int _last_wp;
    
    struct FloatingWidgets
    {
        QPointer<SettingsWidget> settings;
        QPointer<UnansweredQueries> unanswered_queries;
        QPointer<AccessPointsContainerWidget> ap_menu;
        QPointer<VehicleInitWidget> vehicle_init;
        QPointer<TrialWidget> user_id;
        QPointer<DistractionContainerWidget> distraction;
    } fl_widgets;
    
};

}
#endif /* _SIMPLEGCS_H */
