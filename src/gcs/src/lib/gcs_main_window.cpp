
#include <stdio.h>
#include <fstream>

#include <QMutex>
#include <QStringBuilder>
#include <QWaitCondition>
#include <QTimer>
#include <QMenuBar>
#include <QSettings>


#include <ros/package.h>

#include "ui_GCSMainWindow.h"

#include <gcs/qt/gcs_main_window.h>
#include <gcs/qt/query_widget.h>
#include <gcs/qt/map_widget_3d.h>
#include <gcs/qt/ui_adapter.h>
#include <gcs/qt/vehicle_manager.h>
#include <gcs/qt/settings_widget.h>
#include <gcs/qt/unanswered_queries.h>
#include <gcs/qt/vehicle_init_widget.h>
#include <gcs/qt/access_points_container_widget.h>
#include <gcs/qt/vehicle_list_widget.h>

#include <gcs/util/debug.h>
#include <gcs/util/settings.h>
#include <gcs/util/flight_modes.h>
#include <gcs/util/image_conversions.h>

#include <vehicle/data_types.h>

namespace gcs
{

GCSMainWindow::GCSMainWindow(VehicleManager *vm):
QMainWindow(nullptr),
widget(new Ui::GCSMainWindow),
cur_v_id(-1),
time_counter(0),
num_queries_last(0),
update_timer(new QTimer(this)),
vm(vm)
{
    widget->setupUi(this);
    widget->map->setVehicleManager(vm);
    
    //todo add these layouts to the GUI
    //layout_by_v_type.insert(VehicleType::ugv, widget->layout_ugvs);
    layout_by_v_type.insert(VehicleType::quad_rotor, widget->layout_quads);
    //layout_by_v_type.insert(VehicleType::octo_rotor, widget->layout_octo);
    //layout_by_v_type.insert(VehicleType::vtol, widget->layout_vtol);

    //setup button logic for the widgets
    connect(widget->btn_exec_play, &QPushButton::clicked, 
            this, &GCSMainWindow::OnExecutePlay);
    connect(widget->btn_scout, &QPushButton::clicked, 
            this, &GCSMainWindow::OnScoutBuilding);
    connect(widget->btn_scout_play_pause, &QPushButton::clicked, 
            this, &GCSMainWindow::OnPauseOrResumeScout);
    connect(widget->btn_scout_stop, &QPushButton::clicked, 
            this, &GCSMainWindow::OnStopScout);
    //this one doesn't like the new signal slot syntax for some reason
    connect(widget->cmbo_box_flight_mode, SIGNAL(currentIndexChanged(int)), 
            this, SLOT(OnChangeFlightMode(int)));
    connect(widget->btn_view_acess_points, &QPushButton::clicked, 
            this, &GCSMainWindow::OnAccessPointsTriggered);
    connect(widget->btn_arm_uav, &QPushButton::clicked, 
            this, &GCSMainWindow::OnArmOrDisarmSelectedUav);
    connect(update_timer, &QTimer::timeout,
            this, &GCSMainWindow::OnTimedUpdate);
    
    UIAdapter *ui_adapter = UIAdapter::Instance();
    connect(ui_adapter, &UIAdapter::NewImageFrame, 
            this, &GCSMainWindow::OnUpdateCameraFeed);
    connect(ui_adapter, &UIAdapter::AddVehicleWidget,
            this, &GCSMainWindow::OnAddVehicleWidget);
    connect(ui_adapter, &UIAdapter::SetMachineLearningMode,
            this, &GCSMainWindow::OnToggleMachineLearningMode);
    connect(ui_adapter, &UIAdapter::SetVehicleWidgetEnabled,
            this, &GCSMainWindow::OnSetVehicleWidgetEnabled);
    connect(ui_adapter, &UIAdapter::NotifyOperator,
            this, &GCSMainWindow::OnOperatorNotified);
    connect(ui_adapter, &UIAdapter::SetImageRootDir,
            this, &GCSMainWindow::OnImageRootDirUpdated);
    
    this->InitMenuBar();
    this->InitSettings();
    this->InitMap();
    this->ToggleScoutButtons("scout");
    
    update_timer->start(0);
}

GCSMainWindow::~GCSMainWindow()
{
}

void GCSMainWindow::OnAddVehicleWidget(int v_id)
{
    int v_type =  vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleWidget *w = new VehicleWidget();
    w->SetId(v_id);
    int v_index = vm->VehicleIndexFromId(v_id);
    w->SetNumber(v_index);
    w->SetName(vm->VehicleStringFromId(v_id) % QChar(' ') % QString::number(v_index));

    int num_widgets = layout_by_v_type[v_type]->count();
    int index = 0;
    while(index < num_widgets && v_id > this->VehicleWidgetAt(v_type, index)->Id())
        index++;
    
    //map this widgets vehicle select button to selecting this widget
    connect(w->Button(), &QPushButton::clicked,
            this, [this, w](){ OnVehicleSelected(w); } );
    
    layout_by_v_type[v_type]->insertWidget(index, w);  
    
    if (cur_v_id == -1) //is this the first vehicle added?
        this->SelectVehicleWidgetById(v_id);
}

void GCSMainWindow::OnDeleteVehicleWidget(int v_id)
{
    QMutex *widget_mutex = vm->GetWidgetMutex();
    QWaitCondition *widget_deleted = vm->GetWaitCondition();
    widget_mutex->lock();
    
    int v_type = vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleWidget *w = this->VehicleWidgetAt(v_type, v_id - v_type);
    delete w;
    
    int num_vehicle = vm->NumVehiclesByType(v_type);
    if(num_vehicle == 0)
    {
        this->ClearQueries();
        this->ToggleScoutButtons("scout");  // reset scout buttons
        this->ToggleArmDisarmButton("Arm"); // set [dis]arm button to arm

        widget->image_frame->setPixmap(QPixmap::fromImage(QImage()));
        widget->lbl_cur_uav->setText("NO UAVS");
        cur_v_id = -1;
    }
    if(v_id == cur_v_id) //current vehicle was just deleted
    {
        if(num_vehicle - 1 > 0)
        {
            if(vm->VehicleIndexFromId(v_id) > 0)
                this->SelectVehicleWidgetById(cur_v_id - 1);
            else
                cur_v_id = -1;
        }
    }
    
    //todo handle cases where num_vehicle != 0
    
    widget_deleted->wakeAll();
    widget_mutex->unlock();
}

void GCSMainWindow::OnSetVehicleWidgetEnabled(int v_id, bool enabled)
{
    int v_type = vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    int index = v_id - v_type;
    
    VehicleWidget * w = this->VehicleWidgetAt(v_type, index);
    if(w->IsButtonEnabled() != enabled)
        w->SetButtonEnabled(enabled);
}

void GCSMainWindow::SaveUavQueries(int uav_id, const std::vector<lcar_msgs::QueryPtr> *queries, const QString ap_type)
{
    QString path = image_root_dir % "/queries/unanswered/" % ap_type;

    int num_images = image_conversions::numImagesInDir(path);

    for(int i = 0; i < queries->size(); i++, num_images += 2)
    {
        lcar_msgs::QueryPtr query = queries->at(i);

        sensor_msgs::Image ros_image = query->img;
        QImage image = image_conversions::rosImgToQimg(ros_image);

        QString file = "img_" % QString::number(num_images) % ".jpg";
        if(!image_conversions::saveImage(path, file, image))
            qCDebug(lcar_bot) << "error saving uav query\n";

        ros_image = query->img_framed;
        image = image_conversions::rosImgToQimg(ros_image);

        file = "img_" %  QString::number(num_images+1) % ".jpg";
        if(!image_conversions::saveImage(path, file, image))
            qCDebug(lcar_bot) << "error saving uav query\n";
    }
}

//void GCS::OnUAVConnectionToggled(int index, int uav_id, bool toggle)
//{
//    if(index >= active_uavs.size() || active_uavs[index]->id != uav_id)
//        return;
//
//    this->VehicleWidgetAt(VehicleType::quad_rotor, index)->ToggleButton(toggle);
//}

//Timed update of GCS gui
void GCSMainWindow::OnTimedUpdate()
{
    if(cur_v_id == -1)
        return;

    if(time_counter++ >= 100)
    {
        this->UpdateQueries();
        time_counter = 0;
    }

    this->UpdateFlightStateWidgets();
    this->UpdateVehicleWidgets();
    
    if(vm->IsArmed(cur_v_id))
        this->ToggleArmDisarmButton("disarm");
    else 
        this->ToggleArmDisarmButton("arm");
    
    MissionMode m = vm->GetMissionMode(cur_v_id);
    if (m == MissionMode::active)
        this->ToggleScoutButtons("pause");
    else if(m == MissionMode::paused)
        this->ToggleScoutButtons("play");
    else if(m == MissionMode::stopped || m == MissionMode::invalid)
        this->ToggleScoutButtons("scout");
                
}

void GCSMainWindow::ClearQueries()
{
    for(int i = widget->layout_queries->count() - 1; i >= 0; i--)
        delete widget->layout_queries->itemAt(i)->widget();
    
    num_queries_last = 0;
}


void GCSMainWindow::UpdateQueries()
{
    if(cur_v_id == -1 || !widget->frame_queries_cntnr->isVisible())
        return;

    std::vector<lcar_msgs::QueryPtr> *queries = vm->GetUAVDoorQueries(cur_v_id);
    int pqv_size = queries->size();

    for(int i = num_queries_last; i < pqv_size; i++)
    {
        //retrieve Query msg for door image
        lcar_msgs::QueryPtr doorQuery = queries->at(i);

        QPixmap image = image_conversions::rosImgToQpixmap(doorQuery->img_framed);

        //create the widget
        QueryWidget * qw = new QueryWidget();
        qw->SetImage(image);

        connect(qw->YesButton(), &QPushButton::clicked,
                this, [this, qw](){ OnAcceptDoorQuery(qw); });
        connect(qw->RejectButton(), &QPushButton::clicked,
                this, [this, qw](){ OnRejectDoorQuery(qw); });
                
        //add to user interface
        widget->layout_queries->addWidget(qw);
    }

    num_queries_last = pqv_size;
}

void GCSMainWindow::AnswerQuery(QWidget * qw, QString ap_type, bool accepted)
{
    int index = widget->layout_queries->indexOf(qw);
    std::vector<lcar_msgs::QueryPtr> * vec_uav_queries_ptr = vm->GetUAVDoorQueries(cur_v_id);
    lcar_msgs::QueryPtr door = vec_uav_queries_ptr->at(index);

    QString path = image_root_dir % "/queries";
    QString file;
    if(accepted)
        path.append("/accepted/" % ap_type);
    else
         path.append("/rejected/" % ap_type);
        
    int num_images = image_conversions::numImagesInDir(path);
    file.append("img_" % QString::number(num_images) % ".jpg");
    image_conversions::saveImage(path, file, door->img);

    vec_uav_queries_ptr->erase(vec_uav_queries_ptr->begin() + index);
    delete qw;

    door->is_accepted = accepted;
    num_queries_last--;
}

void GCSMainWindow::OnAcceptDoorQuery(QWidget *qw)
{
    AnswerQuery(qw, "door", true);
}

void GCSMainWindow::OnRejectDoorQuery(QWidget * qw)
{
    AnswerQuery(qw, "door", false);
}

void GCSMainWindow::OnExecutePlay()
{
    if(vm->NumTotalVehicles() == 0)
        return;

    QString play = widget->cmbo_box_play_book->currentText();
    
    int number;
    sscanf(play.toStdString().c_str(), "%d", &number);
    
    emit UIAdapter::Instance()->ExecutePlay(number);

    ROS_INFO_STREAM("Play " << play.toStdString() << " initiated");
}

void GCSMainWindow::OnCancelPlay()
{
    if(cur_v_id == -1)
        return;

    emit UIAdapter::Instance()->CancelPlay();

    this->ToggleScoutButtons("scout");
}

void GCSMainWindow::OnScoutBuilding()
{
    if(cur_v_id == -1)
        return;

    QString building = widget->cmbo_box_buildings->currentText();
    
    int number;
    char b[16];
    sscanf(building.toStdString().c_str(), "%s %d", b, &number);
    qCDebug(lcar_bot) << "scout building: " << number;
    
    emit UIAdapter::Instance()->ScoutBuilding(cur_v_id, number);

    ROS_INFO_STREAM("Scouting " << building.toStdString());
}

void GCSMainWindow::OnPauseOrResumeScout()
{
    if(cur_v_id == -1)
        return;

    if(vm->GetMissionMode(cur_v_id) == MissionMode::active)
    {
        emit UIAdapter::Instance()->PauseMission(cur_v_id);
        widget->btn_scout_play_pause->setIcon(QIcon(":/images/icons/play.png"));
    }
    else
    {
        emit UIAdapter::Instance()->ResumeMission(cur_v_id);
        widget->btn_scout_play_pause->setIcon(QIcon(":/images/icons/pause.png"));
    }
}

void GCSMainWindow::OnStopScout()
{
    if(cur_v_id == -1)
        return;

    emit UIAdapter::Instance()->CancelMission(cur_v_id);
    this->ToggleScoutButtons("scout");
}

void GCSMainWindow::OnChangeFlightMode(int index)
{
    if(cur_v_id == -1)
        return;
    
    if(index == 0)
    {
        ROS_INFO_STREAM("Quadrotor Stablized");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::stabilized);
    }
    else if(index == 1)
    {
        ROS_INFO_STREAM("Quadrotor Loiter");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::loiter);
    }
    else if(index == 2)
    {
        ROS_INFO_STREAM("Quadrotor Land");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::land);
    }
    else if(index == 3)
    {
        ROS_INFO_STREAM("Altitude Hold");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::altitude_control);
    }
    else if( index == 4)
    {
        ROS_INFO_STREAM("Position Hold");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::position_control);
    }
    else if(index == 5)
    {
        ROS_INFO_STREAM("Quadrotor Return-To-Launch");
        emit UIAdapter::Instance()->SetRTL(cur_v_id);
    }
    else if(index == 6)
    {
        ROS_INFO_STREAM("Quadrotor Auto");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::mode_auto);
    }
    else if(index == 7)
    {
        ROS_INFO_STREAM("Quadrotor Offboard");
        emit UIAdapter::Instance()->SetMode(cur_v_id, FlightModes::offboard);
    }

    if(vm->GetMissionMode(cur_v_id) != MissionMode::stopped)
        emit UIAdapter::Instance()->CancelMission(cur_v_id);
}

void GCSMainWindow::ToggleScoutButtons(QString mode)
{ // icon_type should be "play" or "pause"
    mode = mode.toLower();
    bool scout = (mode == "scout"); 
    Q_ASSERT(scout || mode == "play" || mode == "pause");
    
    widget->btn_scout->setVisible(scout);
    widget->btn_scout_play_pause->setVisible(!scout);
    if(!scout)
        widget->btn_scout_play_pause->setIcon(QIcon(":/images/icons/" % mode % ".png"));
    widget->btn_scout_stop->setVisible(!scout);
}

// slot gets called when user click on a uav button
void GCSMainWindow::OnVehicleSelected(VehicleWidget *w)
{
//    this->SelectUav(layout_by_v_type[VehicleType::quad_rotor]->indexOf(w));
    this->SelectVehicleWidgetById(w->Id());
}


void GCSMainWindow::SelectVehicleWidgetById(int v_id)
{
    int v_type = vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    if(cur_v_id == v_id)
        return;
    
    cur_v_id = v_id;
    
    QString vehicle_type = vm->VehicleStringFromId(v_id);
    widget->lbl_cur_uav->setText(vehicle_type % QString::number(v_id - v_type));
    
    QString topic("/V" % QString::number(v_id) % "/stereo_cam/left/image_rect");
    vm->SubscribeToImageTopic(topic);
    
    widget->image_frame->setPixmap(QPixmap::fromImage(QImage()));
    this->ClearQueries(); // new uav selected, so make room for its queries
    if(fl_widgets.ap_menu != nullptr)
        fl_widgets.ap_menu->SetUAVAccessPointsAndId(vm->GetUAVAccessPoints(v_id), v_id - v_type);
    
    //todo update gui buttons according to current vehicles mission status 
    MissionMode m = vm->GetMissionMode(cur_v_id);
    if(m == MissionMode::stopped)
        this->ToggleScoutButtons("scout");
    else if(m == MissionMode::active)
        this->ToggleScoutButtons("pause");
    else
        this->ToggleScoutButtons("play");

    if(vm->IsArmed(cur_v_id))
        this->ToggleArmDisarmButton("disarm");
    else
        this->ToggleArmDisarmButton("arm");
}

void GCSMainWindow::UpdateFlightStateWidgets()
{
    if(cur_v_id == -1)
        return;

    FlightState flight_state = vm->GetFlightState(cur_v_id);
    
    // PFD
    widget->pfd->setRoll(flight_state.roll*180);
    widget->pfd->setPitch(flight_state.pitch*90);
    widget->pfd->setHeading(flight_state.heading);
    widget->pfd->setAirspeed(flight_state.air_speed);
    widget->pfd->setAltitude(flight_state.altitude);
    widget->pfd->setClimbRate(flight_state.vertical_speed);
    widget->pfd->update();
    
    QString temp_data;
    //text based widget
    temp_data.setNum(flight_state.yaw, 'f', 2);
    widget->lbl_yaw_val->setText(temp_data);
    
    temp_data.setNum(flight_state.roll, 'f', 2);
    widget->lbl_roll_val->setText(temp_data);
    
    temp_data.setNum(flight_state.pitch, 'f', 2);
    widget->lbl_pitch_val->setText(temp_data);
    
    temp_data.setNum(flight_state.ground_speed, 'f', 2).append(" m/s");
    widget->lbl_gnd_spd_val->setText(temp_data);
    
    temp_data.setNum(vm->GetDistanceToWP(cur_v_id)).append(" m");
    widget->lbl_dist_wp_val->setText(temp_data);
    
    
    StatePtr state = vm->GetState(cur_v_id);
    temp_data.setNum(state->battery * 100);
    widget->pgs_bar_battery->setValue(temp_data.toInt());
    
    int v_index = vm->VehicleIndexFromId(cur_v_id);
    temp_data = "UAV " % QString::number(v_index);
    widget->lbl_cur_uav->setText(temp_data);
    
    widget->pgs_bar_mission->setValue(state->mission_progress * 100);
}

void GCSMainWindow::OnArmOrDisarmSelectedUav()
{
    if(cur_v_id == -1)
        return;

    bool armed = vm->IsArmed(cur_v_id);
    emit UIAdapter::Instance()->Arm(cur_v_id, !armed);
}

void GCSMainWindow::ToggleArmDisarmButton(QString mode)
{
    QString temp_mode = mode.toLower();
    temp_mode = temp_mode[0].toUpper() % temp_mode.mid(1);
    Q_ASSERT(temp_mode == "Arm" || temp_mode == "Disarm");
    widget->btn_arm_uav->setText(temp_mode);
}

void GCSMainWindow::CenterFloatingWidget(QWidget* w)
{
    QRect window = this->window()->geometry();
    int x = (this->width() / 2) - (w->width() / 2);
    int y = (this->height() / 2) - (w->height() / 2);
    w->move(window.x() + x, window.y() + y);
    w->setVisible(true);
}

void GCSMainWindow::OnAccessPointsTriggered()
{
    if(fl_widgets.ap_menu == nullptr)
    {
        fl_widgets.ap_menu = new AccessPointsContainerWidget(image_root_dir);

        this->CenterFloatingWidget(fl_widgets.ap_menu);

        connect(fl_widgets.ap_menu, &AccessPointsContainerWidget::destroyed,
                this, [=](){ fl_widgets.ap_menu = nullptr; });

        if(vm->NumVehiclesByType(VehicleType::quad_rotor) > 0)
        {
            int v_type = vm->VehicleTypeFromId(cur_v_id);
            fl_widgets.ap_menu->SetUAVAccessPointsAndId(vm->GetUAVAccessPoints(cur_v_id), cur_v_id - v_type);
        }
        else
            fl_widgets.ap_menu->SetUAVAccessPointsAndId(nullptr, -1);
    }
    else
        fl_widgets.ap_menu->showNormal();
        fl_widgets.ap_menu->activateWindow();
}

void GCSMainWindow::OnSettingsTriggered()
{
    if(fl_widgets.settings == nullptr)
    {
        fl_widgets.settings = new SettingsWidget();

        this->CenterFloatingWidget(fl_widgets.settings);

        connect(fl_widgets.settings, &SettingsWidget::destroyed,
                this, [=](){ fl_widgets.settings = nullptr; });
                
        connect(fl_widgets.settings, &SettingsWidget::localCoordinatesUpdated,
                vm, &VehicleManager::OnLocalCoordinatesUpdated);
    }
    else
    {
        fl_widgets.settings->showNormal();
        fl_widgets.settings->activateWindow();
    }
}

void GCSMainWindow::OnUnansweredQueriesTriggered()
{
    if(fl_widgets.unanswered_queries == nullptr)
    {
        fl_widgets.unanswered_queries = new UnansweredQueries(image_root_dir);

        this->CenterFloatingWidget(fl_widgets.unanswered_queries);

        connect(fl_widgets.unanswered_queries, &UnansweredQueries::destroyed,
                this, [=](){ fl_widgets.unanswered_queries = nullptr; });
    }
    else
    {
        fl_widgets.unanswered_queries->showNormal();
        fl_widgets.unanswered_queries->activateWindow();
    }
}

void GCSMainWindow::OnAddVehicleTriggered()
{
    if(fl_widgets.vehicle_init == nullptr)
    {
        fl_widgets.vehicle_init = new VehicleInitWidget(vm);

        this->CenterFloatingWidget(fl_widgets.vehicle_init);

        connect(fl_widgets.vehicle_init, &VehicleInitWidget::destroyed,
                this, [=](){ fl_widgets.vehicle_init = nullptr; });
    }
    else
    {
        fl_widgets.vehicle_init->showNormal();
        fl_widgets.vehicle_init->activateWindow();
    }
}

void GCSMainWindow::InitMap()
{
}

void GCSMainWindow::InitMenuBar()
{
    QMenuBar *menu_bar = this->menuBar();
    
    QMenu *file_menu = menu_bar->addMenu("File");
    QAction *add_vehicle_act = file_menu->addAction("Add Vehicle(s)");
    connect(add_vehicle_act, &QAction::triggered,
            this, &GCSMainWindow::OnAddVehicleTriggered);

    //view menu
    QMenu *view_menu = menu_bar->addMenu("View");
    QAction *unanswered_queries_act = view_menu->addAction("Unanswered Queries");
    connect(unanswered_queries_act, &QAction::triggered,
            this, &GCSMainWindow::OnUnansweredQueriesTriggered);

    //tools menu
    QMenu *tools_menu = menu_bar->addMenu("Tools");
    QAction *settings_act = tools_menu->addAction("Settings");
    connect(settings_act, &QAction::triggered,
            this, &GCSMainWindow::OnSettingsTriggered);

    //help menu
    QMenu *help_menu = menu_bar->addMenu("Help");
    QAction *lcar_bot_act = help_menu->addAction("Learning Classifying And Recognizing Bot (LCAR-Bot)");
    QAction *ros_act      = help_menu->addAction("Robot Operating System (ROS)");
    QAction *opencv_act   = help_menu->addAction("Open Computer Vision (OpenCV)");
    QAction *qt_act       = help_menu->addAction("Qt");
    QAction *about_act    = help_menu->addSection("About");
}

// SettingsWidget and QSettings related stuff
void GCSMainWindow::InitSettings()
{
    //initialize settings and uav queries display
    Settings settings;
    
    bool ml_on = (settings.GetMachineLearningType() == settings.val_machine_learning_online);
    this->OnToggleMachineLearningMode(ml_on);

    image_root_dir = settings.GetImagesRootDir();
}

void GCSMainWindow::OnToggleMachineLearningMode(bool toggle)
{
    widget->frame_queries_cntnr->setVisible(toggle);
    widget->frame_queries_cntnr->setEnabled(toggle);
}

void GCSMainWindow::OnImageRootDirUpdated(QString new_dir)
{
    image_root_dir = new_dir;
}

void GCSMainWindow::UpdateVehicleWidgets()
{
    //todo unhard code this v_type
    int v_type = VehicleType::quad_rotor;
    int num_widgets = layout_by_v_type[v_type]->count();
    for(int i = 0; i < num_widgets; i++)
    {
        VehicleWidget *widget = this->VehicleWidgetAt(v_type, i);
        StatePtr ptr = vm->GetState(v_type + i);
        widget->SetBattery(ptr->battery);
        widget->SetCondition(ptr->mode.c_str());
    }
}

void GCSMainWindow::OnUpdateCameraFeed(QPixmap img)
{
    int w = widget->image_frame->width();
    int h = widget->image_frame->height();
    widget->image_frame->setPixmap(img.scaled(w, h, Qt::KeepAspectRatio));
}

void GCSMainWindow::OnOperatorNotified(QString msg)
{
    widget->status_bar->showMessage(msg, 15000);
}

void GCSMainWindow::closeEvent(QCloseEvent* event)
{    
    cur_v_id = -1;
    
    // this wouldn't work in the destructor
    if(fl_widgets.ap_menu != nullptr) 
        delete fl_widgets.ap_menu;
    
    if(fl_widgets.settings != nullptr)
        delete fl_widgets.settings;
    
    if(fl_widgets.unanswered_queries != nullptr)
        delete fl_widgets.unanswered_queries;
   
    if(fl_widgets.vehicle_init != nullptr)
        delete fl_widgets.vehicle_init;
    
    event->accept();
}

VehicleWidget* GCSMainWindow::VehicleWidgetAt(int v_type, int index)
{
    Q_ASSERT(VehicleType::invalid_low < v_type && v_type < VehicleType::invalid_high);
    Q_ASSERT(index < layout_by_v_type[v_type]->count());
    if(index >= layout_by_v_type[v_type]->count())
        return nullptr;

    return static_cast<VehicleWidget*>(layout_by_v_type[v_type]->itemAt(index)->widget());
}

} // namespace
