
#include <stdio.h>
#include <fstream>

#include <QProcessEnvironment>
#include <QDesktopWidget>
#include <QMainWindow>
#include <QStringBuilder>
#include <QWaitCondition>
#include <QMutex>
#include <QMetaType>

#include "gcs/gcs_main_window.h"
#include "gcs/query_widget.h"
#include "gcs/vehicle_init_widget.h"
#include "util/image.h"
#include "util/debug.h"
#include "util/strings.h"
#include "util/data_types.h"

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

namespace gcs
{

GCSMainWindow::GCSMainWindow(VehicleManager *vm):
    cur_v_id(-1),
    time_counter(0),
    num_queries_last(0),
    update_timer(new QTimer(this)),
    vm(vm)
{
    widget.setupUi(this);
    
    //todo add these layouts to the GUI
    //layout_by_v_type.insert(VehicleType::ugv, widget.layout_ugvs);
    layout_by_v_type.insert(VehicleType::quad_rotor, widget.layout_quads);
    //layout_by_v_type.insert(VehicleType::octo_rotor, widget.layout_octo);
    //layout_by_v_type.insert(VehicleType::vtol, widget.layout_vtol);

    //setup button logic for the widgets
    connect(widget.btn_exec_play, &QPushButton::clicked, 
            this, &GCSMainWindow::OnExecutePlay);
    connect(widget.btn_scout, &QPushButton::clicked, 
            this, &GCSMainWindow::OnScoutBuilding);
    connect(widget.btn_scout_play_pause, &QPushButton::clicked, 
            this, &GCSMainWindow::OnPauseOrResumeScout);
    connect(widget.btn_scout_stop, &QPushButton::clicked, 
            this, &GCSMainWindow::OnStopScout);
    //this one doesn't like the new signal slot syntax for some reason
    connect(widget.cmbo_box_flight_mode, SIGNAL(currentIndexChanged(int)), 
            this, SLOT(OnChangeFlightMode(int)));
    connect(widget.btn_view_acess_points, &QPushButton::clicked, 
            this, &GCSMainWindow::OnAccessPointsTriggered);
    connect(widget.btn_arm_uav, &QPushButton::clicked, 
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

    this->InitMenuBar();
    this->InitSettings();
//    this->InitHelperThread();
    this->InitMap();
    this->ToggleScoutButtons(true);
    
    update_timer->start(0);
}

GCSMainWindow::~GCSMainWindow()
{
    ros::shutdown();
}

//void GCS::OnAddUav(int uav_id)
//{
//    uav_mutex.lock();
//
//    UAVControl * uav = new UAVControl(uav_id);
//
//    //todo loop and get num_image for each acess point type;
//    //create separate accepted and rejected counts for each ap_type
//    QString path = img::image_root_dir_ + "/queries/accepted/door/uav_" + QString::number(uav_id);
//    uav->accepted_images = img::numImagesInDir(path);
//
//    path = img::image_root_dir_ % "/queries/unanswered/door/uav_" % QString::number(uav_id);
//    uav->rejected_images = img::numImagesInDir(path);
//
//    int index = 0;
//    while(index < NUM_UAV && uav_id > active_uavs[index]->id)
//        index++;
//
//    uav_db.insert(uav_id, uav);
//    active_uavs.insert(active_uavs.begin() + index, uav);
//
//    VehicleWidget *uav_widget = new VehicleWidget();
//    uav_widget->SetNumber(uav_id);
//    uav_widget->SetName("UAV " % QString::number(uav_id));
//
//    //map this widgets vehicle select button to selecting this widget
//    connect(uav_widget->Button(), &QPushButton::clicked,
//            this, [=](){ OnUavSelected(uav_widget); } );
//
//    widget.layout_quads->insertWidget(index, uav_widget);
//
//    NUM_UAV++;
//
//    if(NUM_UAV == 1)
//        SelectUav(0);
//
//    num_uav_changed.wakeAll();
//    uav_mutex.unlock();
//}

void GCSMainWindow::OnAddVehicleWidget(int v_id)
{
    int v_type =  vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleWidget *w = new VehicleWidget();
    w->SetId(v_id);
    int v_index = vm->VehicleIndexFromId(v_id);
    w->SetNumber(v_index);
    w->SetName(vm->VehicleStringFromId(v_id) % " " % QString::number(v_index));

    // - 1 accounts for the vehicle corresponding to this widget
    // that was added before the GUI was told to create this widget.
    int num_vehicles = vm->NumVehiclesByType(v_type) - 1;
    int index = 0;
    while(index < num_vehicles && v_id > this->VehicleWidgetAt(v_type, index)->Id())
        index++;
    
    //map this widgets vehicle select button to selecting this widget
    connect(w->Button(), &QPushButton::clicked,
            this, [=](){ OnUavSelected(w); } );
    
    layout_by_v_type[v_type]->insertWidget(index, w);  
    if(vm->NumVehiclesByType(v_type) == 1)
        this->SelectVehicleWidgetById(w->Id());
}

void GCSMainWindow::OnDeleteVehicleWidget(int v_id)
{
    QMutex *widget_mutex = vm->GetWidgetMutex();
    QWaitCondition *widget_deleted = vm->GetWaitCondition();
    widget_mutex->lock();
    
    int v_type = vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleWidget * w = this->VehicleWidgetAt(v_type, v_id - v_type);
    delete w;
    
    int num_vehicle = vm->NumVehiclesByType(v_type);
    if(num_vehicle == 0)
    {
        this->ClearQueries();
        this->ToggleScoutButtons(true);  // reset scout buttons
        this->ToggleArmDisarmButton(false); //set [dis]arm button to arm

        widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
        widget.lbl_cur_uav->setText("NO UAVS");
        cur_v_id = -1;
    }
    if(v_id == cur_v_id) // current vehicle was just deleted
    {
        if(vm->VehicleIndexFromId(v_id) > 0)
            this->SelectVehicleWidgetById(cur_v_id - 1);
        else
            cur_v_id = -1;
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

//void GCS::OnDeleteUav(int index)
//{
//    uav_mutex.lock();
//    
//    VehicleWidget * vw = this->VehicleWidgetAt(VehicleType::quad_rotor, index);
//    UAVControl * uav = active_uavs[index];
//    
//    this->SaveUavQueries(uav->id, uav->GetDoorQueries(), "door");
//
//    // dont delete uav_db[index] as it points to same UAVControl object as active_uavs
//    delete uav;
//    delete vw; // removes this widget from the layout
//
//    active_uavs.erase(active_uavs.begin() + index);
//    uav_db.erase(uav_db.begin() + index);
//
//    NUM_UAV--;
//
//    if(NUM_UAV == 0) // no more uav's
//    {
//        if(fl_widgets.ap_menu != nullptr)
//            fl_widgets.ap_menu->SetUAV(nullptr);
//
//        this->ClearQueries();
//        this->ToggleScoutButtons(true);  // reset scout buttons
//        this->ToggleArmDisarmButton(false); //set [dis]arm button to arm
//
//        widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
//        widget.lbl_cur_uav->setText("NO UAVS");
//        cur_v_id = -1;
//    }
//    //deleted uav was in front of cur_uav, causing NUM_UAV to shrink. cur_uav >= NUM_UAV
//    else if(NUM_UAV <= cur_vehicle) // subsequent calls to TimedUpdate will be invalid at this point
//    {
//        this->SelectUav(NUM_UAV - 1);
//    }
//    else if(cur_vehicle == index) // deleted current uav
//    {
//        if(cur_vehicle > 0)
//            this->SelectUav(cur_vehicle - 1);
//        else
//            this->SelectUav(0);
//    }
//
//    num_uav_changed.wakeAll();
//    uav_mutex.unlock();
//}

void GCSMainWindow::SaveUavQueries(int uav_id, const std::vector<lcar_msgs::QueryPtr> *queries, const QString ap_type)
{
    QString path = img::image_root_dir_ % "/queries/unanswered/"
        % ap_type % "/uav_" % QString::number(uav_id);

    int num_images = img::numImagesInDir(path);

    for(int i = 0; i < queries->size(); i++, num_images += 2)
    {
        lcar_msgs::QueryPtr query = queries->at(i);

        sensor_msgs::Image ros_image = query->img;
        QImage image = img::rosImgToQimg(ros_image);

        QString file = "img_" % QString::number(num_images) % ".jpg";
        if(!img::saveImage(path, file, image))
            qCDebug(lcar_bot) << "error saving uav query\n";

        ros_image = query->img_framed;
        image = img::rosImgToQimg(ros_image);

        file = "img_" %  QString::number(num_images+1) % ".jpg";
        if(!img::saveImage(path, file, image))
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
}

void GCSMainWindow::ClearQueries()
{
    for(int i = widget.layout_queries->count() - 1; i >= 0; i--)
        delete widget.layout_queries->itemAt(i)->widget();
    
    num_queries_last = 0;
}


void GCSMainWindow::UpdateQueries()
{
    if(cur_v_id == -1 || !widget.frame_queries_cntnr->isVisible())
        return;

    std::vector<lcar_msgs::QueryPtr> *queries = vm->GetUAVDoorQueries(cur_v_id);
    int pqv_size = queries->size();

    for(int i = num_queries_last; i < pqv_size; i++)
    {
        //retrieve Query msg for door image
        lcar_msgs::QueryPtr doorQuery = queries->at(i);

        QPixmap image = img::rosImgToQpixmap(doorQuery->img_framed);

        //create the widget
        QueryWidget * qw = new QueryWidget();
        qw->SetImage(image);

        connect(qw->YesButton(), &QPushButton::clicked,
                this, [=](){ OnAcceptDoorQuery(qw); });
        connect(qw->RejectButton(), &QPushButton::clicked,
                this, [=](){ OnRejectDoorQuery(qw); });
                
        //add to user interface
        widget.layout_queries->addWidget(qw);
    }

    num_queries_last = pqv_size;
}

void GCSMainWindow::AnswerQuery(QWidget * qw, QString ap_type, bool accepted)
{
    int index = widget.layout_queries->indexOf(qw);
    std::vector<lcar_msgs::QueryPtr> * vec_uav_queries_ptr = vm->GetUAVDoorQueries(cur_v_id);
    lcar_msgs::QueryPtr door = vec_uav_queries_ptr->at(index);

    QString path = img::image_root_dir_ % "/queries";
    QString file;
    if(accepted)
        path.append("/accepted/" % ap_type);
    else
         path.append("/rejected/" % ap_type);
        
    int num_images = img::numImagesInDir(path);
    file.append("img_" % QString::number(num_images) % ".jpg");
    img::saveImage(path, file, door->img);

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

    QString play = widget.cmbo_box_play_book->currentText();
    
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

    this->ToggleScoutButtons(true);
}

void GCSMainWindow::OnScoutBuilding()
{
    if(cur_v_id == -1)
        return;

    QString building = widget.cmbo_box_buildings->currentText();
    
    int number;
    sscanf(building.toStdString().c_str(), "%d", &number);
    
    emit UIAdapter::Instance()->ScoutBuilding(cur_v_id, number);

    this->ToggleScoutButtons(false);

    ROS_INFO_STREAM("Scouting " << building.toStdString());
}

void GCSMainWindow::OnPauseOrResumeScout()
{
    if(cur_v_id == -1)
        return;

    if(vm->GetMissionMode(cur_v_id) == MissionMode::active)
    {
        emit UIAdapter::Instance()->PauseMission(cur_v_id);
        widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/play.png"));
    }
    else
    {
        emit UIAdapter::Instance()->ResumeMission(cur_v_id);
        widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/pause.png"));
    }
}

void GCSMainWindow::OnStopScout()
{
    if(cur_v_id == -1)
        return;

    emit UIAdapter::Instance()->CancelMission(cur_v_id);
    this->ToggleScoutButtons(true);
}

void GCSMainWindow::OnChangeFlightMode(int index)
{
    if(cur_v_id == -1)
        return;
    
    if(index == 0)
    {
        ROS_INFO_STREAM("Quadrotor Stablized");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.stabilized);
    }
    else if(index == 1)
    {
        ROS_INFO_STREAM("Quadrotor Loiter");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.loiter);
    }
    else if(index == 2)
    {
        ROS_INFO_STREAM("Quadrotor Land");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.land);
    }
    else if(index == 3)
    {
        ROS_INFO_STREAM("Altitude Hold");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.altitude_control);
    }
    else if( index == 4)
    {
        ROS_INFO_STREAM("Position Hold");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.position_control);
    }
    else if(index == 5)
    {
        ROS_INFO_STREAM("Quadrotor Return-To-Launch");
        emit UIAdapter::Instance()->SetRTL(cur_v_id);
    }
    else if(index == 6)
    {
        ROS_INFO_STREAM("Quadrotor Auto");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.mode_auto);
    }
    else if(index == 7)
    {
        ROS_INFO_STREAM("Quadrotor Offboard");
        emit UIAdapter::Instance()->SetMode(cur_v_id, flight_modes_.offboard);
    }

    if(vm->GetMissionMode(cur_v_id) != MissionMode::stopped)
        emit UIAdapter::Instance()->CancelMission(cur_v_id);
}

void GCSMainWindow::ToggleScoutButtons(bool visible, QString icon_type)
{ // icon_type should be "play" or "pause"
    widget.btn_scout->setVisible(visible);
    widget.btn_scout_play_pause->setVisible(!visible);
    widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/"%icon_type%".png"));
    widget.btn_scout_stop->setVisible(!visible);
}

// slot gets called when user click on a uav button
void GCSMainWindow::OnUavSelected(VehicleWidget *w)
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
    
    QString vehicle_type = "UAV ";
    if(v_type == VehicleType::ugv)
        vehicle_type = "UGV ";
    
    widget.lbl_cur_uav->setText(vehicle_type % QString::number(v_id - v_type));
    
    QString topic("/V" % QString::number(v_id) % "/stereo_cam/left/image_rect");
    vm->SubscribeToImageTopic(topic);
    
    widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
    this->ClearQueries(); // new uav selected, so make room for its queries
    if(fl_widgets.ap_menu != nullptr)
        fl_widgets.ap_menu->SetUAVAccessPointsAndId(vm->GetUAVAccessPoints(cur_v_id), 
                                                    cur_v_id - v_type);
    
    //todo update gui buttons according to current vehicles mission status 
    MissionMode m = vm->GetMissionMode(cur_v_id);
    if(m == MissionMode::stopped)
        this->ToggleScoutButtons(true);
    else if(m == MissionMode::active)
        this->ToggleScoutButtons(false, "pause");
    else
        this->ToggleScoutButtons(false, "play");

    this->ToggleArmDisarmButton(vm->GetState(cur_v_id)->armed);
}

//
////needed in addUav(int) and deleteUav(int)
//void GCS::SelectUav(int uav_number)
//{
//    Q_ASSERT(0 <= uav_number && uav_number < vm->NumVehiclesByType(VehicleType::quad_rotor));
//
//    if(uav_number == cur_vehicle)
//        return; // no use resetting everything to the way it was
//
//    cur_vehicle = uav_number;
//    widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
//    this->ClearQueries(); // new uav selected, so make room for its queries
//
//    UAVControl * uav = active_uavs[cur_vehicle];
//    // handle image topic subscription and image refresh for new uav
////    sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav->id) + "/stereo_cam/left/image_rect",
////                                     30, &GCS::ImageCallback, this);
//    
//    if(fl_widgets.ap_menu != nullptr)
//        fl_widgets.ap_menu->SetUAV(uav);
//
//    //handle mission and arm buttons for new uav
//    MissionMode m = uav->GetMissionMode();
//    if(m == MissionMode::stopped)
//        this->ToggleScoutButtons(true);
//    else if(m == MissionMode::active)
//        this->ToggleScoutButtons(false, "pause");
//    else
//        this->ToggleScoutButtons(false, "play");
//
//    this->ToggleArmDisarmButton(active_uavs[cur_vehicle]->GetState().armed);
//}

void GCSMainWindow::UpdateFlightStateWidgets()
{
    if(cur_v_id == -1)
        return;

    FlightState flight_state = vm->GetFlightState(cur_v_id);
    
    // PFD
    widget.pfd->setRoll(flight_state.roll*180);
    widget.pfd->setPitch(flight_state.pitch*90);
    widget.pfd->setHeading(flight_state.heading);
    widget.pfd->setAirspeed(flight_state.ground_speed);
    widget.pfd->setAltitude(flight_state.altitude);
    widget.pfd->setClimbRate(flight_state.vertical_speed);
    widget.pfd->update();
    
    QString temp_data;
    //text based widget
    temp_data.setNum(flight_state.yaw, 'f', 2);
    widget.lbl_yaw_val->setText(temp_data);
    
    temp_data.setNum(flight_state.roll, 'f', 2);
    widget.lbl_roll_val->setText(temp_data);
    
    temp_data.setNum(flight_state.pitch, 'f', 2);
    widget.lbl_pitch_val->setText(temp_data);
    
    temp_data.setNum(flight_state.ground_speed, 'f', 2).append(" m/s");
    widget.lbl_gnd_spd_val->setText(temp_data);
    
    temp_data.setNum(vm->GetDistanceToWP(cur_v_id)).append(" m");
    widget.lbl_dist_wp_val->setText(temp_data);
    
    
    StatePtr state = vm->GetState(cur_v_id);
    temp_data.setNum(state->battery * 100);
    widget.pgs_bar_battery->setValue(temp_data.toInt());
    
    int v_index = vm->VehicleIndexFromId(cur_v_id);
    temp_data = "UAV " + QString::number(v_index);
    widget.lbl_cur_uav->setText(temp_data);
    
    widget.pgs_bar_mission->setValue(state->mission_progress * 100);
}

void GCSMainWindow::OnArmOrDisarmSelectedUav()
{
    if(cur_v_id == -1)
        return;

    bool armed = vm->IsArmed(cur_v_id);

    this->ToggleArmDisarmButton(armed);

    emit UIAdapter::Instance()->Arm(cur_v_id, !armed);
}

void GCSMainWindow::ToggleArmDisarmButton(bool arm)
{
    if(arm)
        widget.btn_arm_uav->setText("Disarm");
    else
        widget.btn_arm_uav->setText("Arm");
}

void GCSMainWindow::InitMap()
{
    QString s = ros::package::getPath("gcs").c_str();
    QString map_url = "file://" % s % "/map/uavmap.html";
    widget.web_view->load(QUrl(map_url));
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
    QSettings settings(company_, application_);

    settings.beginGroup("general_tab");

    if(settings.value("machine_learning", "online").toString() == "online")
        this->OnToggleMachineLearningMode(true);
    else
        this->OnToggleMachineLearningMode(false);

    QString default_path = QProcessEnvironment::systemEnvironment().value("HOME") % "/Pictures/" % company_;
    img::image_root_dir_ = settings.value("images_root_directory", default_path).toString();

    settings.endGroup();
}

void GCSMainWindow::OnAccessPointsTriggered()
{
    if(fl_widgets.ap_menu == nullptr)
    {
        fl_widgets.ap_menu = new AccessPointsContainerWidget();

        QRect window = this->window()->geometry();
        int x = (this->width() / 2) - (fl_widgets.ap_menu->width() / 2);
        int y = (this->height() / 2) - (fl_widgets.ap_menu->height() / 2);
        fl_widgets.ap_menu->move(window.x() + x, window.y() + y);
        fl_widgets.ap_menu->setVisible(true);

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
        fl_widgets.settings = new SettingsWidget(vm);

        QRect window = this->window()->geometry();
        int x = (this->width() / 2) - (fl_widgets.settings->width() / 2);
        int y = (this->height() / 2) - (fl_widgets.settings->height() / 2);
        fl_widgets.settings->move(window.x() + x, window.y() + y);
        fl_widgets.settings->setVisible(true);

        connect(fl_widgets.settings, &SettingsWidget::destroyed,
                this, [=](){ fl_widgets.settings = nullptr; });
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
        fl_widgets.unanswered_queries = new UnansweredQueries(this);

        QRect window = this->window()->geometry();
        int x = (this->width() / 2) - (fl_widgets.unanswered_queries->width() / 2);
        int y = (this->height() / 2) - (fl_widgets.unanswered_queries->height() / 2);
        fl_widgets.unanswered_queries->move(window.x() + x, window.y() + y);
        fl_widgets.unanswered_queries->setVisible(true);

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

        QRect window = this->window()->geometry();
        int x = (this->width() / 2) - (fl_widgets.vehicle_init->width() / 2);
        int y = (this->height() / 2) - (fl_widgets.vehicle_init->height() / 2);
        fl_widgets.vehicle_init->move(window.x() + x, window.y() + y);
        fl_widgets.vehicle_init->setVisible(true);

        connect(fl_widgets.vehicle_init, &VehicleInitWidget::destroyed,
                this, [=](){ fl_widgets.vehicle_init = nullptr; });
    }
    else
    {
        fl_widgets.vehicle_init->showNormal();
        fl_widgets.vehicle_init->activateWindow();
    }
}

void GCSMainWindow::OnToggleMachineLearningMode(bool toggle)
{
    widget.frame_queries_cntnr->setVisible(toggle);
    widget.frame_queries_cntnr->setEnabled(toggle);
}

std::string GCSMainWindow::GetMissionType(std::string file_name)
{
    std::string mission_type;
    std::ifstream fileIn(file_name);

    fileIn >> mission_type;
    fileIn.close();

    return mission_type;
}

void GCSMainWindow::UpdateVehicleWidgets()
{
    //todo unhard code this v_type
    int v_type = VehicleType::quad_rotor;
    int num_vehicle = vm->NumVehiclesByType(v_type);
    for(int i = 0; i < num_vehicle; i++)
    {
        VehicleWidget * widget = this->VehicleWidgetAt(v_type, i);
        StatePtr ptr = vm->GetState(v_type + i);
        widget->SetBattery(ptr->battery);
        widget->SetCondition(ptr->mode.c_str());
    }
}

void GCSMainWindow::OnUpdateCameraFeed(QPixmap img)
{
    int w = widget.image_frame->width();
    int h = widget.image_frame->height();
    widget.image_frame->setPixmap(img.scaled(w, h, Qt::KeepAspectRatio));
}

void GCSMainWindow::closeEvent(QCloseEvent* event)
{    
    
    // this wouldn't work in the destructor
    if(fl_widgets.ap_menu != nullptr) 
        fl_widgets.ap_menu->close();
    
    if(fl_widgets.settings != nullptr)
        fl_widgets.settings->close();
    
    if(fl_widgets.unanswered_queries != nullptr)
        fl_widgets.unanswered_queries->close();
    
    if(fl_widgets.vehicle_init != nullptr)
        fl_widgets.vehicle_init->close();
    
    event->accept();
}

VehicleWidget* GCSMainWindow::VehicleWidgetAt(int v_type, int index)
{
    Q_ASSERT(VehicleType::invalid_low < v_type && v_type < VehicleType::invalid_high);
    Q_ASSERT(index < vm->NumVehiclesByType(v_type));
    
    return static_cast<VehicleWidget*>(layout_by_v_type[v_type]->itemAt(index)->widget());
}

} // namespace
