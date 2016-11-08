
#include <stdio.h>
#include <fstream>

#include <QProcessEnvironment>
#include <QDesktopWidget>
#include <QMainWindow>
#include <QStringBuilder>
#include <QMetaType>

#include "rqt_gcs/query_widget.h"
#include "rqt_gcs/gcs.h"
#include "util/image.h"
#include "util/debug.h"
#include "util/strings.h"
#include "rqt_gcs/vehicle_init_widget.h"
#include "util/data_types.h"

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

namespace rqt_gcs
{
    
    namespace img
    {
        QString image_root_dir_; // defined globally in util/strings.h
    }

GCS::GCS():
    cur_v_id(-1),
    time_counter(0),
    num_queries_last(0),
    update_timer(new QTimer(this)),
    vm(new VehicleManager())
{
    widget.setupUi(this);
    
    //todo add these layouts to the GUI
    //layout_by_v_type.insert(VehicleType::ugv, widget.layout_ugvs);
    layout_by_v_type.insert(VehicleType::quad_rotor, widget.layout_quads);
    //layout_by_v_type.insert(VehicleType::octo_rotor, widget.layout_octo);
    //layout_by_v_type.insert(VehicleType::vtol, widget.layout_vtol);

    //setup button logic for the widgets
    connect(widget.btn_exec_play, &QPushButton::clicked, 
            this, &GCS::OnExecutePlay);
    connect(widget.btn_scout, &QPushButton::clicked, 
            this, &GCS::OnScoutBuilding);
    connect(widget.btn_scout_play_pause, &QPushButton::clicked, 
            this, &GCS::OnPauseOrResumeScout);
    connect(widget.btn_scout_stop, &QPushButton::clicked, 
            this, &GCS::OnStopScout);
    //this one doesn't like the new signal slot syntax for some reason
    connect(widget.cmbo_box_flight_mode, SIGNAL(currentIndexChanged(int)), 
            this, SLOT(OnChangeFlightMode(int)));
    connect(widget.btn_view_acess_points, &QPushButton::clicked, 
            this, &GCS::OnAccessPointsTriggered);
    connect(widget.btn_arm_uav, &QPushButton::clicked, 
            this, &GCS::OnArmOrDisarmSelectedUav);
    connect(update_timer, &QTimer::timeout,
            this, &GCS::OnTimedUpdate);
    
    connect(vm, &VehicleManager::NewImageFrame, 
            this, &GCS::OnUpdateCameraFeed);
    connect(vm, &VehicleManager::AddVehicleWidget,
            this, &GCS::OnAddVehicleWidget);
    
    dbg::InitDbg();
    this->InitMenuBar();
    this->InitSettings();
    this->InitHelperThread();
    this->InitMap();
    this->ToggleScoutButtons(true);
    
    update_timer->start(0);
}

GCS::~GCS()
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

void GCS::OnAddVehicleWidget(int v_id)
{
    int v_type =  vm->VehicleTypeFromId(v_id);
    Q_ASSERT(v_type != VehicleType::invalid_low);
    
    VehicleWidget *w = new VehicleWidget();
    w->SetId(v_id);
    w->SetNumber(v_id - v_type);
    w->SetName("UAV " % QString::number(v_id - v_type));

    // - 1 accounts for the vehicle corresponding to this widget
    // that was added before the GUI was told to add this widget.
    int num_vehicle = vm->NumVehiclesByType(v_type) - 1;
    int index = 0;
    while(index < num_vehicle && v_id > this->VehicleWidgetAt(v_type, index)->Id())
        index++;
    
    //map this widgets vehicle select button to selecting this widget
    connect(w->Button(), &QPushButton::clicked,
            this, [=](){ OnUavSelected(w); } );
    
    layout_by_v_type[v_type]->insertWidget(index, w);  
}

void GCS::OnDeleteVehicleWidget(int v_id)
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
    
    //todo handle cases where num_vehicle != 0
    
    widget_deleted->wakeAll();
    widget_mutex->unlock();
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

void GCS::SaveUavQueries(int uav_id, const std::vector<lcar_msgs::QueryPtr> *queries, const QString ap_type)
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
void GCS::OnTimedUpdate()
{
    if(vm->NumVehiclesByType(VehicleType::quad_rotor) == 0)
        return;

    uav_mutex.lock();

    if(time_counter++ >= 100)
    {
        this->UpdateQueries();
        time_counter = 0;
    }

    this->UpdateFlightStateWidgets();
    this->UpdateVehicleWidgets();

    uav_mutex.unlock();
}

void GCS::ClearQueries()
{
    for(int i = widget.layout_queries->count() - 1; i >= 0; i--)
        delete widget.layout_queries->itemAt(i)->widget();
    
    num_queries_last = 0;
}


void GCS::UpdateQueries()
{
    if(vm->NumQuadRotors() == 0 || !widget.frame_queries_cntnr->isVisible())
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
                this, [=](){OnAcceptDoorQuery(qw); });
        connect(qw->RejectButton(), &QPushButton::clicked,
                this, [=](){OnRejectDoorQuery(qw); });
                
        //add to user interface
        widget.layout_queries->addWidget(qw);
    }

    num_queries_last = pqv_size;
}

void GCS::AnswerQuery(QWidget * qw, QString ap_type, bool accepted)
{
//    int index = widget.layout_queries->indexOf(qw);
//    std::vector<lcar_msgs::QueryPtr> * vec_uav_queries_ptr = active_uavs[cur_v_id]->GetRefDoorQueries();
//    lcar_msgs::QueryPtr door = vec_uav_queries_ptr->at(index);
//
//    UAVControl* uav = active_uavs[cur_v_id];
//    QString path = img::image_root_dir_ % "/queries";
//    QString file;
//    if(accepted)
//    {
//        path.append("/accepted/" % ap_type % "/uav_" % QString::number(uav->id));
//        file.append("img_" % QString::number(uav->accepted_images++) % ".jpg");
//    }
//    else
//    {
//        path.append("/rejected/" % ap_type % "/uav_" % QString::number(uav->id));
//        file.append("img_" % QString::number(uav->rejected_images++) % ".jpg");
//    }
//
//    img::saveImage(path, file, door->img);
//
//    vec_uav_queries_ptr->erase(vec_uav_queries_ptr->begin() + index);
//    delete qw;
//
//    door->is_accepted = accepted;
//    num_queries_last--;
//    //uav->SendDoorResponse(doormsg);
}

void GCS::OnAcceptDoorQuery(QWidget *qw)
{
    AnswerQuery(qw, "door", true);
}

void GCS::OnRejectDoorQuery(QWidget * qw)
{
    AnswerQuery(qw, "door", false);
}

void GCS::OnExecutePlay()
{
    if(vm->NumTotalVehicles() == 0)
        return;

    int play_num = widget.cmbo_box_play_book->currentIndex();

    if(play_num == 0)
    {
        for(int i = 0; i < vm->NumQuadRotors(); i++)
        {
            std::string file_name = "play " + std::to_string(i + 1);

            
            vm->Arm(VehicleType::quad_rotor+i, true);
            std::string mission_type = this->GetMissionType(file_name);
            ROS_WARN_STREAM("called get mission type");
            if(mission_type.compare("local") == 0)
            {
//                vm->ScoutBuilding(GetMissionLocal(file_name));
            }
            else
//                active_uavs[i]->ScoutBuilding(GetMissionGlobal(file_name));

            ROS_INFO_STREAM("Scouting Building " << i);
        }
    }
    else if(play_num == 1)
    {
        //todo
    }
    else if(play_num == 2)
    {
        //todo
    }

    ROS_INFO_STREAM("Play " << play_num << " initiated");
}

void GCS::OnCancelPlay()
{
    if(vm->NumTotalVehicles() == 0)
        return;

    for(int i = 0; i < vm->NumQuadRotors(); i++)
//        active_uavs[i]->StopMission();
    

    this->ToggleScoutButtons(true);
}

void GCS::OnScoutBuilding()
{
    if(vm->NumQuadRotors() == 0)
        return;

    int building_index = widget.cmbo_box_buildings->currentIndex() + 1;
    std::string file_name = "building" + std::to_string(building_index) + ".txt";

    if(this->GetMissionType(file_name).compare("local") == 0)
        vm->ScoutBuilding(cur_v_id, this->GetMissionLocal(file_name));
    else
        vm->ScoutBuilding(cur_v_id, this->GetMissionGlobal(file_name));

    this->ToggleScoutButtons(false);

    ROS_INFO_STREAM("Scouting Building " << building_index);
}

void GCS::OnPauseOrResumeScout()
{
    if (vm->NumQuadRotors() == 0)
        return;

    if(vm->GetMissionMode(cur_v_id) == MissionMode::active)
    {
        active_uavs[cur_vehicle]->PauseMission();
        widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/play.png"));
    }
    else
    {
        active_uavs[cur_vehicle]->ResumeMission();
        widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/pause.png"));
    }
}

void GCS::OnStopScout()
{
    if(vm->NumQuadRotors() == 0)
        return;

    active_uavs[cur_v_id]->StopMission();
    this->ToggleScoutButtons(true);
}

void GCS::OnChangeFlightMode(int index)
{
    if(vm->NumQuadRotors() == 0)
        return;
    
    if(index == 0)
    {
        ROS_INFO_STREAM("Quadrotor Stablized");
        vm->SetMode(cur_v_id, "STABILIZED");
    }
    else if(index == 1)
    {
        ROS_INFO_STREAM("Quadrotor Loiter");
        vm->SetMode(cur_v_id, "AUTO.LOITER");
    }
    else if(index == 2)
    {
        ROS_INFO_STREAM("Quadrotor Land");
        vm->SetMode(cur_v_id, "AUTO.LAND");
    }
    else if(index == 3)
    {
        ROS_INFO_STREAM("Altitude Hold");
        vm->SetMode(cur_v_id, "ALTCTL");
    }
    else if( index == 4)
    {
        ROS_INFO_STREAM("Position Hold");
        vm->SetMode(cur_v_id, "POSCTL");
    }
    else if(index == 5)
    {
        ROS_INFO_STREAM("Quadrotor Return-To-Launch");
        vm->SetRTL(cur_v_id);
    }
    else if(index == 6)
    {
        ROS_INFO_STREAM("Quadrotor Auto");
        active_uavs[cur_v_id]->SetMode("AUTO");
    }
    else if(index == 7)
    {
        ROS_INFO_STREAM("Quadrotor Offboard");
        active_uavs[cur_v_idd]->SetMode("OFFBOARD");
    }

    if(active_uavs[cur_v_id]->GetMissionMode() != MissionMode::stopped)
        this->OnStopScout();
}

void GCS::ToggleScoutButtons(bool visible, QString icon_type)
{ // icon_type should be "play" or "pause"
    widget.btn_scout->setVisible(visible);
    widget.btn_scout_play_pause->setVisible(!visible);
    widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/"%icon_type%".png"));
    widget.btn_scout_stop->setVisible(!visible);
}

// slot gets called when user click on a uav button
void GCS::OnUavSelected(VehicleWidget *w)
{
//    this->SelectUav(layout_by_v_type[VehicleType::quad_rotor]->indexOf(w));
    this->SelectVehicleWidget(w->Id());
}


void GCS::SelectVehicleWidgetById(int v_id)
{
    int v_type = vm->VehicleTypeFromId(v_id);
    Q_ASSERT(VehicleType::invalid_low < v_type && v_type < VehicleType::invalid_high);
    
    if(cur_v_id == v_id)
        return;
    
    cur_v_id = v_id;
    
    QString vehicle_type = "UAV";
    if(v_type == VehicleType::ugv)
        vehicle_type = "UGV";
    
    widget.lbl_cur_uav->setText(vehicle_type % " " % QString::number(index));
    
    widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
    this->ClearQueries(); // new uav selected, so make room for its queries
    QString topic("/V" % QString::number(v_id) % "/stereo_cam/left/image_rect");
    vm->SubscribeToImageTopic(topic);
    
    if(fl_widgets.ap_menu != nullptr)
    {
        //todo change ap_menu to accept a given uav's access point queue instead of actual uav
    }
    
    //todo update gui buttons according to current vehicles mission status 
    
}


//needed in addUav(int) and deleteUav(int)
void GCS::SelectUav(int uav_number)
{
    Q_ASSERT(0 <= uav_number && uav_number < vm->NumVehiclesByType(VehicleType::quad_rotor));

    if(uav_number == cur_vehicle)
        return; // no use resetting everything to the way it was

    cur_vehicle = uav_number;
    widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
    this->ClearQueries(); // new uav selected, so make room for its queries

    UAVControl * uav = active_uavs[cur_vehicle];
    // handle image topic subscription and image refresh for new uav
//    sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav->id) + "/stereo_cam/left/image_rect",
//                                     30, &GCS::ImageCallback, this);
    
    if(fl_widgets.ap_menu != nullptr)
        fl_widgets.ap_menu->SetUAV(uav);

    //handle mission and arm buttons for new uav
    MissionMode m = uav->GetMissionMode();
    if(m == MissionMode::stopped)
        this->ToggleScoutButtons(true);
    else if(m == MissionMode::active)
        this->ToggleScoutButtons(false, "pause");
    else
        this->ToggleScoutButtons(false, "play");

    this->ToggleArmDisarmButton(active_uavs[cur_vehicle]->GetState().armed);
}

void GCS::UpdateFlightStateWidgets()
{
    if(vm->NumUAVs() == 0)
        return;

    FlightState state = vm->GetFlightState(cur_v_id);
    
    // PFD
    widget.pfd->setRoll(state.roll*180);
    widget.pfd->setPitch(state.pitch*90);
    widget.pfd->setHeading(state.heading);
    widget.pfd->setAirspeed(state.ground_speed);
    widget.pfd->setAltitude(state.altitude);
    widget.pfd->setClimbRate(state.vertical_speed);
    widget.pfd->update();

    UAVControl* uav;
    
    QString temp_data;
    //text based widget
    temp_data.setNum(state.yaw, 'f', 2);
    widget.lbl_yaw_val->setText(temp_data);
    temp_data.setNum(state.roll, 'f', 2);
    widget.lbl_roll_val->setText(temp_data);
    temp_data.setNum(state.pitch, 'f', 2);
    widget.lbl_pitch_val->setText(temp_data);
    temp_data.setNum(state.ground_speed, 'f', 2);
    temp_data.append(" m/s");
    widget.lbl_gnd_spd_val->setText(temp_data);
    temp_data.setNum(uav->GetDistanceToWP());
    temp_data.append(" m");
    widget.lbl_dist_wp_val->setText(temp_data);
    temp_data.setNum(uav->GetBatteryState().percentage * 100);
    widget.pgs_bar_battery->setValue(temp_data.toInt());
    temp_data = "UAV " + QString::number(uav->id);
    widget.lbl_cur_uav->setText(temp_data);
    widget.pgs_bar_mission->setValue(uav->GetMissionProgress() * 100);
}

void GCS::OnArmOrDisarmSelectedUav()
{
    if(vm->NumVehiclesByType(VehicleType::quad_rotor) == 0)
        return;

    bool armed = active_uavs[cur_vehicle]->GetState().armed;

    this->ToggleArmDisarmButton(armed);

    active_uavs[cur_vehicle]->Arm(!armed);
}

void GCS::ToggleArmDisarmButton(QString arm)
{
    if(arm)
        widget.btn_arm_uav->setText("Disarm");
    else
        widget.btn_arm_uav->setText("Arm");
}

void GCS::InitHelperThread()
{
    thread_uav_monitor = new GCSHelperThread(this);
    
    connect(thread_uav_monitor, &GCSHelperThread::AddUav,
            this, &GCS::OnAddUav);
    connect(thread_uav_monitor, &GCSHelperThread::DeleteUav,
            this, &GCS::OnDeleteUav);
    connect(thread_uav_monitor, &GCSHelperThread::ToggleUavConnection,
            this, &GCS::OnUAVConnectionToggled);
            
    thread_uav_monitor->start();
}

void GCS::InitMap()
{
    QString s = ros::package::getPath("rqt_gcs").c_str();
    QString map_url = "file://" % s % "/map/uavmap.html";
    widget.web_view->load(QUrl(map_url));
}

void GCS::InitMenuBar()
{
    QMenuBar *menu_bar = this->menuBar();
    
    QMenu *file_menu = menu_bar->addMenu("File");
    QAction *add_vehicle_act = file_menu->addAction("Add Vehicle(s)");
    connect(add_vehicle_act, &QAction::triggered,
            this, &GCS::OnAddVehicleTriggered);

    //view menu
    QMenu *view_menu = menu_bar->addMenu("View");
    QAction *unanswered_queries_act = view_menu->addAction("Unanswered Queries");
    connect(unanswered_queries_act, &QAction::triggered,
            this, &GCS::OnUnansweredQueriesTriggered);

    //tools menu
    QMenu *tools_menu = menu_bar->addMenu("Tools");
    QAction *settings_act = tools_menu->addAction("Settings");
    connect(settings_act, &QAction::triggered,
            this, &GCS::OnSettingsTriggered);

    //help menu
    QMenu *help_menu = menu_bar->addMenu("Help");
    QAction *lcar_bot_act = help_menu->addAction("Learning Classifying And Recognizing Bot (LCAR-Bot)");
    QAction *ros_act      = help_menu->addAction("Robot Operating System (ROS)");
    QAction *opencv_act   = help_menu->addAction("Open Computer Vision (OpenCV)");
    QAction *qt_act       = help_menu->addAction("Qt");
    QAction *about_act    = help_menu->addSection("About");
}

// SettingsWidget and QSettings related stuff
void GCS::InitSettings()
{
    //initialize settings and uav queries display
    QSettings settings(COMPANY, APPLICATION);

    settings.beginGroup("general_tab");

    if(settings.value("machine_learning", "online").toString() == "online")
        this->OnToggleMachineLearningMode(true);
    else
        this->OnToggleMachineLearningMode(false);

    QString default_path = QProcessEnvironment::systemEnvironment().value("HOME") % "/Pictures/LCAR_Bot";
    img::image_root_dir_ = settings.value("images_root_directory", default_path).toString();

    settings.endGroup();
}

void GCS::OnAccessPointsTriggered()
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

        if(vm->NumQuadRotors() > 0)
            fl_widgets.ap_menu->SetUAV(active_uavs[cur_vehicle]);
        else
            fl_widgets.ap_menu->SetUAV(nullptr);
    }
    else
        fl_widgets.ap_menu->showNormal();
        fl_widgets.ap_menu->activateWindow();
}

void GCS::OnSettingsTriggered()
{
    if(fl_widgets.settings == nullptr)
    {
        fl_widgets.settings = new SettingsWidget(vm);

        QRect window = this->window()->geometry();
        int x = (this->width() / 2) - (fl_widgets.settings->width() / 2);
        int y = (this->height() / 2) - (fl_widgets.settings->height() / 2);
        fl_widgets.settings->move(window.x() + x, window.y() + y);
        fl_widgets.settings->setVisible(true);
        
        connect(fl_widgets.settings, &SettingsWidget::machineLearningModeToggled,
                this, &GCS::OnToggleMachineLearningMode);

        connect(fl_widgets.settings, &SettingsWidget::destroyed,
                this, [=](){ fl_widgets.settings = nullptr; });
    }
    else
    {
        fl_widgets.settings->showNormal();
        fl_widgets.settings->activateWindow();
    }
}

void GCS::OnUnansweredQueriesTriggered()
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

void GCS::OnAddVehicleTriggered()
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

void GCS::OnToggleMachineLearningMode(bool toggle)
{
    widget.frame_queries_cntnr->setVisible(toggle);
    widget.frame_queries_cntnr->setEnabled(toggle);

    for(int i = 0; i < NUM_UAV; i++)
        active_uavs[i]->SetOnlineMode(toggle);
}

std::string GCS::GetMissionType(std::string file_name)
{
    std::string mission_type;
    std::ifstream fileIn(file_name);

    fileIn >> mission_type;
    fileIn.close();

    return mission_type;
}

lcar_msgs::TargetLocal GCS::GetMissionLocal(std::string file_name)
{
    float pos_x, pos_y, pos_z, radius;
    std::string mission_type;

    lcar_msgs::TargetLocal building_local;

    std::ifstream fileIn(file_name);

    fileIn >> mission_type;
    fileIn >> pos_x;
    fileIn >> pos_y;
    fileIn >> pos_z;
    fileIn >> radius;

    building_local.target.position.x = pos_x;
    building_local.target.position.y = pos_y;
    building_local.target.position.z = pos_z;
    building_local.radius = radius;

    return building_local;
}

lcar_msgs::TargetGlobal GCS::GetMissionGlobal(std::string file_name)
{
    float pos_x, pos_y, pos_z, radius;
    lcar_msgs::TargetGlobal building_global;
    std::string mission_type;

    std::ifstream fileIn(file_name);

    fileIn >> mission_type;
    fileIn >> pos_x;
    fileIn >> pos_y;
    fileIn >> pos_z;
    fileIn >> radius;

    building_global.target.latitude = pos_x;
    building_global.target.longitude = pos_y;
    building_global.target.altitude = pos_z;

    return building_global;
}

void GCS::UpdateVehicleWidgets()
{
    int v_type = VehicleType::quad_rotor;
    int num_vehicle = vm->NumVehiclesByType(v_type);
    for(int i = 0; i < num_vehicle; i++)
    {
        VehicleWidget * widget = this->VehicleWidgetAt(v_type, i);
        int num = active_uavs[i]->GetBatteryState().percentage * 100;
        widget->SetBattery(num);
        QString temp_data = active_uavs[i]->GetState().mode.c_str();
        widget->SetCondition(temp_data);
    }
}

void GCS::OnUpdateCameraFeed(QPixmap img)
{
    int w = widget.image_frame->width();
    int h = widget.image_frame->height();
    widget.image_frame->setPixmap(img.scaled(w, h, Qt::KeepAspectRatio));
}

void GCS::closeEvent(QCloseEvent* event)
{
    thread_uav_monitor->Stop();
    thread_uav_monitor->wait();
    delete thread_uav_monitor;
    
    // move backwards because OnDeleteUav decrements NUM_UAV
    for(int i = NUM_UAV - 1; i >= 0; i--)
        this->OnDeleteUav(i);
    
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

VehicleWidget* GCS::VehicleWidgetAt(int v_type, int index)
{
    Q_ASSERT(v_type != VehicleType::invalid_low);
//    Q_ASSERT(index < vm->NumVehiclesByType(v_type));
    
    return static_cast<VehicleWidget*>(layout_by_v_type[v_type]->itemAt(index)->widget());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////  SimpleGCSHelper  /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

GCSHelperThread::GCSHelperThread(GCS * sgcs) :
    gcs(sgcs)
{
}

GCSHelperThread::~GCSHelperThread()
{
    gcs = nullptr;
}

void GCSHelperThread::ParseUavNamespace(std::map<int, int>& map)
{  
    ros::V_string nodes;
    ros::master::getNodes(nodes);
    
    for(const std::string & node : nodes)
    {
        int index = node.find("/V");
        if(index == node.npos)
        {   //nodes are printed alphabetically, if we passed UAV namespace, there are no more
            if(map.size() > 0)
                break;
            else  // skip nodes in front of /UAV namespace
                continue;
        }

        std::string id_string = node.substr(index+4, id_string.length());
        id_string = id_string.substr(0,id_string.find("/"));
        char * end;
        int uav_id = std::strtol(&id_string[0], &end, 10);

        if(map.count(uav_id) == 0)
            map.insert(std::pair<int,int>(uav_id, uav_id));
    }
}

void GCSHelperThread::MonitorUavNamespace()
{
    std::map<int,int> uav_map;
    ParseUavNamespace(uav_map);

    gcs->uav_mutex.lock();

    if(gcs->NUM_UAV < uav_map.size()) // uav added
    {
        for(auto const& uav : uav_map)
        {
            if(!gcs->uav_db.contains(uav.first))
            {
                emit AddUav(uav.first);
                gcs->num_uav_changed.wait(&gcs->uav_mutex);
            }
        }
    }
    else if(gcs->NUM_UAV > uav_map.size()) // uav shutdown
    {
        for(int i = 0; i < gcs->active_uavs.size(); i++)
        {
            if(uav_map.count(gcs->active_uavs[i]->id) == 0)
            {
                emit DeleteUav(i);
                gcs->num_uav_changed.wait(&gcs->uav_mutex);
                i--;
            }
        }
    }

    gcs->uav_mutex.unlock();
}

void GCSHelperThread::MonitorUavConnections()
{
    /*
     * under normal conditions (recieved a heartbeat), button stylesheet is:
     *      background-color: rgb(64, 89, 140);
     *      color: rgb(240, 240, 240);
     *
     * when heartbeat is not recieved, we grey it out:
     *      background-color: rgb(80, 90, 100);
     *      color: rgb(150, 150, 150);
     */
    
    for(int i = 0; i < gcs->NUM_UAV; i++)
    {
        UAVControl* uav = gcs->active_uavs[i];
        VehicleWidget * vw = gcs->VehicleWidgetAt(VehicleType::quad_rotor, i);
        bool enabled = vw ->IsButtonEnabled();

        if(!uav->RecievedHeartbeat()) // no heartbeat
        {
            if(enabled) // is the button already disabled?
                emit ToggleUavConnection(i, uav->id, false);
        }
        else
        {
            if(!enabled) // is the button already enabled?
                emit ToggleUavConnection(i, uav->id, true);
        }
    }
}

void GCSHelperThread::RunUavs()
{
    for(auto const& uav : gcs->active_uavs)
        uav->Run();
}

void GCSHelperThread::run()
{
    while(!this->isInterruptionRequested())
    {
//        MonitorUavNamespace();
//        MonitorUavConnections();
        RunUavs();  
    }
}

void GCSHelperThread::Stop()
{
    this->requestInterruption();
}

} // namespace
