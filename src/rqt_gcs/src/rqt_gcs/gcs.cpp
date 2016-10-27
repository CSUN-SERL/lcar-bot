
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
#include "rqt_gcs/vehicle_init_widget.h"

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

namespace rqt_gcs
{

    namespace img
    {
        QString image_root_dir_; // defined globally in util/image.h
    }

GCS::GCS():
    NUM_UAV(0),
    cur_uav(-1),
    time_counter(0),
    img_q_max_size(30),
    num_queries_last(0),
    update_timer(new QTimer(this)),
    vm(new VehicleManager())
{
    widget.setupUi(this);

    //setup button logic for the widgets
    connect(widget.btn_exec_play, &QPushButton::clicked, 
            this, &GCS::OnExecutePlay);
    connect(widget.btn_scout, &QPushButton::clicked, 
            this, &GCS::OnScoutBuilding);
    connect(widget.btn_scout_play_pause, &QPushButton::clicked, 
            this, &GCS::OnPauseOrResumeScout);
    connect(widget.btn_scout_stop, &QPushButton::clicked, 
            this, &GCS::OnStopScout);
    connect(widget.cmbo_box_flight_mode, SIGNAL(currentIndexChanged(int)), 
            this, SLOT(OnChangeFlightMode(int)));
    connect(widget.btn_view_acess_points, &QPushButton::clicked, 
            this, &GCS::OnAccessPointsTriggered);
    connect(widget.btn_arm_uav, &QPushButton::clicked, 
            this, &GCS::OnArmOrDisarmSelectedUav);
    connect(this, &GCS::NewCameraFeedFrame, 
            this, &GCS::OnUpdateCameraFeed);
    connect(update_timer, &QTimer::timeout,
            this, &GCS::OnTimedUpdate);
    
    dbg::InitDbg();
    this->InitMenuBar();
    this->InitSettings();
    this->InitHelperThread();
    this->InitMap();
    this->AdvertiseObjectDetection();
    this->ToggleScoutButtons(true);

    update_timer->start(0);
}

GCS::~GCS()
{
    ros::shutdown();
}

void GCS::OnAddUav(int uav_id)
{
    uav_mutex.lock();

    UAVControl * uav = new UAVControl(uav_id);

    //todo loop and get num_image for each acess point type;
    //create separate accepted and rejected counts for each ap_type
    QString path = img::image_root_dir_ + "/queries/accepted/door/uav_" + QString::number(uav_id);
    uav->accepted_images = img::numImagesInDir(path);

    path = img::image_root_dir_ % "/queries/unanswered/door/uav_" % QString::number(uav_id);
    uav->rejected_images = img::numImagesInDir(path);

    int index = 0;
    while(index < NUM_UAV && uav_id > active_uavs[index]->id)
        index++;

    uav_db.insert(uav_id, uav);
    active_uavs.insert(active_uavs.begin() + index, uav);

    VehicleWidget *uav_widget = new VehicleWidget();
    uav_widget->SetNumber(uav_id);
    temp_data = "UAV " + QString::number(uav_id);
    uav_widget->SetName(temp_data);

    //map this widgets vehicle select button to selecting this widget
    connect(uav_widget->Button(), &QPushButton::clicked,
            this, [=](){ OnUavSelected(uav_widget); } );

    widget.layout_uavs->insertWidget(index, uav_widget);

    NUM_UAV++;

    if(NUM_UAV == 1)
        SelectUav(0);

    num_uav_changed.wakeAll();
    uav_mutex.unlock();
}

void GCS::OnDeleteUav(int index)
{
    uav_mutex.lock();

    VehicleWidget * vw = this->VehicleWidgetAt(index);
    UAVControl * uav = active_uavs[index];
    
    this->SaveUavQueries(uav->id, uav->GetDoorQueries(), "door");

    // dont delete uav_db[index] as it points to same UAVControl object as active_uavs
    delete uav;
    delete vw; // removes this widget from the layout

    active_uavs.erase(active_uavs.begin() + index);
    uav_db.erase(uav_db.begin() + index);

    NUM_UAV--;

    if(NUM_UAV == 0) // no more uav's
    {
        if(fl_widgets.ap_menu != nullptr)
            fl_widgets.ap_menu->SetUAV(nullptr);

        this->ClearQueries();
        this->ToggleScoutButtons(true);  // reset scout buttons
        this->ToggleArmDisarmButton(false); //set [dis]arm button to arm

        widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
        widget.lbl_cur_uav->setText("NO UAVS");
        cur_uav = -1;
    }
    //deleted uav was in front of cur_uav, causing NUM_UAV to shrink. cur_uav >= NUM_UAV
    else if(NUM_UAV <= cur_uav) // subsequent calls to TimedUpdate will be invalid at this point
    {
        this->SelectUav(NUM_UAV - 1);
    }
    else if(cur_uav == index) // deleted current uav
    {
        if(cur_uav > 0)
            this->SelectUav(cur_uav - 1);
        else
            this->SelectUav(0);
    }

   num_uav_changed.wakeAll();
   uav_mutex.unlock();
}

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

void GCS::OnUAVConnectionToggled(int index, int uav_id, bool toggle)
{
    if(index >= active_uavs.size() || active_uavs[index]->id != uav_id)
        return;

    this->VehicleWidgetAt(index)->ToggleButton(toggle);
}

//Timed update of GCS gui
void GCS::OnTimedUpdate()
{
    if(NUM_UAV == 0)
        return;

    uav_mutex.lock();

    Q_ASSERT(0 <= cur_uav && cur_uav < NUM_UAV);

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
    if(NUM_UAV == 0 || !widget.frame_queries_cntnr->isVisible())
        return;

    vec_uav_queries_ptr = active_uavs[cur_uav]->GetDoorQueries();
    int pqv_size = vec_uav_queries_ptr->size();

    for(int i = num_queries_last; i < pqv_size; i++)
    {
        //retrieve Query msg for door image
        lcar_msgs::QueryPtr doorQuery = vec_uav_queries_ptr->at(i);

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
    int index = widget.layout_queries->indexOf(qw);
    lcar_msgs::QueryPtr door = vec_uav_queries_ptr->at(index);

    UAVControl* uav = active_uavs[cur_uav];
    QString path = img::image_root_dir_ % "/queries";
    QString file;
    if(accepted)
    {
        path.append("/accepted/" % ap_type % "/uav_" % QString::number(uav->id));
        file.append("img_" % QString::number(uav->accepted_images++) % ".jpg");
    }
    else
    {
        path.append("/rejected/" % ap_type % "/uav_" % QString::number(uav->id));
        file.append("img_" % QString::number(uav->rejected_images++) % ".jpg");
    }

    img::saveImage(path, file, door->img);

    vec_uav_queries_ptr->erase(vec_uav_queries_ptr->begin() + index);
    delete qw;

    door->is_accepted = accepted;
    num_queries_last--;
    //uav->SendDoorResponse(doormsg);
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
    if(NUM_UAV == 0)
        return;

    int play_num = widget.cmbo_box_play_book->currentIndex();

    if(play_num == 0)
    {
        for(int i = 0; i < NUM_UAV; i++)
        {
            std::string file_name = "play " + std::to_string(i + 1);

            active_uavs[i]->Arm(true);
            std::string mission_type = this->GetMissionType(file_name);
            ROS_WARN_STREAM("called get mission type");
            if(mission_type.compare("local") == 0)
                active_uavs[i]->ScoutBuilding(GetMissionLocal(file_name));
            else
                active_uavs[i]->ScoutBuilding(GetMissionGlobal(file_name));

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
    if(NUM_UAV == 0)
        return;

    for(int i = 0; i < NUM_UAV; i++)
        active_uavs[i]->StopMission();
    

    this->ToggleScoutButtons(true);
}

void GCS::OnScoutBuilding()
{
    if(NUM_UAV == 0)
        return;

    int building_index = widget.cmbo_box_buildings->currentIndex() + 1;
    std::string file_name = "building" + std::to_string(building_index) + ".txt";

    if(this->GetMissionType(file_name).compare("local") == 0)
        active_uavs[cur_uav]->ScoutBuilding(GetMissionLocal(file_name));
    else
        active_uavs[cur_uav]->ScoutBuilding(GetMissionGlobal(file_name));

    this->ToggleScoutButtons(false);

    ROS_INFO_STREAM("Scouting Building " << building_index);
}

void GCS::OnPauseOrResumeScout()
{
    if (NUM_UAV == 0)
        return;

    if(active_uavs[cur_uav]->GetMissionMode() == MissionMode::active)
    {
        active_uavs[cur_uav]->PauseMission();
        widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/play.png"));
    }
    else
    {
        active_uavs[cur_uav]->ResumeMission();
        widget.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/pause.png"));
    }
}

void GCS::OnStopScout()
{
    if(NUM_UAV == 0)
        return;

    active_uavs[cur_uav]->StopMission();
    this->ToggleScoutButtons(true);
}

void GCS::OnChangeFlightMode(int index)
{
    if(NUM_UAV == 0)
        return;
    
    if(index == 0)
    {
        ROS_INFO_STREAM("Quadrotor Stablized");
        active_uavs[cur_uav]->SetMode("STABILIZED");
    }
    else if(index == 1)
    {
        ROS_INFO_STREAM("Quadrotor Loiter");
        active_uavs[cur_uav]->SetMode("AUTO.LOITER");
    }
    else if(index == 2)
    {
        ROS_INFO_STREAM("Quadrotor Land");
        active_uavs[cur_uav]->SetMode("AUTO.LAND");
    }
    else if(index == 3)
    {
        ROS_INFO_STREAM("Altitude Hold");
        active_uavs[cur_uav]->SetMode("ALTCTL");
    }
    else if( index == 4)
    {
        ROS_INFO_STREAM("Position Hold");
        active_uavs[cur_uav]->SetMode("POSCTL");
    }
    else if(index == 5)
    {
        ROS_INFO_STREAM("Quadrotor Return-To-Launch");
        active_uavs[cur_uav]->SetRTL();
    }
    else if(index == 6)
    {
        ROS_INFO_STREAM("Quadrotor Auto");
        active_uavs[cur_uav]->SetMode("AUTO");
    }
    else if(index == 7)
    {
        ROS_INFO_STREAM("Quadrotor Offboard");
        active_uavs[cur_uav]->SetMode("OFFBOARD");
    }

    if(active_uavs[cur_uav]->GetMissionMode() != MissionMode::stopped)
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
void GCS::OnUavSelected(QWidget* w)
{
    SelectUav(widget.layout_uavs->indexOf(w));
}

//needed in addUav(int) and deleteUav(int)
void GCS::SelectUav(int uav_number)
{
    Q_ASSERT(0 <= uav_number && uav_number < NUM_UAV);

    if(uav_number == cur_uav)
        return; // no use resetting everything to the way it was

    cur_uav = uav_number;
    widget.image_frame->setPixmap(QPixmap::fromImage(QImage()));
    this->ClearQueries(); // new uav selected, so make room for its queries

    UAVControl * uav = active_uavs[cur_uav];
    // handle image topic subscription and image refresh for new uav
    sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav->id) + "/stereo_cam/left/image_rect",
                                     img_q_max_size, &GCS::ImageCallback, this);

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

    this->ToggleArmDisarmButton(active_uavs[cur_uav]->GetState().armed);
}

void GCS::UpdateFlightStateWidgets()
{
    if(NUM_UAV == 0)
        return;

    UAVControl *uav = active_uavs[cur_uav];

    // PFD
    widget.pfd->setRoll(uav->GetFlightState().roll*180);
    widget.pfd->setPitch(uav->GetFlightState().pitch*90);
    widget.pfd->setHeading(uav->GetFlightState().heading);
    widget.pfd->setAirspeed(uav->GetFlightState().ground_speed);
    widget.pfd->setAltitude(uav->GetFlightState().altitude);
    widget.pfd->setClimbRate(uav->GetFlightState().vertical_speed);
    widget.pfd->update();

    //text based widget
    temp_data.setNum(uav->GetFlightState().yaw, 'f', 2);
    widget.lbl_yaw_val->setText(temp_data);
    temp_data.setNum(uav->GetFlightState().roll, 'f', 2);
    widget.lbl_roll_val->setText(temp_data);
    temp_data.setNum(uav->GetFlightState().pitch, 'f', 2);
    widget.lbl_pitch_val->setText(temp_data);
    temp_data.setNum(uav->GetFlightState().ground_speed, 'f', 2);
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
    if(NUM_UAV == 0)
        return;

    bool armed = active_uavs[cur_uav]->GetState().armed;

    this->ToggleArmDisarmButton(armed);

    active_uavs[cur_uav]->Arm(!armed);
}

void GCS::ToggleArmDisarmButton(bool arm)
{
    if(arm)
        widget.btn_arm_uav->setText("Disarm");
    else
        widget.btn_arm_uav->setText("Arm");
}

void GCS::AdvertiseObjectDetection()
{
    od_handlers.pub_hit_thresh = nh.advertise<std_msgs::Float64>("/object_detection/hit_threshold", 5);
    od_handlers.pub_step_size = nh.advertise<std_msgs::Int32>("/object_detection/step_size", 5);
    od_handlers.pub_padding = nh.advertise<std_msgs::Int32>("/object_detection/padding", 5);
    od_handlers.pub_scale_factor = nh.advertise<std_msgs::Float64>("/object_detection/scale_factor", 5);
    od_handlers.pub_mean_shift = nh.advertise<std_msgs::Int32>("/object_detection/mean_shift_grouping", 5);
    od_handlers.sub_od_request = nh.subscribe("/object_detection/param_request", 5,
                                              &GCS::ReceivedObjectDetectionRequest, this);
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
    settings = new QSettings("SERL", "LCAR_Bot", this);

    settings->beginGroup("general_tab");

    if(settings->value("machine_learning", "online").toString() == "online")
        OnToggleMachineLearningMode(true);
    else
        OnToggleMachineLearningMode(false);

    QString default_path = QProcessEnvironment::systemEnvironment().value("HOME") % "/Pictures/LCAR_Bot";
    img::image_root_dir_ = settings->value("images_root_directory", default_path).toString();

    settings->endGroup();

    settings->beginGroup("object_detection_tab");

    QString params = "tuning_paramaters";
    od_params.hit_thresh = settings->value(params % "/hit_threshold", od_params.hit_thresh).toDouble();
    od_params.step_size = settings->value(params % "/step_size", od_params.step_size).toInt();
    od_params.padding = settings->value(params % "/padding", od_params.padding).toInt();
    od_params.scale_factor = settings->value(params % "/scale_factor", od_params.scale_factor).toDouble();
    od_params.mean_shift = settings->value(params % "/mean_shift_grouping", od_params.mean_shift).toBool();

    settings->endGroup();
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

        if(NUM_UAV > 0)
            fl_widgets.ap_menu->SetUAV(active_uavs[cur_uav]);
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
        fl_widgets.settings = new SettingsWidget(this);

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
                
        connect(fl_widgets.vehicle_init, &VehicleInitWidget::AddVehicleToDb,
                vm, &VehicleManager::OnOperatorInitRequested);
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
    for(int i = 0; i < NUM_UAV; i++)
    {
        VehicleWidget * vw = this->VehicleWidgetAt(i);
        int num = active_uavs[i]->GetBatteryState().percentage * 100;
        vw->SetBattery(num);
        temp_data = active_uavs[i]->GetState().mode.c_str();
        vw->SetCondition(temp_data);
    }
}

void GCS::OnUpdateCameraFeed()
{
    img_mutex.lock();

    //dequeue function assumes the queue isn't empty, so check first.
    if(img_q.size() > 0)
        widget.image_frame->setPixmap(img_q.dequeue());
    
    img_mutex.unlock();
}

void GCS::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    img_mutex.lock();
    
    QPixmap image = img::rosImgToQpixmap(msg);
    
    if(img_q.size() < img_q_max_size)
    {
        int w = widget.image_frame->width();
        int h = widget.image_frame->height();
        img_q.enqueue(image.scaled(w, h, Qt::KeepAspectRatio));
        emit NewCameraFeedFrame();
    }

    img_mutex.unlock();
}

void GCS::ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg)
{
    ROS_INFO_STREAM("received object detection paramater request");
    this->PublishHitThreshold(od_params.hit_thresh);
    this->PublishStepSize(od_params.step_size);
    this->PublishPadding(od_params.padding);
    this->PublishScaleFactor(od_params.scale_factor);
    this->PublishMeanShift(od_params.mean_shift);
}

void GCS::PublishHitThreshold(double thresh)
{
    std_msgs::Float64 msg;
    msg.data = thresh;
    od_handlers.pub_hit_thresh.publish(msg);
}

void GCS::PublishStepSize(int step)
{
    std_msgs::Int32 msg;
    msg.data = step;
    od_handlers.pub_step_size.publish(msg);
}

void GCS::PublishPadding(int padding)
{
    std_msgs::Int32 msg;
    msg.data = padding;
    od_handlers.pub_padding.publish(msg);
}

void GCS::PublishScaleFactor(double scale)
{
    std_msgs::Float64 msg;
    msg.data = scale;
    od_handlers.pub_scale_factor.publish(msg);
}

void GCS::PublishMeanShift(bool on)
{
    std_msgs::Int32 msg;
    msg.data = on;
    od_handlers.pub_mean_shift.publish(msg);
}

void GCS::closeEvent(QCloseEvent* event)
{
    thread_uav_monitor->Stop();
    thread_uav_monitor->wait();
    delete thread_uav_monitor;
    
    for(int i = NUM_UAV - 1; i >= 0; i--)
        this->OnDeleteUav(i);
    
    // this wouldn't work in the destructor
    if(fl_widgets.ap_menu != nullptr) 
        fl_widgets.ap_menu->close();
    
    if(fl_widgets.settings != nullptr)
        fl_widgets.settings->close();
    
    if(fl_widgets.unanswered_queries != nullptr)
        fl_widgets.unanswered_queries->close();
    
    event->accept();
}

VehicleWidget* GCS::VehicleWidgetAt(int index)
{
    return (VehicleWidget*) widget.layout_uavs->itemAt(index)->widget();
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
        int index = node.find("/UAV");
        if(index == node.npos)
        {   //nodes are printed alphabetically, if we passed UAV namespace, there are no more
            if(map.size() > 0)
                break;
            else  // skip nodes in front of /UAV namespace
                continue;
        }

        std::string id_string = node.substr(index+4, 3);
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
        VehicleWidget * vw = gcs->VehicleWidgetAt(i);
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
        MonitorUavNamespace();
        MonitorUavConnections();
        RunUavs();  
    }
}

void GCSHelperThread::Stop()
{
    this->requestInterruption();
}

} // namespace
