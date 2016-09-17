#include "rqt_gcs/simple_gcs.h"
#include "util/image.h"
#include "util/debug.h"

#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <fstream>

#include <QProcessEnvironment>
#include <QDesktopWidget>
#include <QMainWindow>
#include <QStringBuilder>
#include <QMetaType>

namespace rqt_gcs
{
    
    namespace image_util
    {
        QString image_root_dir_; // defined globally in image_utility.h
    }

    SimpleGCS::SimpleGCS()
    : rqt_gui_cpp::Plugin()
    , central_widget_(nullptr)
    {
        //Constructor is called first before initPlugin function
        setObjectName("LCAR Bot GCS");
    }
    
    SimpleGCS::~SimpleGCS()
    {
    }
    
    void SimpleGCS::shutdownPlugin()
    {
        update_timer->stop();
        delete update_timer;
        ros::shutdown();
    }

    void SimpleGCS::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void SimpleGCS::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }

    /*bool hasConfiguration() const
    {
      return true;
    }

    void triggerConfiguration()
    {
      // Usually used to open a dialog to offer the user a set of configuration
    }*/
    
    void SimpleGCS::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        //QStringList argv = context.argv();
        
        NUM_UAV = 0;
        timeCounter = 0;
        cur_uav = -1;
        num_queries_last = 0;

        //Setup the UI objects with the widgets
        central_widget_ = new QWidget();
        central_ui_.setupUi(central_widget_);

        //Add widgets to the Main UI
        context.addWidget(central_widget_);

        //setup button logic for the widgets
        connect(central_ui_.btn_exec_play, &QPushButton::clicked, this, &SimpleGCS::OnExecutePlay);
        connect(central_ui_.btn_scout, &QPushButton::clicked, this, &SimpleGCS::OnScoutBuilding);
        connect(central_ui_.btn_scout_play_pause, &QPushButton::clicked, this, &SimpleGCS::OnPauseOrResumeScout);
        connect(central_ui_.btn_scout_stop, &QPushButton::clicked, this, &SimpleGCS::OnStopScout);
        connect(central_ui_.cmbo_box_flight_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(OnChangeFlightMode(int)));
        connect(central_ui_.btn_view_acess_points, &QPushButton::clicked, this, &SimpleGCS::OnAccessPointsTriggered);
        connect(central_ui_.btn_arm_uav, &QPushButton::clicked, this, &SimpleGCS::OnArmOrDisarmSelectedUav);
        connect(this, &SimpleGCS::NewCameraFeedFrame, this, &SimpleGCS::OnUpdateCameraFeed);
        
        
        //Setup UAV lists select functions
        uav_select_mapper = new QSignalMapper(this);
        accept_door_mapper = new QSignalMapper(this);
        deny_door_mapper = new QSignalMapper(this);
        
        connect(uav_select_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(OnUavSelected(QWidget*)));
        connect(accept_door_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(OnAcceptDoorQuery(QWidget*)));
        connect(deny_door_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(OnRejectDoorQuery(QWidget*)));
        
        od_handlers.pub_hit_thresh = nh.advertise<std_msgs::Float64>("/object_detection/hit_threshold", 1);
        od_handlers.pub_step_size = nh.advertise<std_msgs::Int32>("/object_detection/step_size", 1);
        od_handlers.pub_padding = nh.advertise<std_msgs::Int32>("/object_detection/padding", 1);
        od_handlers.pub_scale_factor = nh.advertise<std_msgs::Float64>("/object_detection/scale_factor", 1);
        od_handlers.pub_mean_shift = nh.advertise<std_msgs::Int32>("/object_detection/mean_shift_grouping", 1);
        od_handlers.sub_od_request = nh.subscribe("/object_detection/param_request", 2,
                                                  &SimpleGCS::ReceivedObjectDetectionRequest, this);
        
        InitDbg();
        this->InitMenuBar();
        this->InitSettings();
        this->InitHelperThread();
        this->InitMap();
        
        this->ToggleScoutButtons(true);
        
        //Setup update timer
        update_timer = new QTimer(this);
        connect(update_timer, &QTimer::timeout, this, &SimpleGCS::OnTimedUpdate);
        //30 hz :1000/30 = 33.33...
        update_timer->start(0);
    }
    
    void SimpleGCS::OnAddUav(int uav_id)
    {
        uav_mutex.lock();

        UAVControl * uav = new UAVControl(uav_id);
        
        //todo loop and get num_image for each acess point type;
        //create separate accepted and rejected counts for each ap_type
        QString path = image_util::image_root_dir_ + "/queries/accepted/door/uav_" + QString::number(uav_id);
        uav->accepted_images = image_util::numImagesInDir(path);
        
        path = image_util::image_root_dir_ % "/queries/unanswered/door/uav_" % QString::number(uav_id);
        uav->rejected_images = image_util::numImagesInDir(path);
        
        qCWarning(lcar_bot) << "Adding UAV with id:" << uav_id;

        temp_data = "UAV " + QString::number(uav_id);

        int index = 0;
        while(index < NUM_UAV && uav_id > active_uavs[index]->id)
            index++;

        uav_db.insert(uav_id, uav);
        active_uavs.insert(active_uavs.begin() + index, uav);
        vec_uav_list_ui_.insert(vec_uav_list_ui_.begin() + index, new Ui::UAVConditionWidget());
        vec_uav_list_widget_.insert(vec_uav_list_widget_.begin() + index, new QWidget());
        
        vec_uav_list_ui_[index]->setupUi(vec_uav_list_widget_[index]);
        vec_uav_list_ui_[index]->VehicleSelectButton->setText(QString::number(uav_id));
        vec_uav_list_ui_[index]->VehicleNameLine->setText(temp_data);
        
        uav_select_mapper->setMapping(vec_uav_list_ui_[index]->VehicleSelectButton, vec_uav_list_widget_[index]);
        connect(vec_uav_list_ui_[index]->VehicleSelectButton, SIGNAL(clicked()),
               uav_select_mapper, SLOT(map()));
        central_ui_.layout_uavs->insertWidget(index, vec_uav_list_widget_[index]);

        NUM_UAV++;

        if(NUM_UAV == 1)
            SelectUav(0);

        num_uav_changed.wakeAll();
        uav_mutex.unlock();
    }

    void SimpleGCS::OnDeleteUav(int index)
    {
        uav_mutex.lock();

        UAVControl * uav = active_uavs[index];
        int uav_id = uav->id;
        qCWarning(lcar_bot) << "Deleting UAV with id: " << uav_id;

        central_ui_.layout_uavs->removeWidget(vec_uav_list_widget_[index]);

        SaveUavQueries(uav->id, uav->GetDoorQueries(), "door");
        
        // dont delete uav_db[index] as it points to same SimpleControl object as active_uavs
        delete uav;
        delete vec_uav_list_ui_[index];
        delete vec_uav_list_widget_[index];

        vec_uav_list_ui_.erase(vec_uav_list_ui_.begin() + index);
        vec_uav_list_widget_.erase(vec_uav_list_widget_.begin() + index);
        active_uavs.erase(active_uavs.begin() + index);
        uav_db.erase(uav_db.begin() + index); // erase 
        
        NUM_UAV--;
        
        if(cur_uav == index) // deleted current uav
        {  
            if(NUM_UAV > 0 && cur_uav == 0) // if there's more and we deleted the first
                SelectUav(0);       // choose the new first
            else                            // else
                SelectUav(cur_uav - 1); // choose one in front
        }

       num_uav_changed.wakeAll();
       uav_mutex.unlock();
    }

    void SimpleGCS::SaveUavQueries(int uav_id, const std::vector<lcar_msgs::DoorPtr> *queries, const QString ap_type)
    {
        QString path = image_util::image_root_dir_ % "/queries/unanswered/" 
            % ap_type % "/uav_" % QString::number(uav_id);
        
        int num_images = image_util::numImagesInDir(path);
        
//        const std::vector<lcar_msgs::DoorPtr>* queries = uav->GetDoorQueries();
        for(int i = 0; i < queries->size(); i++, num_images += 2)
        {
            lcar_msgs::DoorPtr query = queries->at(i);

            sensor_msgs::Image ros_image = query->original_picture;
            QImage image = image_util::rosImgToQimg(ros_image);
            
            QString file = "img_" % QString::number(num_images) % ".jpg";
            if(!image_util::saveImage(path, file, image))
                std::cout  << "error saving uav query\n";

            ros_image = query->framed_picture;
            image = image_util::rosImgToQimg(ros_image);
            
            file = "img_" %  QString::number(num_images+1) % ".jpg";
            if(!image_util::saveImage(path, file, image))
                std::cout  << "error saving uav query\n";
        }
    }

    void SimpleGCS::OnUAVConnectionToggled(int index, int uav_id, bool toggle)
    {
        if(index >= active_uavs.size() || active_uavs[index]->id != uav_id)
            return;
        
        QWidget* button = vec_uav_list_ui_[index]->VehicleSelectButton;
        button->setEnabled(toggle);

        QString style_sheet = button->isEnabled() ?
            "background-color: rgb(64, 89, 140); color: rgb(240, 240, 240);" :
            "background-color: rgb(80, 90, 110); color: rgb(150, 150, 150);" ;

        button->setStyleSheet(style_sheet);
    }

    //Timed update of GCS gui
    void SimpleGCS::OnTimedUpdate()
    {
        if(NUM_UAV == 0)
            return;
        
        if(timeCounter++ >= 30)
        {
            UpdateQueries();
            timeCounter = 0;
        }
        
        this->UpdateFlightStateWidgets();
        this->UpdateVehicleWidgets();
    }

    void SimpleGCS::ClearQueries()
    {
        int size = query_widgets_.size();
        for(int i = 0; i < size; i++)
        {
            central_ui_.layout_queries->removeWidget(query_widgets_[i]);
            delete query_widgets_[i];
        }
        query_widgets_.clear();
        num_queries_last = 0;
    }


    void SimpleGCS::UpdateQueries()
    {
        if(NUM_UAV == 0 || !central_ui_.frame_queries_cntnr->isVisible())
            return;

        vec_uav_queries_ = active_uavs[cur_uav]->GetDoorQueries();
        int pqv_size = vec_uav_queries_->size();
        int new_queries = pqv_size - num_queries_last;

        for(int i = 0; i < new_queries; i++)
        {
            //retrieve Query msg for door image
            lcar_msgs::DoorPtr doorQuery = vec_uav_queries_->at(i + num_queries_last);

            QImage image = image_util::rosImgToQimg(doorQuery->framed_picture);

            //create the widget
            QWidget * pmWidget = new QWidget();
            Ui::PictureMsgWidget pmUiWidget;
            pmUiWidget.setupUi(pmWidget);

            int w = pmUiWidget.image_frame->width();
            int h = pmUiWidget.image_frame->height();
            pmUiWidget.image_frame->setPixmap(QPixmap::fromImage(image).scaled(w,h));

            //add to list
            query_widgets_.push_back(pmWidget);
            int index = query_widgets_.size() - 1;

            //map signal for yes button
            accept_door_mapper->setMapping(pmUiWidget.yesButton,
                                         query_widgets_[index]);
            connect(pmUiWidget.yesButton, SIGNAL(clicked()),
                    accept_door_mapper, SLOT(map()));

            //map signal for yes button
            deny_door_mapper->setMapping(pmUiWidget.rejectButton,
                                       query_widgets_[index]);
            connect(pmUiWidget.rejectButton, SIGNAL(clicked()),
                    deny_door_mapper, SLOT(map()));

            //add to user interface
            central_ui_.layout_queries->addWidget(query_widgets_[index]);
        }

        num_queries_last = pqv_size;
    }

    void SimpleGCS::AnswerQuery(QWidget * qw, QString ap_type, bool accepted)
    {
        int index = central_ui_.layout_queries->indexOf(qw);
        lcar_msgs::DoorPtr door = vec_uav_queries_->at(index);

        UAVControl* uav = active_uavs[cur_uav];
        QString path = image_util::image_root_dir_ % "/queries";
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

        image_util::saveImage(path, file, door->original_picture);

        vec_uav_queries_->erase(vec_uav_queries_->begin() + index);
        query_widgets_.erase(query_widgets_.begin() + index);
        central_ui_.layout_queries->removeWidget(qw);
        delete qw;

        door->accepted = accepted;
        num_queries_last--;
        //UAVs[cur_uav]->SendDoorResponse(doormsg);
    }

    void SimpleGCS::OnAcceptDoorQuery(QWidget *qw)
    {
        AnswerQuery(qw, "door", true);
    }

    void SimpleGCS::OnRejectDoorQuery(QWidget * qw)
    {
        AnswerQuery(qw, "door", false);
    }

    void SimpleGCS::OnExecutePlay()
    {
        if(NUM_UAV == 0)
            return;

        int play_num = central_ui_.cmbo_box_play_book->currentIndex();
        
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

    void SimpleGCS::OnCancelPlay()
    {
        if(NUM_UAV == 0)
            return;

        for(int i = 0; i < NUM_UAV; i++)
        {
            active_uavs[i]->StopMission();
        }
        
        this->ToggleScoutButtons(true);
    }

    void SimpleGCS::OnScoutBuilding()
    {
        if(NUM_UAV == 0)
            return;

        int building_index = central_ui_.cmbo_box_buildings->currentIndex() + 1;
        std::string file_name = "building" + std::to_string(building_index) + ".txt";

        if(this->GetMissionType(file_name).compare("local") == 0) 
            active_uavs[cur_uav]->ScoutBuilding(GetMissionLocal(file_name));
        else 
            active_uavs[cur_uav]->ScoutBuilding(GetMissionGlobal(file_name));

        this->ToggleScoutButtons(false);
        
        ROS_INFO_STREAM("Scouting Building " << building_index);
    }

    void SimpleGCS::OnPauseOrResumeScout()
    {
        if (NUM_UAV == 0)
            return;
        
        if(active_uavs[cur_uav]->GetMissionMode() == MissionMode::active)
        {
            active_uavs[cur_uav]->PauseMission();
            central_ui_.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/play.png"));
        }
        else
        {
            active_uavs[cur_uav]->ResumeMission();
            central_ui_.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/pause.png"));
        }
    }    
    
    void SimpleGCS::OnStopScout()
    {
        if(NUM_UAV == 0)
            return;

        active_uavs[cur_uav]->StopMission();
        this->ToggleScoutButtons(true);
    }

    void SimpleGCS::OnChangeFlightMode(int index)
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
        {
            this->OnStopScout();
        }
    }

    void SimpleGCS::ToggleScoutButtons(bool visible, QString icon_type)
    { // icon_type should be "play" or "pause"
        central_ui_.btn_scout->setVisible(visible);
        central_ui_.btn_scout_play_pause->setVisible(!visible);
        central_ui_.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/"%icon_type%".png"));
        central_ui_.btn_scout_stop->setVisible(!visible);     
    }

    void SimpleGCS::OnAccessPointsTriggered()
    {
        if(fl_widgets_.ap_menu == nullptr)
        {
            fl_widgets_.ap_menu = new AccessPoints();
           
            QRect window = central_widget_->window()->geometry();
            int x = (central_widget_->width() / 2) - (fl_widgets_.ap_menu->width() / 2);
            int y = (central_widget_->height() / 2) - (fl_widgets_.ap_menu->height() / 2);
            fl_widgets_.ap_menu->move(window.x() + x, window.y() + y);
            fl_widgets_.ap_menu->setVisible(true);
            
            connect(fl_widgets_.ap_menu, &AccessPoints::destroyed, 
                    this, [=](){ fl_widgets_.ap_menu = nullptr; });
                    
            if(NUM_UAV > 0)
                fl_widgets_.ap_menu->SetUAV(active_uavs[cur_uav]);
            else
                fl_widgets_.ap_menu->SetUAV(nullptr);
        }
        else   
            fl_widgets_.ap_menu->showNormal();
            fl_widgets_.ap_menu->activateWindow();
    }

    // slot gets called when user click on a uav button
    void SimpleGCS::OnUavSelected(QWidget* w)
    {
        SelectUav(central_ui_.layout_uavs->indexOf(w));
    }

    //needed in addUav(int) and deleteUav(int)
    void SimpleGCS::SelectUav(int uav_number)
    {
        if(uav_number == cur_uav) 
            return; // no use resetting everything to the way it was
        
        cur_uav = uav_number;
        central_ui_.image_frame->setPixmap(QPixmap::fromImage(QImage()));
        this->ClearQueries(); // new uav selected, so make room for its queries
        
        if(cur_uav == -1) // no more uav's in system. 
        {
            if(fl_widgets_.ap_menu != nullptr)
                fl_widgets_.ap_menu->SetUAV(nullptr); 
            this->ToggleScoutButtons(true);  // reset scout buttons
            this->ToggleArmDisarmButton(false); //set arm disarm button to arm
            central_ui_.lbl_cur_uav->setText("NO UAVS");
        }
        else
        {
            UAVControl * uav = active_uavs[cur_uav];
            // handle image topic subscription and image refresh for new uav
            sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav->id) + "/stereo_cam/left/image_rect",
                                             img_q_max_size, &SimpleGCS::ImageCallback, this);

            if(fl_widgets_.ap_menu != nullptr)
                fl_widgets_.ap_menu->SetUAV(uav); 
            
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
    }
    
    void SimpleGCS::UpdateFlightStateWidgets()
    {
        if(NUM_UAV == 0)
            return;
        
        UAVControl* uav = active_uavs[cur_uav];
        
        // PFD
        central_ui_.graphicsPFD->setRoll(uav->GetFlightState().roll*180);
        central_ui_.graphicsPFD->setPitch(uav->GetFlightState().pitch*90);
        central_ui_.graphicsPFD->setHeading(uav->GetFlightState().heading);
        central_ui_.graphicsPFD->setAirspeed(uav->GetFlightState().ground_speed);
        central_ui_.graphicsPFD->setAltitude(uav->GetFlightState().altitude);
        central_ui_.graphicsPFD->setClimbRate(uav->GetFlightState().vertical_speed);
        central_ui_.graphicsPFD->update();
        
        //text based widget
        temp_data.setNum(uav->GetFlightState().yaw, 'f', 2);
        central_ui_.lbl_yaw_val->setText(temp_data);
        temp_data.setNum(uav->GetFlightState().roll, 'f', 2);
        central_ui_.lbl_roll_val->setText(temp_data);
        temp_data.setNum(uav->GetFlightState().pitch, 'f', 2);
        central_ui_.lbl_pitch_val->setText(temp_data);
        temp_data.setNum(uav->GetFlightState().ground_speed, 'f', 2);
        temp_data.append(" m/s");
        central_ui_.lbl_gnd_spd_val->setText(temp_data);
        temp_data.setNum(uav->GetDistanceToWP());
        temp_data.append(" m");
        central_ui_.lbl_dist_wp_val->setText(temp_data);
        temp_data.setNum(uav->GetBatteryState().percentage * 100);
        central_ui_.pgs_bar_battery->setValue(temp_data.toInt());
        temp_data = "UAV " + QString::number(uav->id);
        central_ui_.lbl_cur_uav->setText(temp_data);
        central_ui_.pgs_bar_mission->setValue(uav->GetMissionProgress() * 100);
    }

    void SimpleGCS::OnArmOrDisarmSelectedUav()
    {
        if(NUM_UAV == 0)
            return;
        
        bool armed = active_uavs[cur_uav]->GetState().armed;
       
        this->ToggleArmDisarmButton(armed);
        
        active_uavs[cur_uav]->Arm(!armed);        
    }
    
    void SimpleGCS::ToggleArmDisarmButton(bool arm)
    {
        if(arm)
            central_ui_.btn_arm_uav->setText("Disarm");
        else
            central_ui_.btn_arm_uav->setText("Arm");
    }

    void SimpleGCS::InitHelperThread()
    {
        uav_monitor = new SimpleGCSHelper(this);
        uav_monitor->moveToThread(&t_uav_monitor);

        connect(&t_uav_monitor, &QThread::started,
                uav_monitor, &SimpleGCSHelper::help);

        connect(uav_monitor, &SimpleGCSHelper::addUav,
                this, &SimpleGCS::OnAddUav);

        connect(uav_monitor, &SimpleGCSHelper::deleteUav,
                this, &SimpleGCS::OnDeleteUav);

        connect(uav_monitor, &SimpleGCSHelper::toggleUavConnection,
                this, &SimpleGCS::OnUAVConnectionToggled);

        t_uav_monitor.start();
    }
    
    void SimpleGCS::InitMap()
    {
        QString s = ros::package::getPath("rqt_gcs").c_str();         
        QString map_url = "file://" % s % "/map/uavmap.html";
        central_ui_.web_view->load(QUrl(map_url));
    }
    
    void SimpleGCS::InitMenuBar()
    {
       menu_bar_ = new QMenuBar(central_widget_);

       //file menu
       file_menu = menu_bar_->addMenu("File");
       start_uav_act = file_menu->addAction("Start UAV");
       start_uav_group_act = file_menu->addAction("Start UAV Group");
       shutdown_uav_act = file_menu->addAction("Shutdown UAV");
       shutdown_uav_group_act = file_menu->addAction("Shutdown UAV Group");
       
       //view menu
       view_menu = menu_bar_->addMenu("View");
       unanswered_queries_act = view_menu->addAction("Unanswered Queries");
       connect(unanswered_queries_act, &QAction::triggered, 
               this, &SimpleGCS::OnUnansweredQueriesTriggered);
       
       //tools menu
       tools_menu = menu_bar_->addMenu("Tools");
       settings_act = tools_menu->addAction("Settings");
       connect(settings_act, &QAction::triggered, 
               this, &SimpleGCS::OnSettingsTriggered); 
       
       //help menu
       help_menu = menu_bar_->addMenu("Help");
       lcar_bot_act = help_menu->addAction("Learning Classifying And Recognizing Bot (LCAR-Bot)");
       ros_act      = help_menu->addAction("Robot Operating System (ROS)");
       opencv_act   = help_menu->addAction("Open Computer Vision (OpenCV)");
       qt_act       = help_menu->addAction("Qt");
       about_act    = help_menu->addSection("About");
       
       menu_bar_->adjustSize();
       menu_bar_->setVisible(true);
    }

    // SettingsWidget and QSettings related stuff
    void SimpleGCS::InitSettings()
    {
        //initialize settings and uav queries display
        settings_ = new QSettings("SERL", "LCAR_Bot");

        settings_->beginGroup("general_tab");

        if(settings_->value("machine_learning", "online").toString() == "online")
            OnToggleMachineLearningMode(true);
        else
            OnToggleMachineLearningMode(false);

        QString default_path = QProcessEnvironment::systemEnvironment().value("HOME") % "/Pictures/LCAR_Bot";
        image_util::image_root_dir_ = settings_->value("images_root_directory", default_path).toString();
        qCDebug(lcar_bot) << "images root directory: " << image_util::image_root_dir_;

        settings_->endGroup();
        
        settings_->beginGroup("object_detection_tab");
         
        QString params = "tuning_paramaters";
        od_params.hit_thresh = settings_->value(params % "/hit_threshold", od_params.hit_thresh).toDouble();    
        od_params.step_size = settings_->value(params % "/step_size", od_params.step_size).toInt();
        od_params.padding = settings_->value(params % "/padding", od_params.padding).toInt();
        od_params.scale_factor = settings_->value(params % "/scale_factor", od_params.scale_factor).toDouble();
        od_params.mean_shift = settings_->value(params % "/mean_shift_grouping", od_params.mean_shift).toBool();

        settings_->endGroup();
    }

    void SimpleGCS::OnSettingsTriggered()
    {
        if(fl_widgets_.settings == nullptr)
        {
            fl_widgets_.settings = new SettingsWidget(this);
            
            QRect window = central_widget_->window()->geometry();
            int x = (central_widget_->width() / 2) - (fl_widgets_.settings->width() / 2);
            int y = (central_widget_->height() / 2) - (fl_widgets_.settings->height() / 2);
            fl_widgets_.settings->move(window.x() + x, window.y() + y);
            fl_widgets_.settings->setVisible(true);

            connect(fl_widgets_.settings, &SettingsWidget::machineLearningModeToggled,
                    this, &SimpleGCS::OnToggleMachineLearningMode);
            
            connect(fl_widgets_.settings, &SettingsWidget::destroyed, 
                    this, [=](){ fl_widgets_.settings = nullptr; });
        }
        else
        {
            fl_widgets_.settings->showNormal();
            fl_widgets_.settings->activateWindow();
        }
    }
    
    void SimpleGCS::OnUnansweredQueriesTriggered()
    {
        if(fl_widgets_.unanswered_queries == nullptr)
        {
            fl_widgets_.unanswered_queries = new UnansweredQueries(this);
            
            QRect window = central_widget_->window()->geometry();
            int x = (central_widget_->width() / 2) - (fl_widgets_.unanswered_queries->width() / 2);
            int y = (central_widget_->height() / 2) - (fl_widgets_.unanswered_queries->height() / 2);
            fl_widgets_.unanswered_queries->move(window.x() + x, window.y() + y);
            fl_widgets_.unanswered_queries->setVisible(true);
            
            connect(fl_widgets_.unanswered_queries, &UnansweredQueries::destroyed,
                    this, [=](){ fl_widgets_.unanswered_queries = nullptr; });
        }
        else
        {
            fl_widgets_.unanswered_queries->showNormal();
            fl_widgets_.unanswered_queries->activateWindow();
        }
    }

    void SimpleGCS::OnToggleMachineLearningMode(bool toggle)
    {
        central_ui_.frame_queries_cntnr->setVisible(toggle);
        central_ui_.frame_queries_cntnr->setEnabled(toggle);

        for(int i = 0; i < NUM_UAV; i++)
            active_uavs[i]->SetOnlineMode(toggle);
    }
    
    std::string SimpleGCS::GetMissionType(std::string file_name)
    {   
        std::string mission_type;
        std::ifstream fileIn(file_name);

        fileIn >> mission_type;
        fileIn.close();
        
        return mission_type;
    }

    lcar_msgs::TargetLocal SimpleGCS::GetMissionLocal(std::string file_name)
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

    lcar_msgs::TargetGlobal SimpleGCS::GetMissionGlobal(std::string file_name)
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
    
    void SimpleGCS::UpdateVehicleWidgets()
    {
        for(int i = 0; i < NUM_UAV; i++)
        {
            temp_data.setNum(active_uavs[i]->GetBatteryState().percentage * 100);
            vec_uav_list_ui_[i]->VehicleBatteryLine->setText(temp_data);
            temp_data = active_uavs[i]->GetState().mode.c_str();
            vec_uav_list_ui_[i]->VehicleConditionLine->setText(temp_data);
        }
    }
    
    void SimpleGCS::OnUpdateCameraFeed()
    {
        img_mutex.lock();
        
        //dequeue function assumes the queue isn't empty, so check first.
        if(img_q.size() > 0)
            central_ui_.image_frame->setPixmap(img_q.dequeue());
        
        img_mutex.unlock();
    }
    
    void SimpleGCS::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        img_mutex.lock();
        
        QImage image = image_util::rosImgToQimg(msg);

        if(img_q.size() < img_q_max_size)
        {
            int w = central_ui_.image_frame->width();
            int h = central_ui_.image_frame->height();
            img_q.enqueue(QPixmap::fromImage(image).scaled(w, h, Qt::KeepAspectRatio));
            emit NewCameraFeedFrame();
        }
        
        img_mutex.unlock();
    }
    
    void SimpleGCS::ReceivedObjectDetectionRequest(const std_msgs::Int32ConstPtr& msg)
    {
        ROS_INFO_STREAM("received object detection paramater request");
        this->PublishHitThreshold(od_params.hit_thresh);
        this->PublishStepSize(od_params.step_size);
        this->PublishPadding(od_params.padding);
        this->PublishScaleFactor(od_params.scale_factor);
        this->PublishMeanShift(od_params.mean_shift);
    }

    void SimpleGCS::PublishHitThreshold(double thresh)
    {
        std_msgs::Float64 msg;
        msg.data = thresh;
        od_handlers.pub_hit_thresh.publish(msg);
    }
    
    void SimpleGCS::PublishStepSize(int step)
    {
        std_msgs::Int32 msg;
        msg.data = step;
        od_handlers.pub_step_size.publish(msg);
    }
    
    void SimpleGCS::PublishPadding(int padding)
    {
        std_msgs::Int32 msg;
        msg.data = padding;
        od_handlers.pub_padding.publish(msg);
    }
    
    void SimpleGCS::PublishScaleFactor(double scale)
    {
        std_msgs::Float64 msg;
        msg.data = scale;
        od_handlers.pub_scale_factor.publish(msg);
    }
    
    void SimpleGCS::PublishMeanShift(bool on)
    {
        std_msgs::Int32 msg;
        msg.data = on;
        od_handlers.pub_mean_shift.publish(msg); 
    }
    
    ////////////////////////////////////////////////////////////////////////////
    //////////////////////////  SimpleGCSHelper  ///////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    
    SimpleGCSHelper::SimpleGCSHelper(SimpleGCS * sgcs) :
        gcs(sgcs)
    { }

    SimpleGCSHelper::~SimpleGCSHelper()
    {
        gcs = nullptr;
    }

    void SimpleGCSHelper::parseUavNamespace(std::map<int, int>& map)
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

    void SimpleGCSHelper::monitorUavNamespace()
    {
        std::map<int,int> uav_map;
        parseUavNamespace(uav_map);

        gcs->uav_mutex.lock();

        if(gcs->NUM_UAV < uav_map.size()) // uav added 
        {
            for(auto const& uav : uav_map)
            {
                if(!gcs->uav_db.contains(uav.first))
                {
                    emit addUav(uav.first);
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
                    emit deleteUav(i);
                    gcs->num_uav_changed.wait(&gcs->uav_mutex);
                    i--;
                }
            }
        }
        
        gcs->uav_mutex.unlock();
    }

    void SimpleGCSHelper::monitorUavConnections()
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
            QWidget* button = gcs->vec_uav_list_ui_[i]->VehicleSelectButton;

            if(!uav->RecievedHeartbeat()) // no heartbeat
            {
                //ROS_WARN_STREAM("no heartbeat for UAV_" << quad->id);
                if(button->isEnabled()) // is the button already disabled?
                    emit toggleUavConnection(i, uav->id, false);
            }
            else
            {
                //ROS_INFO_STREAM("recieved heartbeat for UAV_" << quad->id);
                if(!button->isEnabled()) // is the button already enabled?
                    emit toggleUavConnection(i, uav->id, true);
            }
        }
    }

    void SimpleGCSHelper::runUavs()
    {
        for(auto const& uav : gcs->active_uavs)
        {
            uav->Run();
        }
    }

    void SimpleGCSHelper::help()
    {
        forever
        {
            monitorUavNamespace();
            monitorUavConnections();
            runUavs();
        }
    }
    
} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, SimpleGCS, rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
////PLUGINLIB_EXPORT_CLASS(rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
