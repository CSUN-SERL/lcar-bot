#include <rqt_gcs/simple_gcs.h>
//
//
namespace rqt_gcs
{

    SimpleGCS::SimpleGCS()
    : rqt_gui_cpp::Plugin()
    , widget_main_(nullptr)
    {
        //Constructor is called first before initPlugin function
        setObjectName("LCAR Bot GCS");
        qRegisterMetaType<UavStatus>("UavStatus");
    }

    void SimpleGCS::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        //QStringList argv = context.argv();

        widget_main_           = new QWidget();
        uavQuestionWidget_     = new QWidget();
        apmQWidget_            = new QWidget();

        NUM_UAV = 0;
        timeCounter = 0;
        cur_uav = -1;
        num_queries_last = 0;
        num_access_points_last = 0;

        //Setup the UI objects with the widgets
        central_ui_.setupUi(widget_main_);
        apmUi_.setupUi(apmQWidget_);

        //Add widgets to the Main UI
        context.addWidget(widget_main_);

        //setup button logic for the widgets
        connect(central_ui_.btn_exec_play, SIGNAL(clicked()), this, SLOT(executePlay()));
        connect(central_ui_.btn_scout, SIGNAL(clicked()), this, SLOT(scoutBuilding()));
        connect(central_ui_.btn_scout_play_pause, SIGNAL(clicked()), this, SLOT(pauseOrResumeScout()));
        connect(central_ui_.btn_scout_stop, SIGNAL(clicked()), this, SLOT(stopScout()));
        connect(central_ui_.cmbo_box_flight_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(changeFlightMode(int)));
        connect(central_ui_.btn_view_acess_points, SIGNAL(clicked()), this, SLOT(showAccessPoints()));
        connect(central_ui_.btn_arm_uav, SIGNAL(clicked()), this, SLOT(armOrDisarmSelectedUav()));

        //Setup UAV lists select functions
        quad_select_mapper = new QSignalMapper(this);
        access_point_mapper = new QSignalMapper(this);
        acceptDoorMapper = new QSignalMapper(this);
        denyDoorMapper = new QSignalMapper(this);
        
        connect(quad_select_mapper, SIGNAL(mapped(int)), this, SLOT(uavSelected(int)));
        connect(access_point_mapper, SIGNAL(mapped(QWidget*)), this, SLOT(deleteAccessPoint(QWidget*)));
        connect(acceptDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(acceptDoorQuery(QWidget*)));
        connect(denyDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(rejectDoorQuery(QWidget*)));

        initMenuBar();
        initSettings();
        initHelperThread();
        initMap();
        
        od_pubs.pub_hit_thresh = nh.advertise<std_msgs::Float64>("/object_detection/hit_threshold", 1);
        od_pubs.pub_step_size = nh.advertise<std_msgs::Int32>("/object_detection/step_size", 1);
        od_pubs.pub_padding = nh.advertise<std_msgs::Int32>("/object_detection/padding", 1);
        od_pubs.pub_scale_factor = nh.advertise<std_msgs::Float64>("/object_detection/scale_factor", 1);
        od_pubs.pub_mean_shift = nh.advertise<std_msgs::Int32>("/object_detection/mean_shift_grouping", 1);
        
        this->toggleScoutButtons(true);
        
        //Setup update timer
        update_timer = new QTimer(this);
        connect(update_timer, SIGNAL(timeout()), this, SLOT(timedUpdate()));
        //30 hz :1000/30 = 33.33...
        update_timer->start(33);
    }

    void SimpleGCS::addUav(int uav_id)
    {
        uav_mutex.lock();

        SimpleControl * uav;
        UavStatus status = uav_db[uav_id]->status;
        if(status == UavStatus::null || status == UavStatus::purged)
            uav = new SimpleControl(uav_id);
        else if(status == UavStatus::deleted && deleted_uavs[uav_id] != nullptr)
            uav = deleted_uavs[uav_id];
        else
        {
            ROS_ERROR_STREAM("Error adding UAV with id:" << uav_id
                    << "\n all_uavs[uav_id]=" << uav_db[uav_id]
                    << "\n deleted_uavs[uav_id]=" << deleted_uavs[uav_id]);
            exit(1);
        }
        
        //todo loop and get num_image for each acess point type;
        //create separate accepted and rejected counts for each ap_type
        QString path = image_root_dir_ + "/queries/accepted/door/uav_" + QString::number(uav_id);
        uav->accepted_images = UnansweredQueries::numImagesInDir(path);
        
        path = image_root_dir_ + "/queries/unanswered/door/uav_" + QString::number(uav_id);
        uav->rejected_images = UnansweredQueries::numImagesInDir(path);
        
        uav_db[uav_id]->status = UavStatus::active;
        uav_db[uav_id]->uav = uav;
        
        ROS_WARN_STREAM("Adding UAV with id: " << uav_id);

        quad_id.setNum(uav_id);
        temp_data = "UAV " + quad_id;

        int index = 0;
        while(index < NUM_UAV && uav_id > active_uavs[index]->id)
            index++;

        active_uavs.insert(active_uavs.begin() + index, uav);
        uavCondWidgetArr.insert(uavCondWidgetArr.begin() + index, new Ui::UAVConditionWidget());
        uavListWidgetArr_.insert(uavListWidgetArr_.begin() + index, new QWidget());

        uavCondWidgetArr[index]->setupUi(uavListWidgetArr_[index]);
        uavCondWidgetArr[index]->VehicleSelectButton->setText(std::to_string(uav_id).c_str());
        uavCondWidgetArr[index]->VehicleNameLine->setText(temp_data);
        
        central_ui_.layout_uavs->insertWidget(index, uavListWidgetArr_[index]);
        
        for(int i = index; i < NUM_UAV; i++)
        {
//            central_ui_.layout_uavs->inseremoveWidget(uavListWidgetArr_[i]);
            disconnect(uavCondWidgetArr[i]->VehicleSelectButton,
                    SIGNAL(clicked()), quad_select_mapper, SLOT(map()));
        }

        for(int i = index; i < active_uavs.size(); i++)
        {
//            central_ui_.layout_uavs->addWidget(uavListWidgetArr_[i]);
            quad_select_mapper->setMapping(uavCondWidgetArr[i]->VehicleSelectButton, i);
            connect(uavCondWidgetArr[i]->VehicleSelectButton, SIGNAL(clicked()),
                    quad_select_mapper, SLOT(map()));
        }

        NUM_UAV++;

        if(NUM_UAV == 1)
            selectUav(0);

        num_uav_changed.wakeAll();
        uav_mutex.unlock();
    }

    void SimpleGCS::deleteUav(int index, UavStatus status)
    {
        uav_mutex.lock();

        SimpleControl * uav = active_uavs[index];
        int uav_id = uav->id;
        
        if(status == UavStatus::purged)
        {
            //TODO loop through all the query types: door, window, hole
            saveUavQueries(uav, "door");
            saveUavAccessPoints(uav, "door");
            delete uav;
            deleted_uavs.remove(uav_id);
        }
        else if(status == UavStatus::deleted)
        {
            deleted_uavs[uav_id] = uav;
        }
        else
        {
            ROS_ERROR_STREAM("invalid deletion status for uav: " << uav_id
                    << " with deletion status: " << status);
            exit(1);
        }
        uav_db[uav_id]->status = status;
        uav_db[uav_id]->uav = nullptr;

        ROS_WARN_STREAM("Deleting UAV with id: " << uav_id);

        central_ui_.layout_uavs->removeWidget(uavListWidgetArr_[index]);

        delete uavCondWidgetArr[index];
        delete uavListWidgetArr_[index];

        uavCondWidgetArr.erase(uavCondWidgetArr.begin() + index);
        uavListWidgetArr_.erase(uavListWidgetArr_.begin() + index);
        active_uavs.erase(active_uavs.begin() + index);

        for(int i = index; i < active_uavs.size(); i++)
        {
            disconnect(uavCondWidgetArr[i]->VehicleSelectButton,
                    SIGNAL(clicked()), quad_select_mapper, SLOT(map()));

            quad_select_mapper->setMapping(uavCondWidgetArr[i]->VehicleSelectButton, i);
            connect(uavCondWidgetArr[i]->VehicleSelectButton,
                    SIGNAL(clicked()), quad_select_mapper, SLOT(map()));
        }

        if(cur_uav == index)
        {
            if(apmQWidget_->isVisible())
                clearAccessPoints();
            if(central_ui_.frame_queries_cntnr->isVisible())
                clearQueries();
        }

        NUM_UAV--;

        if(NUM_UAV == 0) // perform gui cleanup
        {
            cur_uav = -1; // no more UAV's to select
            central_ui_.image_frame->setPixmap(QPixmap::fromImage(QImage()));
        }
        else if(NUM_UAV == 1 || (cur_uav == index && index == 0))
            selectUav(0); // default to only remaining uav/first in the list
        else if(NUM_UAV > 1 && cur_uav == index)
            selectUav(cur_uav-1);
            //if the currently selected uav was deleted, choose the one in front of it

       num_uav_changed.wakeAll();
       uav_mutex.unlock();
    }

    void SimpleGCS::saveUavQueries(SimpleControl * uav, std::string ap_type)
    {
        std::string path = image_root_dir_.toStdString() + "/queries/unanswered/" 
            + ap_type + "/uav_" + std::to_string(uav->id);
        std::vector<lcar_msgs::DoorPtr>* queries = uav->GetDoorQueries();
        for(int i = 0; i < queries->size(); i++)
        {
            lcar_msgs::DoorPtr query = queries->at(i);

            sensor_msgs::Image * ros_image = &query->original_picture;
            cv::Mat image = cv_bridge::toCvCopy(*ros_image,"rgb8")->image;
            
            int num_images = UnansweredQueries::numImagesInDir(QString(path.c_str()));
            
            std::string file = "img_" + std::to_string(num_images) + ".jpg";
            saveImage(path, file, image);

            ros_image = &query->framed_picture;
            image = cv_bridge::toCvCopy(*ros_image, "rgb8")->image;
            file = "img_" + std::to_string(num_images+1) + ".jpg";
            saveImage(path, file, image);
        }
    }

    void SimpleGCS::saveUavAccessPoints(SimpleControl* uav, std::string ap_type)
    {
        std::string path = image_root_dir_.toStdString() + "/access_points/" +ap_type;
        path += "/uav_" + std::to_string(uav->id);
        std::vector<AccessPoint> * ap_vector = uav->GetRefAccessPoints();
        for(int i = 0; i < ap_vector->size(); i++)
        {
            std::string file = "img_" + std::to_string(i) + ".jpg";

            AccessPoint ap = ap_vector->at(i);
            sensor_msgs::Image ros_image = ap.GetImage();
            cv::Mat image = cv_bridge::toCvCopy(ros_image, "rgb8")->image;
            saveImage(path, file, image);
        }
    }

    void SimpleGCS::purgeDeletedUavs()
    {
        std::map<int, UAV*> map = uav_db.toStdMap();
        std::map<int, SimpleControl*> delete_map = deleted_uavs.toStdMap();
        for (auto& uav : map)
        {
            if(uav.second->status == UavStatus::deleted)
            {
                delete deleted_uavs[uav.first];
                deleted_uavs[uav.first] = nullptr;
                uav.second->status = UavStatus::purged;
                ROS_WARN_STREAM("purging UAV with id: " << uav.first);
            }
        }
    }

    void SimpleGCS::uavConnectionToggled(int index, int uav_id, bool toggle)
    {
        if(index >= active_uavs.size() || active_uavs[index]->id != uav_id)
            return;

        QWidget* button = uavCondWidgetArr[index]->VehicleSelectButton;
        button->setEnabled(toggle);

        QString style_sheet = button->isEnabled() ?
            "background-color: rgb(64, 89, 140); color: rgb(240, 240, 240);" :
            "background-color: rgb(80, 90, 110); color: rgb(150, 150, 150);" ;

        button->setStyleSheet(style_sheet);
    }

    //Timed update of GCS gui
    void SimpleGCS::timedUpdate()
    {
        if(NUM_UAV == 0 || cur_uav == -1)
        {
            central_ui_.lbl_cur_uav->setText("NO UAV's");
            return;
        }

        if(timeCounter++ >= 30)
        {
            updateQueries();
            updateAccessPoints();
            timeCounter = 0;
        }

        SimpleControl* uav = active_uavs[cur_uav];
        quad_id.setNum(uav->id);

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

        quad_id.setNum(uav->id);
        temp_data = "UAV " + quad_id;
        central_ui_.lbl_cur_uav->setText(temp_data);

        central_ui_.pgs_bar_mission->setValue(uav->GetMissionProgress() * 100);

        this->updatePFD();

        //Update Uav List widgets
        for(int i = 0; i < NUM_UAV; i++)
        {
            temp_data.setNum(active_uavs[i]->GetBatteryState().percentage * 100);
            uavCondWidgetArr[i]->VehicleBatteryLine->setText(temp_data);
            temp_data = active_uavs[i]->GetState().mode.c_str();
            uavCondWidgetArr[i]->VehicleConditionLine->setText(temp_data);
        }
    }

    void SimpleGCS::clearQueries()
    {
        for(int i = pictureQueryWidgets_.size() - 1; i >= 0; i--)
        {
            central_ui_.layout_queries->removeWidget(pictureQueryWidgets_[i]);
            delete pictureQueryWidgets_[i];
        }
        pictureQueryWidgets_.clear();

        num_queries_last = 0;
    }


    void SimpleGCS::updateQueries()
    {
        if(NUM_UAV == 0 || !central_ui_.frame_queries_cntnr->isEnabled())
            return;

        pictureQueryVector = active_uavs[cur_uav]->GetDoorQueries();
        int pqv_size = pictureQueryVector->size();
        int new_queries = pqv_size - num_queries_last;

        for(int i = 0; i < new_queries; i++)
        {
            //retrieve Query msg for door image
            lcar_msgs::DoorPtr doorQuery = pictureQueryVector->at(i + num_queries_last);

            QImage image(doorQuery->framed_picture.data.data(),
                         doorQuery->framed_picture.width,
                         doorQuery->framed_picture.height,
                         doorQuery->framed_picture.step,
                         QImage::Format_RGB888);

            //create the widget
            QWidget * pmWidget = new QWidget();
            Ui::PictureMsgWidget pmUiWidget;
            pmUiWidget.setupUi(pmWidget);

            QWidget * imgWidget = new QWidget();
            Ui::ImageViewWidget imgUiWidget;
            imgUiWidget.setupUi(imgWidget);

            // take care of the image
            imgUiWidget.image_frame->setImage(image);
            pmUiWidget.PictureLayout->addWidget(imgWidget);

            //add to list
            pictureQueryWidgets_.push_back(pmWidget);
            int index = pictureQueryWidgets_.size() - 1;

            //map signal for yes button
            acceptDoorMapper->setMapping(pmUiWidget.yesButton,
                                         pictureQueryWidgets_[index]);
            connect(pmUiWidget.yesButton, SIGNAL(clicked()),
                    acceptDoorMapper, SLOT(map()));

            //map signal for yes button
            denyDoorMapper->setMapping(pmUiWidget.rejectButton,
                                       pictureQueryWidgets_[index]);
            connect(pmUiWidget.rejectButton, SIGNAL(clicked()),
                    denyDoorMapper, SLOT(map()));

            //add to user interface
            central_ui_.layout_queries->addWidget(pictureQueryWidgets_[index]);
        }

        num_queries_last = pqv_size;
    }

    void SimpleGCS::answerQuery(QWidget * qw, std::string ap_type, bool accepted)
    {
        int index = central_ui_.layout_queries->indexOf(qw);
        lcar_msgs::DoorPtr door = pictureQueryVector->at(index);

        SimpleControl* uav = active_uavs[cur_uav];
        std::string path = image_root_dir_.toStdString() + "/queries";
        std::string file;
        if(accepted)
        {
            path += "/accepted/" + ap_type + "/uav_" + std::to_string(uav->id);
            file = "img_" + std::to_string(uav->accepted_images++) + ".jpg";
        }
        else
        {
            path += "/rejected/" + ap_type + "/uav_" + std::to_string(uav->id);
            file = "img_" + std::to_string(uav->rejected_images++) + ".jpg";
        }

        //dont change my colorspace! (rgb8)
        saveImage(path, file,
            cv_bridge::toCvCopy((sensor_msgs::Image)door->original_picture, "rgb8")->image);

        pictureQueryVector->erase(pictureQueryVector->begin() + index);
        pictureQueryWidgets_.erase(pictureQueryWidgets_.begin() + index);
        central_ui_.layout_queries->removeWidget(qw);
        delete qw;

        door->accepted = accepted;
        num_queries_last--;
        //UAVs[cur_uav]->SendDoorResponse(doormsg);
    }

    void SimpleGCS::acceptDoorQuery(QWidget *qw)
    {
        answerQuery(qw, "door", true);
    }

    void SimpleGCS::rejectDoorQuery(QWidget * qw)
    {
        answerQuery(qw, "door", false);
    }

    void SimpleGCS::saveImage(std::string path, std::string file, const cv::Mat& image)
    {
        if(!boost::filesystem::exists(path))
            boost::filesystem::create_directories(path);

        if(!cv::imwrite(path + "/" + file, image))
            ROS_WARN_STREAM( "failed to write image to: " << path + "/" + file
                    << ". The directory might not exist?");
    }

    void SimpleGCS::executePlay()
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

    void SimpleGCS::cancelPlay()
    {
        if(NUM_UAV == 0)
            return;

        for(int i = 0; i < NUM_UAV; i++)
        {
            active_uavs[i]->SetRTL();
        }
        
        this->toggleScoutButtons(true);
    }

    void SimpleGCS::scoutBuilding()
    {
        if(NUM_UAV == 0)
            return;

        int building_index = central_ui_.cmbo_box_buildings->currentIndex() + 1;
        std::string file_name = "building" + std::to_string(building_index) + ".txt";

        if(this->GetMissionType(file_name).compare("local") == 0) 
            active_uavs[cur_uav]->ScoutBuilding(GetMissionLocal(file_name));
        else 
            active_uavs[cur_uav]->ScoutBuilding(GetMissionGlobal(file_name));

        this->toggleScoutButtons(false);
        
        ROS_INFO_STREAM("Scouting Building " << building_index);
    }

    void SimpleGCS::pauseOrResumeScout()
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
    
    void SimpleGCS::stopScout()
    {
        if(NUM_UAV == 0)
            return;

        active_uavs[cur_uav]->StopMission();
        
        this->toggleScoutButtons(true);
    }

    void SimpleGCS::changeFlightMode(int index)
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
            this->stopScout();
        }
    }

    void SimpleGCS::toggleScoutButtons(bool visible)
    {
        central_ui_.btn_scout->setVisible(visible);
        central_ui_.btn_scout_play_pause->setVisible(!visible);
        central_ui_.btn_scout_stop->setVisible(!visible);
    }
    
    void SimpleGCS::clearAccessPoints()
    {
        //clear out old list of Access points widgets
        int size = accessPointWidgets_.size();
        for(int i = size-1; i >= 0; i--)
        {
            apmUi_.AccessPointMenuLayout->removeWidget(accessPointWidgets_[i]);
            delete accessPointWidgets_[i];
        }
        accessPointWidgets_.clear();

        num_access_points_last = 0;
    }

    void SimpleGCS::updateAccessPoints()
    {
        if(NUM_UAV == 0  || !apmQWidget_->isVisible())
            return;

        //retreive access points
        accessPointVector = active_uavs[cur_uav]->GetRefAccessPoints();
        
        //Get our new number of Access points
        int apv_size = accessPointVector->size();
        int new_access_points = apv_size - num_access_points_last;

        for(int i = 0; i < new_access_points; i++)
        {
            //retrieve access point
            AccessPoint accessPoint = accessPointVector->at(i + num_access_points_last);
            sensor_msgs::Image acImage = accessPoint.GetImage();

            QImage image(acImage.data.data(), acImage.width, acImage.height,
                         acImage.step, QImage::Format_RGB888);

            QWidget * apWidget = new QWidget();
            Ui::AccessPointStatsWidget apUiWidget;
            apUiWidget.setupUi(apWidget);

            QWidget * imageWidget = new QWidget();
            Ui::ImageViewWidget imageUiWidget;
            imageUiWidget.setupUi(imageWidget);

             //image
            imageUiWidget.image_frame->setImage(image);
            apUiWidget.PictureLayout->addWidget(imageWidget);

            //add widget to the list
            accessPointWidgets_.push_back(apWidget);
            int index = accessPointWidgets_.size() - 1;

            //access point name
            access_point_id.setNum(i + num_access_points_last + 1);
            access_point_temp_data = "Access Point " + access_point_id;
            apUiWidget.buildingNameLine->setText(access_point_temp_data);

            //altitude
            access_point_temp_data.setNum(accessPoint.GetAltitude().data, 'f', 2);
            apUiWidget.altitudeLineEdit->setText(access_point_temp_data);

            //heading
            access_point_temp_data.setNum(accessPoint.GetHeading().data, 'f', 2);
            apUiWidget.compassHeadingLineEdit->setText(access_point_temp_data);

            //location longitude
            access_point_temp_data.setNum(accessPoint.GetLocation().longitude, 'f', 2);
            apUiWidget.longitudeLineEdit->setText(access_point_temp_data);

            //location latitude
            access_point_temp_data.setNum(accessPoint.GetLocation().latitude, 'f', 2);
            apUiWidget.latitudeLineEdit->setText(access_point_temp_data);

            //time
            access_point_temp_data.setNum(accessPoint.GetTime().toSec(), 'f', 6);
            apUiWidget.captureTimeLineEdit->setText(access_point_temp_data);

            //map signal to the delete button
            access_point_mapper->setMapping(apUiWidget.deleteAccessPointButton,
                                       accessPointWidgets_[index]);

            connect(apUiWidget.deleteAccessPointButton, SIGNAL(clicked()),
                    access_point_mapper, SLOT(map()));

            //finally, add it to the gui
            apmUi_.AccessPointMenuLayout->addWidget(accessPointWidgets_[index]);
        }

        num_access_points_last = apv_size;
    }

    void SimpleGCS::showAccessPoints()
    {
        quad_id.setNum(active_uavs[cur_uav]->id);
        temp_data = "UAV " + quad_id;
        apmUi_.uavNameLineEdit->setText(temp_data);

        apmQWidget_->show();
    }

    void SimpleGCS::deleteAccessPoint(QWidget* w)
    {
        int deleteIndex = apmUi_.AccessPointMenuLayout->indexOf(w);
        // not sure why deleteIndex - 1 is necessary
        accessPointVector->erase(accessPointVector->begin() + deleteIndex - 1);
        accessPointWidgets_.erase(accessPointWidgets_.begin() + deleteIndex -1);
        apmUi_.AccessPointMenuLayout->removeWidget(w);
        delete w;

        num_access_points_last--;
    }

    // slot gets called when user click on a uav button
    void SimpleGCS::uavSelected(int quadNumber)
    {
        selectUav(quadNumber);
    }

    //needed in addUav(int) and deleteUav(int)
    void SimpleGCS::selectUav(int uav_number)
    {
        if(cur_uav == uav_number)
            return;

        cur_uav = uav_number;
        int uav_id = active_uavs[cur_uav]->id;
        sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav_id) + "/stereo_cam/left/image_rect",
                                         5, &SimpleGCS::ImageCallback, this);

        central_ui_.image_frame->setPixmap(QPixmap::fromImage(QImage()));

        clearAccessPoints();
        clearQueries();
        
        MissionMode m = active_uavs[cur_uav]->GetMissionMode();
        if(m == MissionMode::stopped)
            this->toggleScoutButtons(true); 
        else
        {
            this->toggleScoutButtons(false);
            if(m == MissionMode::active)
                central_ui_.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/pause.png"));
            else
                central_ui_.btn_scout_play_pause->setIcon(QIcon(":/icons/icons/play.png"));
        }
        
        this->toggleArmDisarmButton(active_uavs[cur_uav]->GetState().armed);
    }


    void SimpleGCS::armOrDisarmSelectedUav()
    {
        if(NUM_UAV == 0)
            return;
        
        bool armed = active_uavs[cur_uav]->GetState().armed;
       
        toggleArmDisarmButton(armed);
        
        active_uavs[cur_uav]->Arm(!armed);        
    }
    
    void SimpleGCS::toggleArmDisarmButton(bool arm)
    {
    if(arm)
            central_ui_.btn_arm_uav->setText("Disarm");
        else
            central_ui_.btn_arm_uav->setText("Arm");
    }

    void SimpleGCS::shutdownPlugin()
    {
        // TODO unregister all publishers here
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

    void SimpleGCS::updatePFD()
    {
        if(NUM_UAV == 0)
            return;
        
        central_ui_.graphicsPFD->setRoll((active_uavs[cur_uav]->GetFlightState().roll)*180);
        central_ui_.graphicsPFD->setPitch((active_uavs[cur_uav]->GetFlightState().pitch)*90);
        central_ui_.graphicsPFD->setHeading(active_uavs[cur_uav]->GetFlightState().heading);
        central_ui_.graphicsPFD->setAirspeed(active_uavs[cur_uav]->GetFlightState().ground_speed);
        central_ui_.graphicsPFD->setAltitude(active_uavs[cur_uav]->GetFlightState().altitude);
        central_ui_.graphicsPFD->setClimbRate(active_uavs[cur_uav]->GetFlightState().vertical_speed);

        central_ui_.graphicsPFD->update();
    }

    std::string SimpleGCS::GetMissionType(std::string file_name)
    {   
        ROS_WARN_STREAM("in get mission");
        std::string mission_type;
        std::ifstream fileIn(file_name);

        fileIn >> mission_type;

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


    void SimpleGCS::clearImageView()
    {
        //TODO
    }

    void SimpleGCS::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            //don't change my colorspace! (bgr8)
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            const cv::Mat mat= cv_ptr->image;
            QImage image(mat.data, mat.cols, mat.rows, mat.step[0],
                         QImage::Format_RGB888);
            
            int w = central_ui_.image_frame->width();
            int h = central_ui_.image_frame->height();
            central_ui_.image_frame->setPixmap(QPixmap::fromImage(image)
                                                .scaled(w,h,Qt::KeepAspectRatio)
                                               );
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Error in image subsrcriber: %s", e.what());
        }
    }

    void SimpleGCS::initHelperThread()
    {
        uav_monitor = new SimpleGCSHelper(this);
        uav_monitor->moveToThread(&t_uav_monitor);

        connect(&t_uav_monitor, &QThread::started,
                uav_monitor, &SimpleGCSHelper::help);

        connect(uav_monitor, &SimpleGCSHelper::addUav,
                this, &SimpleGCS::addUav);

        connect(uav_monitor, &SimpleGCSHelper::deleteUav,
                this, &SimpleGCS::deleteUav);

        connect(uav_monitor, &SimpleGCSHelper::toggleUavConnection,
                this, &SimpleGCS::uavConnectionToggled);

        t_uav_monitor.start();
    }
    
    void SimpleGCS::initMap()
    {
        std::string s = ros::package::getPath("rqt_gcs");         
        QString map_url = "file://" % QString(s.c_str()) % "/map/uavmap.html";
        central_ui_.web_view->load(QUrl(map_url));
    }
    
    void SimpleGCS::initMenuBar()
    {
       menu_bar_ = new QMenuBar(widget_main_);

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
               this, &SimpleGCS::unansweredQueriesTriggered);
       
       
       //tools menu
       tools_menu = menu_bar_->addMenu("Tools");
       settings_act = tools_menu->addAction("Settings");
       connect(settings_act, &QAction::triggered, 
               this, &SimpleGCS::settingsTriggered); 
       
       
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
    void SimpleGCS::initSettings()
    {
        //initialize settings and uav queries display
        settings_ = new QSettings("SERL", "LCAR_Bot");

        settings_->beginGroup("general_tab");

        if(settings_->value("machine_learning", "online").toString() == "online")
            toggleMachineLearningMode(true);
        else
            toggleMachineLearningMode(false);

        QString path = getenv("HOME");
        path += "/Pictures/LCAR_Bot";
        image_root_dir_ = settings_->value("images_root_directory", path).toString();
        ROS_INFO_STREAM("images root directory: " << image_root_dir_.toStdString());

        settings_->endGroup();
        
        settings_->beginGroup("object_detection_tab");
         // todo: read object detectin settings and publish them
        
        settings_->endGroup();
    }

    void SimpleGCS::settingsTriggered()
    {
        if(widgets_.settings_ == nullptr)
        {
            widgets_.settings_ = new SettingsWidget(this);
            
            QRect window = widget_main_->window()->geometry();
            int x = (widget_main_->width() / 2) - (widgets_.settings_->width() / 2);
            int y = (widget_main_->height() / 2) - (widgets_.settings_->height() / 2);
            widgets_.settings_->move(window.x() + x, window.y() + y);
            widgets_.settings_->setVisible(true);

            connect(widgets_.settings_, SIGNAL(machineLearningModeToggled(bool)),
                    this, SLOT(toggleMachineLearningMode(bool)));
        }
        else
        {
            widgets_.settings_->showNormal();
            widgets_.settings_->activateWindow();
        }
    }
    
    void SimpleGCS::unansweredQueriesTriggered()
    {
        if(widgets_.unanswered_queries_ == nullptr)
        {
            widgets_.unanswered_queries_ = new UnansweredQueries(this);
            
            QRect window = widget_main_->window()->geometry();
            int x = (widget_main_->width() / 2) - (widgets_.unanswered_queries_->width() / 2);
            int y = (widget_main_->height() / 2) - (widgets_.unanswered_queries_->height() / 2);
            widgets_.unanswered_queries_->move(window.x() + x, window.y() + y);
            widgets_.unanswered_queries_->setVisible(true);
        }
        else
        {
            widgets_.unanswered_queries_->showNormal();
            widgets_.unanswered_queries_->activateWindow();
        }
    }

    void SimpleGCS::destroySettingsWidget()
    {
//        delete settings_widget_;
//        settings_widget_ = nullptr;
    }
        

    void SimpleGCS::toggleMachineLearningMode(bool toggle)
    {
        central_ui_.frame_queries_cntnr->setVisible(toggle);
        central_ui_.frame_queries_cntnr->setEnabled(toggle);

        for(int i = 0; i < NUM_UAV; i++)
            active_uavs[i]->SetOnlineMode(toggle);
    }

    void SimpleGCS::publishHitThreshold(double thresh)
    {
        std_msgs::Float64 msg;
        msg.data = thresh;
        od_pubs.pub_hit_thresh.publish(msg);
    }
    
    void SimpleGCS::publishStepSize(int step)
    {
        std_msgs::Int32 msg;
        msg.data = step;
        od_pubs.pub_step_size.publish(msg);
    }
    
    void SimpleGCS::publishPadding(int padding)
    {
        std_msgs::Int32 msg;
        msg.data = padding;
        od_pubs.pub_padding.publish(msg);
    }
    
    void SimpleGCS::publishScaleFactor(double scale)
    {
        std_msgs::Float64 msg;
        msg.data = scale;
        od_pubs.pub_scale_factor.publish(msg);
    }
    
    void SimpleGCS::publishMeanShift(bool on)
    {
        std_msgs::Int32 msg;
        msg.data = on;
        od_pubs.pub_mean_shift.publish(msg); 
    }
    
    //////////////////////////  SimpleGCSHelper  ///////////////////////////////

    SimpleGCSHelper::SimpleGCSHelper(SimpleGCS * sgcs) :
        gcs(sgcs)
    { }

    SimpleGCSHelper::~SimpleGCSHelper()
    {
        gcs = nullptr;
    }

    int SimpleGCSHelper::binarySearch(int target_id, int low, int high)
    {
        if(low > high)
            return -1;

        int index = (low + high) / 2;

        if(gcs->active_uavs[index]->id == target_id)
            return index;
        else if(gcs->active_uavs[index]->id < target_id) //search higher
            return binarySearch(target_id, index+1, high);
        else
            return binarySearch(target_id, low, index-1);
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

        if(gcs->NUM_UAV < uav_map.size())
        {
            for(auto const& uav : uav_map)
            {
                std::cout << "id: " << uav.first << " count: " << gcs->uav_db.count(uav.first) << "\n";
                if(gcs->uav_db.count(uav.first) == 0)
                    gcs->uav_db.insert(uav.first, new SimpleGCS::UAV(nullptr, UavStatus::null));
                
                UavStatus status = gcs->uav_db[uav.first]->status;
                std::cout << "Uav Status for uav_id: " << status << "\n";
                if(gcs->uav_db[uav.first]->status != UavStatus::active)
                {
                    emit addUav(uav.first);
                    gcs->num_uav_changed.wait(&gcs->uav_mutex);
                }
            }
        }
        else if(gcs->NUM_UAV > uav_map.size())
        {
            for(int i = 0; i < gcs->active_uavs.size(); i++)
            {
                if(uav_map.count(gcs->active_uavs[i]->id) == 0)
                {
                    emit deleteUav(i, UavStatus::purged);
                    i--;
                    gcs->num_uav_changed.wait(&gcs->uav_mutex);
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
            SimpleControl* uav = gcs->active_uavs[i];
            QWidget* button = gcs->uavCondWidgetArr[i]->VehicleSelectButton;

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
