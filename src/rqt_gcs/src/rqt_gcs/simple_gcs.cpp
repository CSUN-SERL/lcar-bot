#include <rqt_gcs/simple_gcs.h>


namespace rqt_gcs
{

    SimpleGCS::SimpleGCS()
    : rqt_gui_cpp::Plugin()
    , widget_(nullptr)
    , settings_widget_(nullptr)
    {
        //Constructor is called first before initPlugin function
        setObjectName("LCAR Bot GCS");
        qRegisterMetaType<UavStatus>("UavStatus");
        qRegisterMetaType<UavStatus>("Mat");
    }

    void SimpleGCS::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        QStringList argv = context.argv();

        widget_                = new QWidget();
        missionProgressWidget_ = new QWidget();
        uavQuestionWidget_     = new QWidget();
        uavStatWidget_         = new QWidget();
        imageViewWidget_       = new QWidget();
        PFDQWidget             = new QWidget();
        apmQWidget_            = new QWidget();

        NUM_UAV = 0;
        timeCounter = 0;
        cur_uav = -1;
        num_queries_last = 0;
        num_access_points_last = 0;

        //Setup the UI objects with the widgets
        central_ui_.setupUi(widget_);
        mpUi_.setupUi(missionProgressWidget_);
        uqUi_.setupUi(uavQuestionWidget_);
        usUi_.setupUi(uavStatWidget_);
        ivUi_.setupUi(imageViewWidget_);
        pfd_ui.setupUi(PFDQWidget);
        apmUi_.setupUi(apmQWidget_);

        //Add widgets to the Main UI
        context.addWidget(widget_);
        central_ui_.MissionLayout->addWidget(missionProgressWidget_);
        central_ui_.OverviewLayout->addWidget(uavStatWidget_);
        central_ui_.PFDLayout->addWidget(PFDQWidget);
        central_ui_.CameraLayout->addWidget(imageViewWidget_);

        //Setup mission progress widgets
        uavStatWidget_->setWindowTitle("Flight State");
        missionProgressWidget_->setWindowTitle("Mission Control");

        //setup button logic for the widgets
        connect(mpUi_.executePlayButton, SIGNAL(clicked()), this, SLOT(ExecutePlay()));
        connect(mpUi_.cancelPlayButton, SIGNAL(clicked()), this, SLOT(CancelPlay()));
        connect(mpUi_.scoutBuildingButton, SIGNAL(clicked()), this, SLOT(ScoutBuilding()));
        connect(mpUi_.stopMissionButton, SIGNAL(clicked()), this, SLOT(StopQuad()));
        connect(mpUi_.changeFlightModeButton, SIGNAL(clicked()), this, SLOT(ChangeFlightMode()));
        connect(mpUi_.viewAccessPointsButton, SIGNAL(clicked()), this, SLOT(ShowAccessPoints()));
        connect(mpUi_.armButton, SIGNAL(clicked()), this, SLOT(ArmSelectedQuad()));
        connect(mpUi_.disarmButton, SIGNAL(clicked()), this, SLOT(DisarmSelectedQuad()));

        //Setup UAV lists select functions
        signal_mapper = new QSignalMapper(this);
        signal_mapper2 = new QSignalMapper(this);
        acceptDoorMapper = new QSignalMapper(this);
        denyDoorMapper = new QSignalMapper(this);

        connect(signal_mapper, SIGNAL(mapped(int)), this, SLOT(QuadSelected(int)));
        connect(signal_mapper2, SIGNAL(mapped(QWidget*)), this, SLOT(DeleteAccessPoint(QWidget*)));
        connect(acceptDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(AcceptDoorQuery(QWidget*)));
        connect(denyDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(RejectDoorQuery(QWidget*)));

        initializeSettings();
        initializeHelperThread();
        unanswered_queries = new UnansweredQueries(this);
        unanswered_queries->setVisible(true);

        //Setup update timer
        update_timer = new QTimer(this);
        connect(update_timer, SIGNAL(timeout()), this, SLOT(TimedUpdate()));
        //30 hz :1000/30 = 33.33...
        update_timer->start(33);
    }

    void SimpleGCS::AddUav(int uav_id)
    {
        uav_mutex.lock();

        SimpleControl * uav;
        if(all_uav_stat[uav_id] == UavStatus::null || all_uav_stat[uav_id] == UavStatus::purged)
            uav = new SimpleControl(uav_id);
        else if(all_uav_stat[uav_id] == UavStatus::deleted && deleted_uavs[uav_id] != nullptr)
            uav = deleted_uavs[uav_id];
        else
        {
            ROS_ERROR_STREAM("Error adding UAV with id:" << uav_id
                    << "\n all_uavs[uav_id]=" << all_uav_stat[uav_id]
                    << "\n deleted_uavs[uav_id]=" << deleted_uavs[uav_id]);
            exit(1);
        }

        all_uav_stat[uav_id] = UavStatus::active;

        ROS_WARN_STREAM("Adding UAV with id: " << uav_id);

        quad_id.setNum(uav_id);
        temp_data = "UAV " + quad_id;

        int index = 0;
        while(index < NUM_UAV && uav_id > active_uavs[index]->id)
            index++;

        active_uavs.insert(active_uavs.begin() + index, uav);
        uavCondWidgetArr.insert(uavCondWidgetArr.begin() + index, new Ui::UAVConditionWidget());
        uavListWidgetArr.insert(uavListWidgetArr.begin() + index, new QWidget());

        uavCondWidgetArr[index]->setupUi(uavListWidgetArr[index]);
        uavCondWidgetArr[index]->VehicleSelectButton->setText(std::to_string(uav_id).c_str());
        uavCondWidgetArr[index]->VehicleNameLine->setText(temp_data);

        for(int i = index; i < NUM_UAV; i++)
        {
            central_ui_.UAVListLayout->removeWidget(uavListWidgetArr[i]);
            disconnect(uavCondWidgetArr[i]->VehicleSelectButton,
                    SIGNAL(clicked()), signal_mapper, SLOT(map()));
        }

        for(int i = index; i < active_uavs.size(); i++)
        {
            central_ui_.UAVListLayout->addWidget(uavListWidgetArr[i]);
            signal_mapper->setMapping(uavCondWidgetArr[i]->VehicleSelectButton, i);
            connect(uavCondWidgetArr[i]->VehicleSelectButton, SIGNAL(clicked()),
                    signal_mapper, SLOT(map()));
        }

        NUM_UAV++;

        if(NUM_UAV == 1)
            selectQuad(0);

        num_uav_changed.wakeAll();
        uav_mutex.unlock();
    }

    void SimpleGCS::DeleteUav(int index, UavStatus status)
    {
        uav_mutex.lock();

        SimpleControl * uav = active_uavs[index];
        int uav_id = uav->id;

        if(status == UavStatus::purged)
        {
            saveUavQueries(uav);
            saveUavAccessPoints(uav);
            delete uav;
            deleted_uavs.erase(uav_id);
        }
        else if(status == UavStatus::deleted)
        {
            if(deleted_uavs.count(uav_id) == 0)
                deleted_uavs.insert(std::pair<int, SimpleControl*>(uav_id, uav));
            else
                deleted_uavs[uav_id] = uav;
        }
        else
        {
            ROS_ERROR_STREAM("invalid deletion status for uav: " << uav_id
                    << " with deletion status: " << status);
            exit(1);
        }
        all_uav_stat[uav_id] = status;

        ROS_WARN_STREAM("Deleting UAV with id: " << uav_id);

        central_ui_.UAVListLayout->removeWidget(uavListWidgetArr[index]);

        delete uavCondWidgetArr[index];
        delete uavListWidgetArr[index];

        uavCondWidgetArr.erase(uavCondWidgetArr.begin() + index);
        uavListWidgetArr.erase(uavListWidgetArr.begin() + index);
        active_uavs.erase(active_uavs.begin() + index);

        for(int i = index; i < active_uavs.size(); i++)
        {
            disconnect(uavCondWidgetArr[i]->VehicleSelectButton,
                    SIGNAL(clicked()), signal_mapper, SLOT(map()));

            signal_mapper->setMapping(uavCondWidgetArr[i]->VehicleSelectButton, i);
            connect(uavCondWidgetArr[i]->VehicleSelectButton,
                    SIGNAL(clicked()), signal_mapper, SLOT(map()));
        }

        if(cur_uav == index)
        {
            if(apmQWidget_->isVisible())
                clearAccessPoints();
            if(central_ui_.uavQueriesFame->isVisible())
                clearQueries();
        }

        NUM_UAV--;

        if(NUM_UAV == 0) // perform gui cleanup
        {
            cur_uav = -1; // no more UAV's to select
            ivUi_.image_frame->setImage(QImage()); // clear the image view
        }
        else if(NUM_UAV == 1 || (cur_uav == index && index == 0))
            selectQuad(0); // default to only remaining uav/first in the list
        else if(NUM_UAV > 1 && cur_uav == index)
            selectQuad(cur_uav-1);
            //if the currently selected uav was deleted, choose the one in front of it

       num_uav_changed.wakeAll();
       uav_mutex.unlock();
    }

    void SimpleGCS::saveUavQueries(SimpleControl * uav)
    {
        std::string path = image_root_path_.toStdString() + "/queries/unanswered/door/";
        path += "uav_" + std::to_string(uav->id);
        std::vector<lcar_msgs::DoorPtr>* queries = uav->GetDoorQueries();
        for(int i = 0, j = 0; i < queries->size(); i++, j+=2)
        {
            lcar_msgs::DoorPtr query = queries->at(i);

            sensor_msgs::Image * ros_image = &query->original_picture;
            cv::Mat image = cv_bridge::toCvCopy(*ros_image,"rgb8")->image;
            std::string file = "img_" + std::to_string(j) + ".jpg";
            saveImage(path, file, image);

            ros_image = &query->framed_picture;
            image = cv_bridge::toCvCopy(*ros_image, "rgb8")->image;
            file = "img_" + std::to_string(j+1) + ".jpg";
            saveImage(path, file, image);
        }
    }

    void SimpleGCS::saveUavAccessPoints(SimpleControl* uav)
    {
        std::string path = image_root_path_.toStdString() + "/access_points/door/";
        path += "uav_" + std::to_string(uav->id);
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

    void SimpleGCS::PurgeDeletedUavs()
    {
        for (auto& uav : all_uav_stat)
        {
            if(uav.second == UavStatus::deleted)
            {
                delete deleted_uavs[uav.first];
                deleted_uavs[uav.first] = nullptr;
                uav.second = UavStatus::purged;
                ROS_WARN_STREAM("purging UAV with id: " << uav.first);
            }
        }
    }

    void SimpleGCS::UavConnectionToggled(int index, int uav_id, bool toggle)
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
    void SimpleGCS::TimedUpdate()
    {
        if(NUM_UAV == 0 || cur_uav == -1)
        {
            mpUi_.uavNameEdit->setText("NO UAV's");
            return;
        }

        if(timeCounter++ >= 30)
        {
            //ROS_INFO_STREAM("MSGs UPdated");
            UpdateQueries();
            UpdateAccessPoints();
            timeCounter = 0;
        }

        SimpleControl* uav = active_uavs[cur_uav];
        quad_id.setNum(uav->id);

        temp_data = uav->GetState().mode.c_str();
        temp_data += ":";
        temp_data += uav->GetState().armed ? "Armed" : "Disarmed";
        usUi_.flightModeDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().yaw, 'f', 2);
        usUi_.yawDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().roll, 'f', 2);
        usUi_.rollDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().pitch, 'f', 2);
        usUi_.pitchDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().altitude, 'f', 2);
        usUi_.altitudeDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().vertical_speed, 'f', 2);
        usUi_.verticalSpaceDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().ground_speed, 'f', 2);
        usUi_.horizontalSpaceDisplay->setText(temp_data);

        temp_data.setNum(uav->GetFlightState().heading, 'f', 2);
        usUi_.headingDisplay->setText(temp_data);

        temp_data.setNum(uav->GetDistanceToWP());
        usUi_.waypointDisplay->setText(temp_data);

        temp_data.setNum(uav->GetBatteryStatus().remaining * 100);
        usUi_.batteryProgressBar->setValue(temp_data.toInt());

        quad_id.setNum(uav->id);
        temp_data = "UAV " + quad_id;

        mpUi_.uavNameEdit->setText(temp_data);

        mpUi_.missionProgressBar->setValue(uav->GetMissionProgress()*100);

        this->UpdatePFD();

        //Update Uav List widgets
        for(int i = 0; i < NUM_UAV; i++)
        {
            temp_data.setNum(active_uavs[i]->GetBatteryStatus().remaining * 100);
            uavCondWidgetArr[i]->VehicleBatteryLine->setText(temp_data);
            temp_data = active_uavs[i]->GetState().mode.c_str();
            uavCondWidgetArr[i]->VehicleConditionLine->setText(temp_data);
        }
    }

    void SimpleGCS::clearQueries()
    {
        for(int i = pictureQueryWidgets_.size() - 1; i >= 0; i--)
        {
            central_ui_.PictureMsgLayout->removeWidget(pictureQueryWidgets_[i]);
            delete pictureQueryWidgets_[i];
        }
        pictureQueryWidgets_.clear();

        num_queries_last = 0;
    }


    void SimpleGCS::UpdateQueries()
    {
        if(NUM_UAV == 0 || !central_ui_.uavQueriesFame->isEnabled())
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
            central_ui_.PictureMsgLayout->addWidget(pictureQueryWidgets_[index]);
        }

        num_queries_last = pqv_size;
    }

    void SimpleGCS::answerQuery(QWidget * qw, std::string ap_type, bool accepted)
    {
        int index = central_ui_.PictureMsgLayout->indexOf(qw);
        lcar_msgs::DoorPtr door = (*pictureQueryVector)[index];

        SimpleControl* uav = active_uavs[cur_uav];
        std::string path = image_root_path_.toStdString() + "/queries";
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
        central_ui_.PictureMsgLayout->removeWidget(qw);
        delete qw;

        door->accepted = accepted;
        num_queries_last--;
        //UAVs[cur_uav]->SendDoorResponse(doormsg);
    }

    void SimpleGCS::AcceptDoorQuery(QWidget *qw)
    {
        answerQuery(qw, "door", true);
    }

    void SimpleGCS::RejectDoorQuery(QWidget * qw)
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

    void SimpleGCS::ExecutePlay()
    {
        if(NUM_UAV == 0)
            return;

        int play_num = mpUi_.playComboBox->currentIndex();

        if(play_num == 0)
        {
            for(int i = 0; i < NUM_UAV; i++)
            {
                std::string file_name = "building" + std::to_string(i + 1) + ".txt";

                active_uavs[i]->Arm(true);
                active_uavs[i]->ScoutBuilding(GetMission(file_name));
                ROS_INFO_STREAM("Scouting Building " << i);
            }
        }
        else if(play_num == 1)
        {

        }
        else if(play_num == 2)
        {

        }

        ROS_INFO_STREAM("Play " << play_num << " initiated");
    }

    void SimpleGCS::CancelPlay()
    {
        if(NUM_UAV == 0)
            return;

        for(int i = 0; i < NUM_UAV; i++)
        {
            active_uavs[i]->SetRTL();
        }
    }

    void SimpleGCS::ScoutBuilding()
    {
        if(NUM_UAV == 0)
            return;

        int building_index = mpUi_.buildingsComboBox->currentIndex() + 1;
        std::string file_name = "building" + std::to_string(building_index) + ".txt";

        active_uavs[cur_uav]->ScoutBuilding(GetMission(file_name));
        ROS_INFO_STREAM("Scouting Building " << building_index);
    }

    void SimpleGCS::StopQuad()
    {
        if(NUM_UAV == 0)
            return;

        active_uavs[cur_uav]->SetRTL();
    }

    void SimpleGCS::ChangeFlightMode()
    {
        if(NUM_UAV == 0)
            return;

        if(mpUi_.flightModeComboBox->currentIndex() == 0)
        {
            ROS_INFO_STREAM("Quadrotor Stablized");
            active_uavs[cur_uav]->SetMode("STABILIZED");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 1)
        {
            ROS_INFO_STREAM("Quadrotor Loiter");
            active_uavs[cur_uav]->SetMode("AUTO.LOITER");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 2)
        {
            ROS_INFO_STREAM("Quadrotor Land");
            active_uavs[cur_uav]->SetMode("AUTO.LAND");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 3)
        {
            ROS_INFO_STREAM("Altitude Hold");
            active_uavs[cur_uav]->SetMode("ALTCTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 4)
        {
            ROS_INFO_STREAM("Position Hold");
            active_uavs[cur_uav]->SetMode("POSCTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 5)
        {
            ROS_INFO_STREAM("Quadrotor Return-To-Launch");
            active_uavs[cur_uav]->SetRTL();
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 6)
        {
            ROS_INFO_STREAM("Quadrotor Auto");
            active_uavs[cur_uav]->SetMode("AUTO");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 7)
        {
            ROS_INFO_STREAM("Quadrotor Offboard");
            active_uavs[cur_uav]->SetMode("OFFBOARD");
        }
    }

    void SimpleGCS::clearAccessPoints()
    {
        //clear out old list of Access points widgets
        for(int i = accessPointWidgets_.size()-1; i >= 0; i--)
        {
            apmUi_.AccessPointMenuLayout->removeWidget(accessPointWidgets_[i]);
            delete accessPointWidgets_[i];
        }
        accessPointWidgets_.clear();

        num_access_points_last = 0;
    }

    void SimpleGCS::UpdateAccessPoints()
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
            signal_mapper2->setMapping(apUiWidget.deleteAccessPointButton,
                                       accessPointWidgets_[index]);

            connect(apUiWidget.deleteAccessPointButton, SIGNAL(clicked()),
                    signal_mapper2, SLOT(map()));

            //finally, add it to the gui
            apmUi_.AccessPointMenuLayout->addWidget(accessPointWidgets_[index]);
        }

        num_access_points_last = apv_size;
    }

    void SimpleGCS::ShowAccessPoints()
    {
        quad_id.setNum(active_uavs[cur_uav]->id);
        temp_data = "UAV " + quad_id;
        apmUi_.uavNameLineEdit->setText(temp_data);

        apmQWidget_->show();
    }

    void SimpleGCS::DeleteAccessPoint(QWidget* w)
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
    void SimpleGCS::QuadSelected(int quadNumber)
    {
        selectQuad(quadNumber);
    }

    //needed in addUav(int) and deleteUav(int)
    void SimpleGCS::selectQuad(int quadNumber)
    {
        if(cur_uav == quadNumber)
            return;

        cur_uav = quadNumber;
        int uav_id = active_uavs[cur_uav]->id;
        sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav_id) + "/stereo_cam/left/image_rect",
                                         30, &SimpleGCS::ImageCallback, this);
        ivUi_.image_frame->setImage(QImage()); // clear the image view

        clearAccessPoints();
        clearQueries();
    }

    void SimpleGCS::ArmSelectedQuad()
    {
        if(NUM_UAV == 0)
            return;

        active_uavs[cur_uav]->Arm(true);
    }

    void SimpleGCS::DisarmSelectedQuad()
    {
        if(NUM_UAV == 0)
            return;

        active_uavs[cur_uav]->Arm(false);
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

    void SimpleGCS::UpdatePFD()
    {
        if(NUM_UAV == 0)
            return;

        pfd_ui.widgetPFD->setRoll((active_uavs[cur_uav]->GetFlightState().roll)*180);
        pfd_ui.widgetPFD->setPitch((active_uavs[cur_uav]->GetFlightState().pitch)*90);
        pfd_ui.widgetPFD->setHeading(active_uavs[cur_uav]->GetFlightState().heading);
        pfd_ui.widgetPFD->setAirspeed(active_uavs[cur_uav]->GetFlightState().ground_speed);
        pfd_ui.widgetPFD->setAltitude(active_uavs[cur_uav]->GetFlightState().altitude);
        pfd_ui.widgetPFD->setClimbRate(active_uavs[cur_uav]->GetFlightState().vertical_speed);

        pfd_ui.widgetPFD->update();
    }

    lcar_msgs::Target SimpleGCS::GetMission(std::string fileName)
    {
        float pos_x, pos_y, pos_z, radius;
        lcar_msgs::Target building;
        std::ifstream fileIn(fileName);
        fileIn >> pos_x;
        fileIn >> pos_y;
        fileIn >> pos_z;
        fileIn >> radius;
        //fileIn >> yaw;

        building.target_local.position.x = pos_x;
        building.target_local.position.y = pos_y;
        building.target_local.position.z = pos_z;
        building.radius = radius;
        //building.orientation.w = yaw;

        return building;
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
            conversion_mat_ = cv_ptr->image;
            QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows,
                         conversion_mat_.step[0], QImage::Format_RGB888);
            ivUi_.image_frame->setImage(image);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Error in image subsrcriber: %s", e.what());
        }
    }

    void SimpleGCS::initializeHelperThread()
    {
        uav_monitor = new SimpleGCSHelper(this);
        uav_monitor->moveToThread(&t_uav_monitor);

        connect(&t_uav_monitor, &QThread::started,
                uav_monitor, &SimpleGCSHelper::monitor);

        connect(uav_monitor, &SimpleGCSHelper::addUav,
                this, &SimpleGCS::AddUav);

        connect(uav_monitor, &SimpleGCSHelper::deleteUav,
                this, &SimpleGCS::DeleteUav);

        connect(uav_monitor, &SimpleGCSHelper::toggleUavConnection,
                this, &SimpleGCS::UavConnectionToggled);

        t_uav_monitor.start();
    }

    // SettingsWidget and QSettings related stuff
    void SimpleGCS::initializeSettings()
    {
        //initialize settings and uav queries display
        settings_ = new QSettings("SERL", "LCAR_Bot");

        settings_->beginGroup("general_tab");

        if(settings_->value("machine_learning","online").toString() == "online")
            ToggleMachineLearningMode(true);
        else
            ToggleMachineLearningMode(false);

        QString path = getenv("HOME");
        path += "/Pictures/LCAR_Bot";
        image_root_path_ = settings_->value("machine_learning/save_path", path).toString();
        ROS_INFO_STREAM("images root directory: " << image_root_path_.toStdString());

        settings_->endGroup();

        //connect settings button to settings widget
        connect(central_ui_.settingsButton, SIGNAL(clicked()),
                this, SLOT(SettingsClicked()));
    }


    void SimpleGCS::SettingsClicked()
    {
        if(settings_widget_ == nullptr)
        {
            settings_widget_ = new SettingsWidget(settings_);

            QRect window = widget_->window()->geometry();
            int x = (widget_->width() / 2) - (settings_widget_->width() / 2);
            int y = (widget_->height() / 2) - (settings_widget_->height() / 2);
            settings_widget_->move(window.x() + x, window.y() + y);
            settings_widget_->setVisible(true);

            connect(settings_widget_, SIGNAL(dismissMe()),
                    this, SLOT(DestroySettingsWidget()));

            connect(settings_widget_, SIGNAL(machineLearningModeToggled(bool)),
                    this, SLOT(ToggleMachineLearningMode(bool)));
        }
        else
        {
            settings_widget_->showNormal();
            settings_widget_->activateWindow();
        }
    }

    void SimpleGCS::DestroySettingsWidget()
    {
        delete settings_widget_;
        settings_widget_ = nullptr;
    }

    void SimpleGCS::ToggleMachineLearningMode(bool toggle)
    {
        central_ui_.uavQueriesFame->setVisible(toggle);
        central_ui_.uavQueriesFame->setEnabled(toggle);

        for(int i = 0; i < NUM_UAV; i++)
            active_uavs[i]->setOnlineMode(toggle);
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

        for(auto const& node : nodes)
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
//                if(binarySearch(uav.first, 0, gcs->NUM_UAV-1) == -1)
                if(gcs->all_uav_stat.count(uav.first) == 0)
                    gcs->all_uav_stat.insert(std::pair<int, UavStatus>(uav.first, UavStatus::null));

                if(gcs->all_uav_stat[uav.first] != UavStatus::active)
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

    void SimpleGCSHelper::monitorConnections()
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

    void SimpleGCSHelper::monitor()
    {
        forever
        {
            monitorUavNamespace();
            monitorConnections();
            runUavs();
        }
    }

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, SimpleGCS, rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
//PLUGINLIB_EXPORT_CLASS(rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
