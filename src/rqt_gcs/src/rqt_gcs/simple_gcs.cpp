#include <rqt_gcs/simple_gcs.h>
#include <QDesktopWidget>

namespace rqt_gcs
{

    SimpleGCS::SimpleGCS()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
    , settings_widget_(nullptr)
    {
        //Constructor is called first before initPlugin function
        setObjectName("LCAR Bot GCS");
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
        
        //For each of the Uav condition widgets
        //we set up the ui of the uavcondition widget(name, selection num)
        for(int i = 0; i < NUM_UAV; i++){
            temp_data = "UAV ";
            quad_id.setNum(i+1);
            temp_data += quad_id;
            uavListWidgetArr[i] = new QWidget();
            uavCondWidgetArr[i].setupUi(uavListWidgetArr[i]);
            uavCondWidgetArr[i].VehicleSelectButton->setText(std::to_string(i+1).c_str());
            uavCondWidgetArr[i].VehicleNameLine->setText(temp_data);
        }

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
        for(int i = 0; i < NUM_UAV; i++)
        {
            central_ui_.UAVListLayout->addWidget(uavListWidgetArr[i]);
        }

        //Setup mission progress widgets
        uavStatWidget_->setWindowTitle("Flight State");
        missionProgressWidget_->setWindowTitle("Mission Control");

        //setup button logic for the widgets

        connect(mpUi_.executePlayButton, SIGNAL(clicked()), this, SLOT(ExecutePlay()));
        connect(mpUi_.cancelPlayButton, SIGNAL(clicked()), this, SLOT(CancelPlay()));
        connect(mpUi_.scoutBuildingButton, SIGNAL(clicked()), this, SLOT(ScoutBuilding()));
        connect(mpUi_.stopMissionButton, SIGNAL(clicked()), this, SLOT(StopQuad()));
        connect(mpUi_.changeFlightModeButton, SIGNAL(clicked()), this, SLOT(ChangeFlightMode()));
        connect(mpUi_.viewAccessPointsButton, SIGNAL(clicked()), this, SLOT(RefreshAccessPointsMenu()));
        connect(mpUi_.armButton, SIGNAL(clicked()), this, SLOT(ArmSelectedQuad()));
        connect(mpUi_.disarmButton, SIGNAL(clicked()), this, SLOT(DisarmSelectedQuad()));

        //Setup UAV lists select functions
        signal_mapper = new QSignalMapper(this);
        signal_mapper2 = new QSignalMapper(this);
        acceptDoorMapper = new QSignalMapper(this);
        denyDoorMapper = new QSignalMapper(this);

        for(int i = 0; i < NUM_UAV; i++)
        {
            signal_mapper->setMapping(uavCondWidgetArr[i].VehicleSelectButton, i);
            connect(uavCondWidgetArr[i].VehicleSelectButton, SIGNAL(clicked()), signal_mapper, SLOT(map()));
        }
        connect(signal_mapper, SIGNAL(mapped(int)), this, SLOT(QuadSelect(int)));

        connect(acceptDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(AcceptDoorQuery(QWidget*)));
        connect(denyDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(DenyDoorQuery(QWidget*)));

        //Setup update timer
        update_timer = new QTimer(this);
        connect(update_timer, SIGNAL(timeout()), this, SLOT(TimedUpdate()));

        //30 hz
        update_timer->start(33);


        //set up access points
        accessPointsVector = quadrotors[0].GetRefAccessPoints();
        
        initializeSettings();
    }

    //Timed update of for the GCS

    void SimpleGCS::TimedUpdate()
    {

        timeCounter++;


        if(timeCounter >= 30)
        {
            //ROS_INFO_STREAM("MSGs UPdated");
            UpdateMsgQuery();
            timeCounter = 0;
        }


        quad_id.setNum(cur_uav + 1);
        SimpleControl quad = quadrotors[cur_uav];

        temp_data = quad.GetState().mode.c_str();
        temp_data += ":";
        temp_data += quad.GetState().armed ? "Armed" : "Disarmed";
        usUi_.flightModeDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().yaw, 'f', 2);
        usUi_.yawDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().roll, 'f', 2);
        usUi_.rollDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().pitch, 'f', 2);
        usUi_.pitchDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().altitude, 'f', 2);
        usUi_.altitudeDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().vertical_speed, 'f', 2);
        usUi_.verticalSpaceDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().ground_speed, 'f', 2);
        usUi_.horizontalSpaceDisplay->setText(temp_data);

        temp_data.setNum(quad.GetFlightState().heading, 'f', 2);
        usUi_.headingDisplay->setText(temp_data);

        temp_data.setNum(quad.GetDistanceToWP());
        usUi_.waypointDisplay->setText(temp_data);

        temp_data.setNum(quad.GetBatteryStatus().remaining * 100);
        usUi_.batteryProgressBar->setValue(temp_data.toInt());

        temp_data = "UAV ";
        temp_data += quad_id;

        mpUi_.uavNameEdit->setText(temp_data);

        mpUi_.missionProgressBar->setValue(quad.GetMissionProgress()*100);

        this->UpdatePFD();

        //Update all UAV's in the system
        for(int index = 0; index < NUM_UAV; index++)
        {
            quadrotors[index].Run();
        }

        //Update Uav List widgets
        for(int i = 0; i < NUM_UAV; i++)
        {
            temp_data.setNum(quadrotors[i].GetBatteryStatus().remaining * 100);
            uavCondWidgetArr[i].VehicleBatteryLine->setText(temp_data);
            temp_data = quadrotors[i].GetState().mode.c_str();
            uavCondWidgetArr[i].VehicleConditionLine->setText(temp_data);
        }

    }

    void SimpleGCS::UpdateMsgQuery()
    {
        for(int i = pictureMsgQWidgets_.size() - 1; i >= 0; i--)
        {
            central_ui_.PictureMsgLayout->removeWidget(pictureMsgQWidgets_.at(i));
            delete pictureMsgQWidgets_.at(i);
            pictureMsgQWidgets_.at(i) = NULL;
            pictureMsgQWidgets_.pop_back();

        }
        pictureMsgQWidgets_.clear();


        pictureQueryVector = quadrotors[cur_uav].GetDoorQueries();

        int numOfPictureMsg = pictureQueryVector->size();

        QWidget * pmWidgets[numOfPictureMsg];
        Ui::PictureMsgWidget pmUiWidgets[numOfPictureMsg];
        QWidget * imgWidget[numOfPictureMsg];
        Ui::ImageViewWidget imgUiWidget[numOfPictureMsg];


        for(int i = 0; i < numOfPictureMsg; i++)
        {
            //add widget to the list
            pictureMsgQWidgets_.push_back(pmWidgets[i]);
            pictureMsgQWidgets_.at(i) = new QWidget();
            imgWidget[i] = new QWidget();

            //retrieve Query msg for door image
            query_msgs::Door doorQuery = pictureQueryVector->at(i);

            QImage image(doorQuery.picture.data.data(), doorQuery.picture.width, 
                         doorQuery.picture.height, doorQuery.picture.step, 
                         QImage::Format_RGB888);

            //set up the ui
            pmUiWidgets[i].setupUi(pictureMsgQWidgets_.at(i));
            imgUiWidget[i].setupUi(imgWidget[i]);

            //set image to the image widget
            imgUiWidget[i].image_frame->setImage(image);

            // add the image widget to the Layout
            pmUiWidgets[i].PictureLayout->addWidget(imgWidget[i]);

            //map signal for yes button
            acceptDoorMapper->setMapping(pmUiWidgets[i].yesButton, 
                                         pictureMsgQWidgets_.at(i));
            connect(pmUiWidgets[i].yesButton, SIGNAL(clicked()), 
                    acceptDoorMapper, SLOT(map()));

            //map signal for yes button
            denyDoorMapper->setMapping(pmUiWidgets[i].rejectButton, 
                                       pictureMsgQWidgets_.at(i));
            connect(pmUiWidgets[i].rejectButton, SIGNAL(clicked()), 
                    denyDoorMapper, SLOT(map()));

            central_ui_.PictureMsgLayout->addWidget(pictureMsgQWidgets_.at(i));
        }
        
    }

    void SimpleGCS::AcceptDoorQuery(QWidget *qw)
    {
        // ROS_INFO_STREAM("Accepted");
        int index = central_ui_.PictureMsgLayout->indexOf(qw);
        query_msgs::Door doormsg = pictureQueryVector->at(index);
        // sensor_msgs::Image immsg = pictureQueryVector->at(index);
        // ROS_INFO_STREAM("door number " << index);
        pictureQueryVector->erase(pictureQueryVector->begin() + index);
        pictureMsgQWidgets_.erase(pictureMsgQWidgets_.begin() + index);

        central_ui_.PictureMsgLayout->removeWidget(qw);
        delete qw;

        //doormsg.accepted = true;
        // quadrotors[cur_uav].SendDoorResponse(doormsg);
    }

    void SimpleGCS::DenyDoorQuery(QWidget * qw)
    {
        // ROS_INFO_STREAM("Denyed");

        int index = central_ui_.PictureMsgLayout->indexOf(qw);
        query_msgs::Door doormsg = pictureQueryVector->at(index);
        // sensor_msgs::Image imgmsg = pictureQueryVector->at(index);
        //  ROS_INFO_STREAM("door number " << index);
        pictureQueryVector->erase(pictureQueryVector->begin() + index);
        pictureMsgQWidgets_.erase(pictureMsgQWidgets_.begin() + index);

        central_ui_.PictureMsgLayout->removeWidget(qw);
        delete qw;

        //doormsg.accepted = false;
        //quadrotors[cur_uav].SendDoorResponse(doormsg);

    }

    void SimpleGCS::ExecutePlay()
    {
        int play_num = mpUi_.playComboBox->currentIndex();

        if(play_num == 0)
        {
            for(int i = 0; i < NUM_UAV; i++)
            {
                std::string file_name = "building" + std::to_string(i + 1) + ".txt";

                quadrotors[i].Arm(true);
                quadrotors[i].ScoutBuilding(GetMission(file_name));
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

        for(int i = 0; i < NUM_UAV; i++)
        {
            quadrotors[i].SetRTL();
        }

    }

    void SimpleGCS::ScoutBuilding()
    {
        int building_index = mpUi_.buildingsComboBox->currentIndex() + 1;
        std::string file_name = "building" + std::to_string(building_index) + ".txt";

        quadrotors[cur_uav].ScoutBuilding(GetMission(file_name));
        ROS_INFO_STREAM("Scouting Building " << building_index);
    }

    void SimpleGCS::StopQuad()
    {
        quadrotors[cur_uav].SetRTL();
    }

    void SimpleGCS::ChangeFlightMode()
    {
        if(mpUi_.flightModeComboBox->currentIndex() == 0)
        {
            ROS_INFO_STREAM("Quadrotor Stablized");
            quadrotors[cur_uav].SetMode("STABILIZED");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 1)
        {
            ROS_INFO_STREAM("Quadrotor Loiter");
            quadrotors[cur_uav].SetMode("AUTO.LOITER");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 2)
        {
            ROS_INFO_STREAM("Quadrotor Land");
            quadrotors[cur_uav].SetMode("AUTO.LAND");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 3)
        {
            ROS_INFO_STREAM("Altitude Hold");
            quadrotors[cur_uav].SetMode("ALTCTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 4)
        {
            ROS_INFO_STREAM("Position Hold");
            quadrotors[cur_uav].SetMode("POSCTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 5)
        {
            ROS_INFO_STREAM("Quadrotor Return-To-Launch");
            quadrotors[cur_uav].SetRTL();
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 6)
        {
            ROS_INFO_STREAM("Quadrotor Auto");
            quadrotors[cur_uav].SetMode("AUTO");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 7)
        {
            ROS_INFO_STREAM("Quadrotor Offboard");
            quadrotors[cur_uav].SetMode("OFFBOARD");
        }
    }

    void SimpleGCS::RefreshAccessPointsMenu()
    {

        //clear out old list of Access points widgets
        for(int i = accessPointsQWidgets_.size() - 1; i >= 0; i--)
        {
            apmUi_.AccessPointMenuLayout->removeWidget(accessPointsQWidgets_.at(i));
            delete accessPointsQWidgets_.at(i);
            accessPointsQWidgets_.at(i) = NULL;
            accessPointsQWidgets_.pop_back();

        }
        accessPointsQWidgets_.clear();
        disconnect(signal_mapper2, SIGNAL(mapped(QWidget*)), 
                   this, SLOT(DeleteAccessPoint(QWidget*)));

        //retreive access points
        accessPointsVector = quadrotors[cur_uav].GetRefAccessPoints();

        //Get our new number of Access points and create widgets for each access points
        int numOfAccessPoints = accessPointsVector->size();

        //create widgets for access points
        QWidget * apWidgets[numOfAccessPoints];
        Ui::AccessPointStatsWidget apUiWidgets[numOfAccessPoints];
        QWidget * imageWidget[numOfAccessPoints];
        Ui::ImageViewWidget imageUiWidget[numOfAccessPoints];

        //create  a new list of access points if there are any access points
        if(numOfAccessPoints != 0)
        {
            for(int i = 0; i < numOfAccessPoints; i++)
            {

                //retrive access point
                AccessPoint accessPoint = accessPointsVector->at(i);
                sensor_msgs::Image acImage = accessPoint.GetImage();

                QImage image(acImage.data.data(), acImage.width, acImage.height, 
                             acImage.step, QImage::Format_RGB888);

                //add widget to the list
                accessPointsQWidgets_.push_back(apWidgets[i]);
                accessPointsQWidgets_.at(i) = new QWidget();
                imageWidget[i] = new QWidget();

                //set up the ui
                imageUiWidget[i].setupUi(imageWidget[i]);
                apUiWidgets[i].setupUi(accessPointsQWidgets_.at(i));


                //apUiWidgets[i].
                //map signal to the delete button
                signal_mapper2->setMapping(apUiWidgets[i].deleteAccessPointButton, 
                                           accessPointsQWidgets_.at(i));

                connect(apUiWidgets[i].deleteAccessPointButton, SIGNAL(clicked()), 
                        signal_mapper2, SLOT(map()));


                //access point name
                access_point_id.setNum(i + 1);
                access_point_temp_data = "Building ";
                access_point_temp_data += access_point_id;
                apUiWidgets[i].buildingNameLine->setText(access_point_temp_data);

                //altitude
                access_point_temp_data.setNum(accessPoint.GetAltitude().data, 'f', 2);
                apUiWidgets[i].altitudeLineEdit->setText(access_point_temp_data);

                //heading
                access_point_temp_data.setNum(accessPoint.GetHeading().data, 'f', 2);
                apUiWidgets[i].compassHeadingLineEdit->setText(access_point_temp_data);

                //image
                imageUiWidget[i].image_frame->setImage(image);
                //ivUi_.image_frame->setImage(image);

                apUiWidgets[i].PictureLayout->addWidget(imageWidget[i]);


                //location longitude
                access_point_temp_data.setNum(accessPoint.GetLocation().longitude, 'f', 2);
                apUiWidgets[i].longitudeLineEdit->setText(access_point_temp_data);

                //location latitude
                access_point_temp_data.setNum(accessPoint.GetLocation().latitude, 'f', 2);
                apUiWidgets[i].latitudeLineEdit->setText(access_point_temp_data);

                //time
                access_point_temp_data.setNum(accessPoint.GetTime().toSec(), 'f', 6);
                apUiWidgets[i].captureTimeLineEdit->setText(access_point_temp_data);

                apmUi_.AccessPointMenuLayout->addWidget(accessPointsQWidgets_.at(i));
            }
            connect(signal_mapper2, SIGNAL(mapped(QWidget*)), 
                    this, SLOT(DeleteAccessPoint(QWidget*)));

        }

        // show the menu
        temp_data = "UAV ";
        quad_id.setNum(cur_uav + 1);
        temp_data += quad_id;
        apmUi_.uavNameLineEdit->setText(temp_data);
        apmQWidget_->show();

        //test++;
    }

    void SimpleGCS::DeleteAccessPoint(QWidget* w)
    {
        // ROS_INFO_STREAM("Access point menu closed");

        int deleteIndex = apmUi_.AccessPointMenuLayout->indexOf(w);
        ROS_INFO_STREAM("Access point %d Deleted" << deleteIndex);

        accessPointsVector->erase(accessPointsVector->begin() + deleteIndex - 1);
        accessPointsQWidgets_.erase(accessPointsQWidgets_.begin() + deleteIndex - 1);

        apmUi_.AccessPointMenuLayout->removeWidget(w);
        delete w;


    }

    void SimpleGCS::QuadSelect(int quadNumber)
    {
        cur_uav = quadNumber;
        sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(quadNumber + 1) + "/stereo_cam/left/image_raw", 
                                         1, &SimpleGCS::ImageCallback, this);
    }

    void SimpleGCS::ArmSelectedQuad()
    {
        quadrotors[cur_uav].Arm(true);
    }

    void SimpleGCS::DisarmSelectedQuad()
    {
        quadrotors[cur_uav].Arm(false);
    }

    void SimpleGCS::shutdownPlugin(){
        // TODO unregister all publishers here
    }

    void SimpleGCS::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const{
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void SimpleGCS::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings){
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
        pfd_ui.widgetPFD->setRoll((quadrotors[cur_uav].GetFlightState().roll)*180);
        pfd_ui.widgetPFD->setPitch((quadrotors[cur_uav].GetFlightState().pitch)*90);
        pfd_ui.widgetPFD->setHeading(quadrotors[cur_uav].GetFlightState().heading);
        pfd_ui.widgetPFD->setAirspeed(quadrotors[cur_uav].GetFlightState().ground_speed);
        pfd_ui.widgetPFD->setAltitude(quadrotors[cur_uav].GetFlightState().altitude);
        pfd_ui.widgetPFD->setClimbRate(quadrotors[cur_uav].GetFlightState().vertical_speed);

        pfd_ui.widgetPFD->update();
    }

    query_msgs::Target SimpleGCS::GetMission(std::string fileName)
    {

        float pos_x, pos_y, pos_z, radius;
        query_msgs::Target building;
        std::ifstream fileIn(fileName);
        fileIn >> pos_x;
        fileIn >> pos_y;
        fileIn >> pos_z;
        fileIn >> radius;
        //fileIn >> yaw;

        building.target_point.position.x = pos_x;
        building.target_point.position.y = pos_y;
        building.target_point.position.z = pos_z;
        building.radius = radius;
        //building.orientation.w = yaw;

        return building;
    }


    void SimpleGCS::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
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

    // SettingsWidget and QSettings related stuff
    
    void SimpleGCS::initializeSettings()
    {
        //initialize settings and uav queries display 
        settings_ = new QSettings("SERL", "LCAR_Bot");
        
        settings_->beginGroup("general_tab");
        if(settings_->value("machine_learning","online").toString() == "online")
            central_ui_.uavQueriesFame->setVisible(true);
        else
            central_ui_.uavQueriesFame->setVisible(false);
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
            
            connect(settings_widget_, SIGNAL(showUavQueriesMenu(bool)), 
                    this, SLOT(ShowUavQueriesMenu(bool)));
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
    
    void SimpleGCS::ShowUavQueriesMenu(bool visible)
    {
        central_ui_.uavQueriesFame->setVisible(visible);
        central_ui_.uavQueriesFame->setEnabled(visible);
    }

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, SimpleGCS, rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
//PLUGINLIB_EXPORT_CLASS(rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
