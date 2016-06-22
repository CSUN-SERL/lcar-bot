#include <rqt_gcs/simple_gcs.h>
#include <map>

namespace rqt_gcs
{

    SimpleGCS::SimpleGCS()
    : rqt_gui_cpp::Plugin()
    , widget_(nullptr)
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
        
        NUM_UAV = 0;
        timeCounter = 0;
        cur_uav = -1;
        
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
        connect(mpUi_.viewAccessPointsButton, SIGNAL(clicked()), this, SLOT(RefreshAccessPointsMenu()));
        connect(mpUi_.armButton, SIGNAL(clicked()), this, SLOT(ArmSelectedQuad()));
        connect(mpUi_.disarmButton, SIGNAL(clicked()), this, SLOT(DisarmSelectedQuad()));

        //Setup UAV lists select functions
        signal_mapper = new QSignalMapper(this);
        signal_mapper2 = new QSignalMapper(this);
        acceptDoorMapper = new QSignalMapper(this);
        denyDoorMapper = new QSignalMapper(this);

        connect(signal_mapper, SIGNAL(mapped(int)), this, SLOT(QuadSelected(int)));
        connect(acceptDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(AcceptDoorQuery(QWidget*)));
        connect(denyDoorMapper, SIGNAL(mapped(QWidget*)), this, SLOT(DenyDoorQuery(QWidget*)));

        //Setup update timer
        update_timer = new QTimer(this);
        connect(update_timer, SIGNAL(timeout()), this, SLOT(TimedUpdate()));
        
        initializeSettings();
        initializeThreads();
        
        //30 hz :1000/30 = 33.33...
        update_timer->start(33);

    }
    
    
    void SimpleGCS::addUav(int uav_id)
    {
        uav_mutex.lock();
        
        ROS_WARN_STREAM("Adding UAV with id: " << uav_id);
        
        temp_data = "UAV ";
        quad_id.setNum(uav_id);
        temp_data += quad_id;
        
        auto cond_iter = uavCondWidgetArr.begin();
        auto list_iter = uavListWidgetArr.begin();
        auto quad_iter = quadrotors.begin();
        int index = 0;
        
        while(quad_iter != quadrotors.end() && uav_id > (*quad_iter)->id)
        {               
            ++cond_iter;
            ++list_iter;
            ++quad_iter;
            ++index;
        }

        uavCondWidgetArr.insert(cond_iter, new Ui::UAVConditionWidget());
        uavListWidgetArr.insert(list_iter, new QWidget());
        quadrotors.insert(quad_iter, new SimpleControl(uav_id));
        
        uavCondWidgetArr[index]->setupUi(uavListWidgetArr[index]);
        uavCondWidgetArr[index]->VehicleSelectButton->setText(std::to_string(uav_id).c_str());
        uavCondWidgetArr[index]->VehicleNameLine->setText(temp_data);
            
        for(int i = index; i < NUM_UAV; i++)
            central_ui_.UAVListLayout->removeWidget(uavListWidgetArr[i]);  
        
        for(int i = index; i < quadrotors.size(); i++)
        {
            central_ui_.UAVListLayout->addWidget(uavListWidgetArr[i]);
            signal_mapper->setMapping(uavCondWidgetArr[i]->VehicleSelectButton, i);
            connect(uavCondWidgetArr[i]->VehicleSelectButton, SIGNAL(clicked()),
                    signal_mapper, SLOT(map()));
        }
        
        NUM_UAV++;
        
        if(NUM_UAV == 1)
            selectQuad(0);
        
        uav_mutex.unlock();
    }
    
    void SimpleGCS::deleteUav(int index)
    {   
        uav_mutex.lock();
        
        central_ui_.UAVListLayout->removeWidget(uavListWidgetArr[index]);
        
        auto cond_iter = uavCondWidgetArr.begin();
        auto list_iter = uavListWidgetArr.begin();
        auto quad_iter = quadrotors.begin();

        while((*quad_iter)->id != quadrotors[index]->id)
        {
            ++cond_iter;
            ++list_iter;
            ++quad_iter;
        }
        
        delete uavCondWidgetArr[index];
        delete uavListWidgetArr[index];
        delete quadrotors[index];
        
        uavCondWidgetArr.erase(cond_iter);
        uavListWidgetArr.erase(list_iter);
        quadrotors.erase(quad_iter);
        
        for(int i = index; i < quadrotors.size(); i++)
        {
            signal_mapper->setMapping(uavCondWidgetArr[i]->VehicleSelectButton, i);
            connect(uavCondWidgetArr[i]->VehicleSelectButton, 
                    SIGNAL(clicked()), signal_mapper, SLOT(map()));
        }
        
        NUM_UAV--;
        
        if(NUM_UAV == 0)
            cur_uav= -1; // no more UAV's to select
        else if(NUM_UAV == 1 || (cur_uav == index && index == 0))
            selectQuad(0); // default to only remaining uav/first in the list
        else if(NUM_UAV > 1 && cur_uav == index)
            selectQuad(cur_uav-1); 
            //if the currently selected uav was deleted, choose the one in front of it
        
        uav_mutex.unlock();
    }
    
    void SimpleGCS::parseUavNamespace(std::map<int,int>& map)
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
    
    void SimpleGCS::MonitorUavNamespace()
    {   
        std::map<int,int> uav_map;
        parseUavNamespace(uav_map);
        
        if(NUM_UAV < uav_map.size())
        {
            for(auto const& iter : uav_map)
            {
                bool contains_id = false;
                for(int j = 0; j < quadrotors.size(); j++)
                {
                    if(iter.first == quadrotors[j]->id)
                        contains_id = true;
                }
                
                if(!contains_id)
                    addUav(iter.first);
            }             
        }
        else if(NUM_UAV > uav_map.size())
        {   
            for(int i = 0; i < quadrotors.size(); i++)
            {
                if(uav_map.count(quadrotors[i]->id) == 0)
                {
                    deleteUav(i);
                    i--;
                }
            }
        }
    }
        
    void SimpleGCS::MonitorConnection()
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
        
        for(int i = 0; i < quadrotors.size(); i++)
        {   
            SimpleControl* quad = quadrotors[i];
            QWidget* button = uavCondWidgetArr[i]->VehicleSelectButton;

            if(!quad->RecievedHeartbeat() )
            {
                 //ROS_WARN_STREAM("no heartbeat for UAV_" << quad->id);
                if(button->isEnabled()) // is the button already disabled?
                {
                    button->setEnabled(false);
                    button->setStyleSheet("background-color: rgb(80, 90, 110); color: rgb(150, 150, 150);");
                }
            }
            else
            {
                //ROS_INFO_STREAM("recieved heartbeat for UAV_" << quad->id);
                if(!button->isEnabled()) // is the button already enabled?
                {
                    button->setEnabled(true);
                    button->setStyleSheet("background-color: rgb(64, 89, 140); color: rgb(240, 240, 240);");
                }
            }
        }
    }    

    //Timed update of for the GCS

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
            UpdateMsgQuery();
            timeCounter = 0;
        }

        
        SimpleControl quad = *quadrotors[cur_uav];
        quad_id.setNum(quad.id);
        
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
        
        quad_id.setNum(quad.id);
        temp_data = "UAV ";
        temp_data += quad_id;

        mpUi_.uavNameEdit->setText(temp_data);

        mpUi_.missionProgressBar->setValue(quad.GetMissionProgress()*100);

        this->UpdatePFD();

        //Update all UAV's in the system
        for(int index = 0; index < NUM_UAV; index++)
        {
            quadrotors[index]->Run();
        }

        //Update Uav List widgets
        for(int i = 0; i < NUM_UAV; i++)
        {
            temp_data.setNum(quadrotors[i]->GetBatteryStatus().remaining * 100);
            uavCondWidgetArr[i]->VehicleBatteryLine->setText(temp_data);
            temp_data = quadrotors[i]->GetState().mode.c_str();
            uavCondWidgetArr[i]->VehicleConditionLine->setText(temp_data);
        }
        
    }

    void SimpleGCS::UpdateMsgQuery()
    {
        if(!central_ui_.uavQueriesFame->isEnabled() || NUM_UAV == 0)
            return;
        
        for(int i = pictureMsgQWidgets_.size() - 1; i >= 0; i--)
        {
            central_ui_.PictureMsgLayout->removeWidget(pictureMsgQWidgets_.at(i));
            delete pictureMsgQWidgets_.at(i);
            pictureMsgQWidgets_.at(i) = nullptr;
            pictureMsgQWidgets_.pop_back();
        }
        pictureMsgQWidgets_.clear();

        pictureQueryVector = quadrotors[cur_uav]->GetDoorQueries();

        int num_queries = pictureQueryVector->size();

        QWidget * pmWidgets[num_queries];
        Ui::PictureMsgWidget pmUiWidgets[num_queries];
        QWidget * imgWidget[num_queries];
        Ui::ImageViewWidget imgUiWidget[num_queries];

        for(int i = 0; i < num_queries; i++)
        {
            //add widget to the list
            pictureMsgQWidgets_.push_back(pmWidgets[i]);
            int size = pictureMsgQWidgets_.size();
            pictureMsgQWidgets_.at(size - 1) = new QWidget();
            imgWidget[i] = new QWidget();

            //retrieve Query msg for door image
            lcar_msgs::DoorPtr doorQuery = pictureQueryVector->at(i);

            QImage image(doorQuery->framed_picture.data.data(), doorQuery->framed_picture.width, 
                         doorQuery->framed_picture.height, doorQuery->framed_picture.step, 
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
        int index = central_ui_.PictureMsgLayout->indexOf(qw);
        lcar_msgs::DoorPtr door = pictureQueryVector->at(index);
        
        saveImage(true, "door",
            cv_bridge::toCvCopy((sensor_msgs::Image)door->original_picture, "rgb8")->image);
        
        // ROS_INFO_STREAM("door number " << index);
        pictureQueryVector->erase(pictureQueryVector->begin() + index);
        pictureMsgQWidgets_.erase(pictureMsgQWidgets_.begin() + index);

        central_ui_.PictureMsgLayout->removeWidget(qw);
        delete qw;
        
         door->accepted = true;
        //quadrotors[cur_uav]->SendDoorResponse(doormsg);
    }
    
    void SimpleGCS::DenyDoorQuery(QWidget * qw)
    {
        int index = central_ui_.PictureMsgLayout->indexOf(qw);
        lcar_msgs::DoorPtr door = pictureQueryVector->at(index);
        
        saveImage(false, "door",
            cv_bridge::toCvCopy((sensor_msgs::Image)door->original_picture, "rgb8")->image);
        
        //  ROS_INFO_STREAM("door number " << index);
        pictureQueryVector->erase(pictureQueryVector->begin() + index);
        pictureMsgQWidgets_.erase(pictureMsgQWidgets_.begin() + index);

        central_ui_.PictureMsgLayout->removeWidget(qw);
        delete qw;
        
        door->accepted = false;
        //quadrotors[cur_uav].SendDoorResponse(doormsg);
    }
    
    void SimpleGCS::saveImage(bool img_accepted, std::string ap_type, const cv::Mat& image)
    {   
        std::string path = image_root_path_.toStdString(), file;
        SimpleControl quad = *quadrotors[cur_uav];
        if(img_accepted)
        {
            path += "/accepted/" + ap_type + "/uav_" + std::to_string(quad.id);
            file = "img_" + std::to_string(quad.accepted_images++) + ".jpg";
        }
        else 
        {
            path += "/rejected/" + ap_type + "/uav_" + std::to_string(quad.id);
            file = "img_" + std::to_string(quad.rejected_images++) + ".jpg";
        }
                   
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

                quadrotors[i]->Arm(true);
                quadrotors[i]->ScoutBuilding(GetMission(file_name));
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
            quadrotors[i]->SetRTL();
        }

    }

    void SimpleGCS::ScoutBuilding()
    {
        if(NUM_UAV == 0)
            return;
        
        int building_index = mpUi_.buildingsComboBox->currentIndex() + 1;
        std::string file_name = "building" + std::to_string(building_index) + ".txt";

        quadrotors[cur_uav]->ScoutBuilding(GetMission(file_name));
        ROS_INFO_STREAM("Scouting Building " << building_index);
    }

    void SimpleGCS::StopQuad()
    {
        if(NUM_UAV == 0)
            return;
        
        quadrotors[cur_uav]->SetRTL();
    }

    void SimpleGCS::ChangeFlightMode()
    {
        if(NUM_UAV == 0)
            return;
        
        if(mpUi_.flightModeComboBox->currentIndex() == 0)
        {
            ROS_INFO_STREAM("Quadrotor Stablized");
            quadrotors[cur_uav]->SetMode("STABILIZED");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 1)
        {
            ROS_INFO_STREAM("Quadrotor Loiter");
            quadrotors[cur_uav]->SetMode("AUTO.LOITER");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 2)
        {
            ROS_INFO_STREAM("Quadrotor Land");
            quadrotors[cur_uav]->SetMode("AUTO.LAND");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 3)
        {
            ROS_INFO_STREAM("Altitude Hold");
            quadrotors[cur_uav]->SetMode("ALTCTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 4)
        {
            ROS_INFO_STREAM("Position Hold");
            quadrotors[cur_uav]->SetMode("POSCTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 5)
        {
            ROS_INFO_STREAM("Quadrotor Return-To-Launch");
            quadrotors[cur_uav]->SetRTL();
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 6)
        {
            ROS_INFO_STREAM("Quadrotor Auto");
            quadrotors[cur_uav]->SetMode("AUTO");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 7)
        {
            ROS_INFO_STREAM("Quadrotor Offboard");
            quadrotors[cur_uav]->SetMode("OFFBOARD");
        }
    }

    void SimpleGCS::RefreshAccessPointsMenu()
    {
        if(NUM_UAV == 0)
            return;
        
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
        accessPointsVector = quadrotors[cur_uav]->GetRefAccessPoints();

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
        quad_id.setNum(quadrotors[cur_uav]->id);
        temp_data += quad_id;
        apmUi_.uavNameLineEdit->setText(temp_data);
        apmQWidget_->show();

        //test++;
    }

    void SimpleGCS::DeleteAccessPoint(QWidget* w)
    {
        if(NUM_UAV == 0)
            return;
        
        // ROS_INFO_STREAM("Access point menu closed");

        int deleteIndex = apmUi_.AccessPointMenuLayout->indexOf(w);
        ROS_INFO_STREAM("Access point %d Deleted" << deleteIndex);

        accessPointsVector->erase(accessPointsVector->begin() + deleteIndex - 1);
        accessPointsQWidgets_.erase(accessPointsQWidgets_.begin() + deleteIndex - 1);

        apmUi_.AccessPointMenuLayout->removeWidget(w);
        delete w;
    }
    
    // slot gets called when user click on a uav button
    void SimpleGCS::QuadSelected(int quadNumber) 
    {
        selectQuad(quadNumber);  
    }
    
    //needed in addUav(int) and deleteUav(int)
    void SimpleGCS::selectQuad(int quadNumber)
    {
        if(NUM_UAV == 0)
            return;
        
        cur_uav = quadNumber;
        int uav_id = quadrotors[cur_uav]->id;
        sub_stereo = it_stereo.subscribe("/UAV" + std::to_string(uav_id) + "/stereo_cam/left/image_rect", 
                                         5, &SimpleGCS::ImageCallback, this);
        
        accessPointsVector = quadrotors[cur_uav]->GetRefAccessPoints();
        
        UpdateMsgQuery();
    }

    void SimpleGCS::ArmSelectedQuad()
    {
        if(NUM_UAV == 0)
            return;
        
        quadrotors[cur_uav]->Arm(true);
    }

    void SimpleGCS::DisarmSelectedQuad()
    {
        if(NUM_UAV == 0)
            return;
        
        quadrotors[cur_uav]->Arm(false);
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
        if(NUM_UAV == 0)
            return;
        
        pfd_ui.widgetPFD->setRoll((quadrotors[cur_uav]->GetFlightState().roll)*180);
        pfd_ui.widgetPFD->setPitch((quadrotors[cur_uav]->GetFlightState().pitch)*90);
        pfd_ui.widgetPFD->setHeading(quadrotors[cur_uav]->GetFlightState().heading);
        pfd_ui.widgetPFD->setAirspeed(quadrotors[cur_uav]->GetFlightState().ground_speed);
        pfd_ui.widgetPFD->setAltitude(quadrotors[cur_uav]->GetFlightState().altitude);
        pfd_ui.widgetPFD->setClimbRate(quadrotors[cur_uav]->GetFlightState().vertical_speed);

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

    void SimpleGCS::initializeThreads()
    {
        t_namespace_monitor = new QThread();
        uav_ns_timer = new QTimer();
        uav_ns_timer->setInterval(20);
        uav_ns_timer->moveToThread(t_namespace_monitor);          
        connect(uav_ns_timer, SIGNAL(timeout()),
                this, SLOT(MonitorUavNamespace())); 
        connect(t_namespace_monitor, SIGNAL(started()), 
                uav_ns_timer, SLOT(start()));                                        
        t_namespace_monitor->start();
        
        
        t_connection_monitor = new QThread();
        connection_timer = new QTimer();
        connection_timer->setInterval(20);
        connection_timer->moveToThread(t_connection_monitor);
        connect(connection_timer, SIGNAL(timeout()),
                this, SLOT(MonitorConnection()));
        connect(t_connection_monitor, SIGNAL(started()), 
                connection_timer, SLOT(start()));
        t_connection_monitor->start();
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
            quadrotors[i]->setOnlineMode(toggle);
    }

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, SimpleGCS, rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
//PLUGINLIB_EXPORT_CLASS(rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
