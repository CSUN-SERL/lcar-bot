#include <rqt_gcs/simple_gcs.h>

namespace rqt_gcs {

SimpleGCS::SimpleGCS()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
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
    //uavCondWidgetArr[i].VehicleNameLine->setText(std::to_string(i).append("UAV ",0).c_str());
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
  for(int i = 0; i < NUM_UAV; i++){
      central_ui_.UAVListLayout->addWidget(uavListWidgetArr[i]);
  }

   //Setup mission progress widgets
   uavStatWidget_->setWindowTitle("Flight State");
   missionProgressWidget_->setWindowTitle("Mission Control");

   //setup button logic for the widgets

   connect(mpUi_.executePlayButton,SIGNAL(clicked()),this, SLOT(ExecutePlay()));
   connect(mpUi_.cancelPlayButton,SIGNAL(clicked()),this, SLOT(CancelPlay()));
   connect(mpUi_.scoutBuildingButton, SIGNAL(clicked()),this, SLOT(ScoutBuilding()));
   connect(mpUi_.stopMissionButton,SIGNAL(clicked()),this, SLOT(StopQuad()));
   connect(mpUi_.changeFlightModeButton,SIGNAL(clicked()),this, SLOT(ChangeFlightMode()));
   connect(mpUi_.viewAccessPointsButton,SIGNAL(clicked()),this, SLOT(InitializeAccessPointsMenu()));
   //connect(apmUi_.//, SIGNAL(()),this,SLOT(CloseAccessPointsMenu()));
   connect(mpUi_.armButton, SIGNAL(clicked()),this,SLOT(ArmSelectedQuad()));
   connect(mpUi_.disarmButton, SIGNAL(clicked()),this,SLOT(DisarmSelectedQuad()));


   //Setup UAV lists select functions
   signal_mapper = new QSignalMapper(this);
   for(int i = 0; i < NUM_UAV; i++){
    signal_mapper->setMapping(uavCondWidgetArr[i].VehicleSelectButton, i);
    connect(uavCondWidgetArr[i].VehicleSelectButton,SIGNAL(clicked()),signal_mapper, SLOT(map()));
   }
   connect(signal_mapper, SIGNAL(mapped(int)),this, SLOT(QuadSelect(int)));

   //Setup update timer
   update_timer = new QTimer(this);
   connect(update_timer, SIGNAL(timeout()), this, SLOT(TimedUpdate()));
   update_timer->start(100);
}

//Timed update of for the GCS
void SimpleGCS::TimedUpdate(){

  quad_id.setNum(cur_uav+1);
  SimpleControl quad = quadrotors[cur_uav];

  temp_data = quad.GetState().mode.c_str();
  usUi_.flightModeDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().yaw,'f',2);
  usUi_.yawDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().roll,'f',2);
  usUi_.rollDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().pitch,'f',2);
  usUi_.pitchDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().altitude,'f',2);
  usUi_.altitudeDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().vertical_speed,'f',2);
  usUi_.verticalSpaceDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().ground_speed,'f',2);
  usUi_.horizontalSpaceDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().heading,'f',2);
  usUi_.headingDisplay->setText(temp_data);

  temp_data.setNum(quad.GetDistanceToWP());
  usUi_.waypointDisplay->setText(temp_data);

  temp_data.setNum(quad.GetBatteryStatus().remaining*100);
  usUi_.batteryProgressBar->setValue(temp_data.toInt());

  temp_data = "UAV ";
  temp_data += quad_id;

  mpUi_.uavNameEdit->setText(temp_data);

  mpUi_.missionProgressBar->setValue(quad.GetMissionProgress()*100);

  this->UpdatePFD();

  //Update all UAV's in the system
  for(int index = 0; index < NUM_UAV; index++){
    quadrotors[index].Run();
  }

  //Update Uav List widgets
  for(int i = 0; i < NUM_UAV; i++){
   temp_data.setNum(quadrotors[i].GetBatteryStatus().remaining*100);
   uavCondWidgetArr[i].VehicleBatteryLine->setText(temp_data);
  }

}


void SimpleGCS::ExecutePlay(){
	if(mpUi_.playComboBox->currentIndex() == 0){
		ROS_INFO_STREAM("Play 1 initiated");
        }
        else if(mpUi_.playComboBox->currentIndex() == 1){
		ROS_INFO_STREAM("Play 2 initiated");
        }
        else if(mpUi_.playComboBox->currentIndex() == 2){
		ROS_INFO_STREAM("Play 3 initiated");
	}
}


void SimpleGCS::CancelPlay(){

}


void SimpleGCS::ScoutBuilding(){
	if(mpUi_.buildingsComboBox->currentIndex() == 0){
		ROS_INFO_STREAM("Scouting Building 1");
            quadrotors[cur_uav].ScoutBuilding(0,0,0.5);
        }
        else if(mpUi_.buildingsComboBox->currentIndex() == 1){
		ROS_INFO_STREAM("Scouting Building 2");
        }
        else if(mpUi_.buildingsComboBox->currentIndex() == 2){
		ROS_INFO_STREAM("Scouting Building 3");
	}
}

void SimpleGCS::StopQuad(){
	quadrotors[cur_uav].SetMode("AUTO.RTL");
}

void SimpleGCS::ChangeFlightMode(){
	if(mpUi_.flightModeComboBox->currentIndex() == 0){
		 ROS_INFO_STREAM("Quadrotor Stablized");
		 quadrotors[cur_uav].SetMode("STABILIZED");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 1){
		ROS_INFO_STREAM("Quadrotor RTL");
		quadrotors[cur_uav].SetMode("AUTO.RTL");
        }
        else if(mpUi_.flightModeComboBox->currentIndex() == 2){
		ROS_INFO_STREAM("Quadrotor Loiter");
		quadrotors[cur_uav].SetMode("AUTO.LOITER");
	}
	else if(mpUi_.flightModeComboBox->currentIndex() == 3){
		ROS_INFO_STREAM("Quadrotor Land");
		quadrotors[cur_uav].SetMode("AUTO.LAND");
	}
	else if(mpUi_.flightModeComboBox->currentIndex() == 4){
		ROS_INFO_STREAM("Quadrotor alt hold");
		quadrotors[cur_uav].SetMode("ALTCTL");
	}
	else if(mpUi_.flightModeComboBox->currentIndex() == 5){

	}
	else if(mpUi_.flightModeComboBox->currentIndex() == 6){

	}
}

void SimpleGCS::InitializeAccessPointsMenu(){

    AccessPoint new_point;
     AccessPoint new_point2;
   // new_point.SetAltitude();

    std::vector<AccessPoint> accessPointsVector = quadrotors[cur_uav].GetAccessPoints();

   // quadrotors[cur_uav].GetAccessPoints().push_back(new_point);
     accessPointsVector.push_back(new_point);
      accessPointsVector.push_back(new_point2);
    //Get our new number of Access points and create widgets for each access points

    //int numOfAccessPoints = 2;
    //int numOfAccessPoints = quadrotors[cur_uav].GetAccessPoints().size();
      int numOfAccessPoints = accessPointsVector.size();

    QWidget* apWidgets[numOfAccessPoints];
    Ui::AccessPointStatsWidget apUiWidgets[numOfAccessPoints];


    //clear out old list of Access points
    for(int i = 0; i < accessPointsQWidgets_.size(); i++)
    {
        delete accessPointsQWidgets_.at(i);
        accessPointsQWidgets_.at(i) = NULL;

    }
     accessPointsQWidgets_.clear();



     //create  a new list of access points if there are any access points
     if( numOfAccessPoints != 0){
        for( int i = 0; i < numOfAccessPoints; i++){
           // AccessPoint accessPoint = quadrotors[cur_uav].GetAccessPoints().at(i);
              AccessPoint accessPoint = accessPointsVector.at(i);
            accessPointsQWidgets_.push_back(apWidgets[i]);
            accessPointsQWidgets_.at(i) = new QWidget();
            apUiWidgets[i].setupUi(accessPointsQWidgets_.at(i));

            //access point name
            access_point_id.setNum(i+1);
            access_point_temp_data = "Building ";
            access_point_temp_data += access_point_id;
            apUiWidgets[i].buildingNameLine->setText(access_point_temp_data);

            //altitude
            access_point_temp_data.setNum(accessPoint.GetAltitude().data,'f',2);
            apUiWidgets[i].altitudeLineEdit->setText(access_point_temp_data);

            //heading
             access_point_temp_data.setNum(accessPoint.GetHeading().data,'f',2);
             apUiWidgets[i].compassHeadingLineEdit->setText(access_point_temp_data);

             //image
             //accessPoint.GetImage().
             // apUiWidgets[i].

             //location longitude
             access_point_temp_data.setNum(accessPoint.GetLocation().longitude,'f',2);
             apUiWidgets[i].longitudeLineEdit->setText(access_point_temp_data);

             //location latitude
             access_point_temp_data.setNum(accessPoint.GetLocation().latitude,'f',2);
             apUiWidgets[i].latitudeLineEdit->setText(access_point_temp_data);


            //time
            access_point_temp_data.setNum(accessPoint.GetTime().toSec(),'f',6);
            apUiWidgets[i].captureTimeLineEdit->setText(access_point_temp_data);

            apmUi_.AccessPointMenuLayout->addWidget(accessPointsQWidgets_.at(i));
        }
    }

    // show the menu
     temp_data = "UAV ";
     quad_id.setNum(cur_uav+1);
     temp_data += quad_id;
     apmUi_.uavNameLineEdit->setText(temp_data);
     apmQWidget_->show();
     //test++;
}

void SimpleGCS::RefreshAccessPoints(){
    ROS_INFO_STREAM("Access point menu closed");
    delete accessPointsQWidgets_.at(0);
    accessPointsQWidgets_.pop_back();
}

void SimpleGCS::QuadSelect(int quadNumber){
	cur_uav = quadNumber;
}

void SimpleGCS::ArmSelectedQuad(){
	quadrotors[cur_uav].Arm(true);
}

void SimpleGCS::DisarmSelectedQuad(){
	quadrotors[cur_uav].Arm(false);
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
  pfd_ui.widgetPFD->setRoll       ((quadrotors[cur_uav].GetFlightState().roll)*180);
  pfd_ui.widgetPFD->setPitch      ((quadrotors[cur_uav].GetFlightState().pitch)*90);
  pfd_ui.widgetPFD->setHeading    (quadrotors[cur_uav].GetFlightState().heading);
  pfd_ui.widgetPFD->setAirspeed   (quadrotors[cur_uav].GetFlightState().ground_speed);
  pfd_ui.widgetPFD->setAltitude   (quadrotors[cur_uav].GetFlightState().altitude);
  pfd_ui.widgetPFD->setClimbRate  (quadrotors[cur_uav].GetFlightState().vertical_speed);

  pfd_ui.widgetPFD->update();
}

void SimpleGCS::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
    conversion_mat_ = cv_ptr->image;
    QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    ivUi_.image_frame->setImage(image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error in image subsrcriber: %s", e.what());
  }
}

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, SimpleGCS, rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
//PLUGINLIB_EXPORT_CLASS(rqt_gcs::SimpleGCS, rqt_gui_cpp::Plugin)
