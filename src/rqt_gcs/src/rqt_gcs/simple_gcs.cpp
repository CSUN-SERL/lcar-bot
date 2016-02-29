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

  widget_ = new QWidget();
  missionCancelWidget_   = new QWidget();
  missionSelectWidget_   = new QWidget();
  missionProgressWidget_ = new QWidget();
  uavQuestionWidget_     = new QWidget();
  uavStatWidget_         = new QWidget();
  imageViewWidget_       = new QWidget();
  PFDQWidget             = new QWidget();
  missionConfirmWidget_  = new QWidget();

 for(int i = 0; i < NUM_UAV; i++){
    uavListWidgetArr[i] = new QWidget();
    uavCondWidgetArr[i].setupUi(uavListWidgetArr[i]);
    uavCondWidgetArr[i].VehicleSelectButton->setText(std::to_string(i+1).c_str());
    uavCondWidgetArr[i].VehicleNameLine->setText(std::to_string(i).append("UAV ",0).c_str());
 }

  //Setup the UI objects with the widgets
  central_ui_.setupUi(widget_);
  mcUi_.setupUi(missionCancelWidget_);
  mpUi_.setupUi(missionProgressWidget_);
  msUi_.setupUi(missionSelectWidget_);
  uqUi_.setupUi(uavQuestionWidget_);
  usUi_.setupUi(uavStatWidget_);
  ivUi_.setupUi(imageViewWidget_);
  pfd_ui.setupUi(PFDQWidget);
  mConfirmUi_.setupUi(missionConfirmWidget_);


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
   missionSelectWidget_->setWindowTitle("Mission Selection");
   uavStatWidget_->setWindowTitle("Flight State");
   missionProgressWidget_->setWindowTitle("Mission Control");
   connect(mpUi_.changeMissionButton,SIGNAL(clicked()),this, SLOT(MissionChange()));
   connect(mpUi_.stopMissionButton,SIGNAL(clicked()),this, SLOT(StopQuad()));
   connect(mpUi_.armButton, SIGNAL(clicked()),this,SLOT(ArmSelectedQuad()));
   connect(mpUi_.disarmButton, SIGNAL(clicked()),this,SLOT(DisarmSelectedQuad()));
   connect(mpUi_.flightModeComboBox,SIGNAL(activated(int)),this, SLOT(QuadMissionList(int)));
   
   //Setup Mission select widgets
   connect(msUi_.submitMission,SIGNAL(clicked()),this, SLOT(MissionConfirm()));
   connect(msUi_.cancelMission,SIGNAL(clicked()),this, SLOT(MissionChangeCancel()));
   connect(msUi_.missionComboBox,SIGNAL(activated(int)),this, SLOT(MissionSelect(int)));
  
   //Setup confirm widget
   connect(mConfirmUi_.yesButton,SIGNAL(clicked()),this, SLOT(MissionSubmit()));
   connect(mConfirmUi_.noButton,SIGNAL(clicked()),this, SLOT(MissionConfirmCancel()));

   //Setup UAV lists select functions
   //index 0 refers to quadrotor 1
   //index 1 refers to quadrotor 2
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

void SimpleGCS::QuadSelect(int quadNumber){
	cur_uav = quadNumber;
}

void SimpleGCS::ArmSelectedQuad(){
	quadrotors[cur_uav].Arm(true);
}

void SimpleGCS::DisarmSelectedQuad(){
	quadrotors[cur_uav].Arm(false);
}

void SimpleGCS::QuadMissionList(const int i){
    if(i == 0){
        ROS_INFO_STREAM("Quadrotor Stablized");
        quadrotors[cur_uav].SetMode("STABILIZED");
    }
    else if(i == 1){
        ROS_INFO_STREAM("Quadrotor RTL");
        quadrotors[cur_uav].SetMode("AUTO.RTL");
    }
    else if(i == 2){
        ROS_INFO_STREAM("Quadrotor Loiter");
        quadrotors[cur_uav].SetMode("AUTO.LOITER");
    }
    else if(i == 3){
        ROS_INFO_STREAM("Quadrotor alt hold");
        quadrotors[cur_uav].SetMode("ALTCTL");
    }
}

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

void SimpleGCS::MissionChange(){

    missionSelectWidget_->show();
}

void SimpleGCS::MissionChangeCancel(){

    missionSelectWidget_->close();
}

void SimpleGCS::StopQuad(){
	quadrotors[cur_uav].SetMode("AUTO.RTL");
}

void SimpleGCS::MissionSelect(const int i){

    if(i == 3){
     msUi_.playsComboBox->setEnabled(true);

    }
    else{
     msUi_.playsComboBox->setEnabled(false);
    }
}

void SimpleGCS::MissionSubmit(){
    ROS_INFO_STREAM("Mission Submitted");
    if(msUi_.missionComboBox->currentIndex() == 0){
        quadrotors[cur_uav].Arm(true);
    }
    else if(msUi_.missionComboBox->currentIndex() == 1){
        quadrotors[cur_uav].Arm(false);
    }
    else if(msUi_.missionComboBox->currentIndex() == 2){
        quadrotors[cur_uav].SetMode("AUTO.LAND");
    }
    else if(msUi_.missionComboBox->currentIndex() == 3){
        if(msUi_.playsComboBox->currentIndex() == 0){
            ROS_INFO_STREAM("SCAN ACCESS POINTS");
        }
        else if(msUi_.playsComboBox->currentIndex() == 1){
            quadrotors[cur_uav].EnableOffboard();
            quadrotors[cur_uav].ScoutBuilding(0,0,1.5);
            ROS_INFO_STREAM("SCOUT BUILDING");
        }
    }
    else if(msUi_.missionComboBox->currentIndex() == 4){
        quadrotors[cur_uav].SetMode("AUTO.LOITER");
    }
    else if(msUi_.missionComboBox->currentIndex() == 5){
        quadrotors[cur_uav].SetMode("AUTO.RTL");
    }

    missionSelectWidget_->close();
    missionConfirmWidget_->close();
}

void SimpleGCS::MissionConfirm(){

    missionConfirmWidget_->show();
}

void SimpleGCS::MissionConfirmCancel(){
    missionConfirmWidget_->close();
    missionSelectWidget_->close();
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
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
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
