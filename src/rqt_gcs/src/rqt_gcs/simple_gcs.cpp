#include <rqt_gcs/simple_gcs.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <iomanip>
namespace rqt_gcs {

SimpleGCS::SimpleGCS()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{

  // Constructor is called first before initPlugin function, needless to say.
  // give QObjects reasonable names
  setObjectName("LCAR Bot GCS");
}

void SimpleGCS::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create QWidget
  widget_ = new QWidget();

  missionCancelWidget1_   = new QWidget();
  missionSelectWidget1_   = new QWidget();
  missionProgressWidget1_ = new QWidget();
  UavQuestionWidget1_     = new QWidget();
  UavStatWidget1_         = new QWidget();
  ImageViewWidget_        = new QWidget();
  PFDQWidget              = new QWidget();

  UavConditionWidget1_    = new QWidget();
  UavConditionWidget2_    = new QWidget();

  central_ui_.setupUi(widget_);
  mcUi_.setupUi(missionCancelWidget1_);
  mpUi1_.setupUi(missionProgressWidget1_);
  msUi_.setupUi(missionSelectWidget1_);
  uqUi_.setupUi(UavQuestionWidget1_);
  usUi1_.setupUi(UavStatWidget1_);
  ivUi_.setupUi(ImageViewWidget_);
  pfd_ui.setupUi(PFDQWidget);

  condUi1_.setupUi(UavConditionWidget1_);
  condUi2_.setupUi(UavConditionWidget2_);

  // add widget to the user interface
  context.addWidget(widget_);

  central_ui_.MissionLayout->addWidget(missionProgressWidget1_);
  central_ui_.OverviewLayout->addWidget(UavStatWidget1_);
  central_ui_.PFDLayout->addWidget(PFDQWidget);
  central_ui_.CameraLayout->addWidget(ImageViewWidget_);
  central_ui_.UAVListLayout->addWidget(UavConditionWidget1_);
  central_ui_.UAVListLayout->addWidget(UavConditionWidget2_);


   //setup mission progress widgets
   missionSelectWidget1_->setWindowTitle("Mission Selection");
   UavStatWidget1_->setWindowTitle("Flight State");
   missionProgressWidget1_->setWindowTitle("Mission Control");
   connect(mpUi1_.changeMissionButton,SIGNAL(clicked()),this, SLOT(MissionChange()));

   //setup Mission select widgets
   connect(msUi_.submitMission,SIGNAL(clicked()),this, SLOT(MissionSubmit()));
   connect(msUi_.missionComboBox,SIGNAL(activated(int)),this, SLOT(MissionSelect(int)));

   updateTimer = new QTimer(this);
   connect(updateTimer, SIGNAL(timeout()), this, SLOT(TimedUpdate()));
   updateTimer->start(100);
}

void SimpleGCS::Calculate(){

}


void SimpleGCS::TimedUpdate(){

  QString temp_data;
  SimpleControl quad = quadrotors[cur_uav];

  temp_data = quad.GetState().mode.c_str();
  usUi1_.flightModeDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().yaw,'f',2);
  usUi1_.yawDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().roll,'f',2);
  usUi1_.rollDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().pitch,'f',2);
  usUi1_.pitchDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().altitude,'f',2);
  usUi1_.altitudeDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().vertical_speed,'f',2);
  usUi1_.verticalSpaceDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().ground_speed,'f',2);
  usUi1_.horizontalSpaceDisplay->setText(temp_data);

  temp_data.setNum(quad.GetFlightState().heading,'f',2);
  usUi1_.headingDisplay->setText(temp_data);

  temp_data.setNum(quad.GetDistanceToWP());
  usUi1_.waypointDisplay->setText(temp_data);

  temp_data.setNum(quad.GetBatteryStatus().remaining*100);
  usUi1_.batteryProgressBar->setValue(temp_data.toInt());

  temp_data = "Quadrotor 1";
  mpUi1_.uavNameEdit->setText(temp_data);

  mpUi1_.missionProgressBar->setValue(quad.GetMissionProgress()*100);

  this->UpdatePFD();

  quad.Run();
}

void SimpleGCS::MissionChange(){

    missionSelectWidget1_->show();
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
        ROS_INFO_STREAM("LAND");
   }
   else if(msUi_.missionComboBox->currentIndex() == 3){
        ROS_INFO_STREAM("FOLLOW PLAY");

         if(msUi_.playsComboBox->currentIndex() == 0){
          ROS_INFO_STREAM("SCAN ACCESS POINTS");
         }
         else if(msUi_.playsComboBox->currentIndex() == 1){
           quadrotors[cur_uav].EnableOffboard();
  	        quadrotors[cur_uav].ScoutBuilding(-7,-9,3);
           ROS_INFO_STREAM("SCOUT BUILDING");
         }
   }
   else if(msUi_.missionComboBox->currentIndex() == 4){
           ROS_INFO_STREAM("LOITER");
   }
   else if(msUi_.missionComboBox->currentIndex() == 5){
           ROS_INFO_STREAM("RETURN HOME");
   }

    missionSelectWidget1_->close();
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
  //pfd_ui->widgetPFD->setPressure (quadrotors[cur_uav].GetFlightState().roll);
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
