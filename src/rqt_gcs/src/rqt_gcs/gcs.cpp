#include <rqt_gcs/gcs.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <iomanip>
namespace rqt_gcs {

MyPlugin::MyPlugin()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{

  // Constructor is called first before initPlugin function, needless to say.
  // give QObjects reasonable names
  setObjectName("LCAR Bot GCS");
}

void MyPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create QWidget
  widget_ = new QWidget();

  missionCancelWidget1_ = new QWidget();

  missionSelectWidget1_ = new QWidget();

  missionProgressWidget1_ = new QWidget();

  UavQuestionWidget1_ = new QWidget();

  UavStatWidget1_ = new QWidget();

  PFDQWidget       = new QWidget();

  UavConditionWidget1_ = new QWidget();
  UavConditionWidget2_ = new QWidget();

  central_ui_.setupUi(widget_);

  mcUi_.setupUi(missionCancelWidget1_);

  mpUi1_.setupUi(missionProgressWidget1_);

  msUi_.setupUi(missionSelectWidget1_);

  uqUi_.setupUi(UavQuestionWidget1_);

  usUi1_.setupUi(UavStatWidget1_);

  pfd_ui.setupUi(PFDQWidget);

  condUi1_.setupUi(UavConditionWidget1_);
  condUi2_.setupUi(UavConditionWidget2_);

  //central_ui_.verticalLayout->removeWidget(central_ui_.WidgetOverview);
  //central_ui_.verticalLayout->addWidget(UavStatWidget1_);


  // add widget to the user interface
  context.addWidget(widget_);
 
  central_ui_.MissionLayout->addWidget(missionProgressWidget1_);
  central_ui_.OverviewLayout->addWidget(UavStatWidget1_);
  central_ui_.PFDLayout->addWidget(PFDQWidget);
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

void MyPlugin::Calculate(){

}


void MyPlugin::TimedUpdate(){

    tempData = quad1.GetState().mode.c_str();
    usUi1_.flightModeDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().yaw,'f',2);
    usUi1_.yawDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().roll,'f',2);
    usUi1_.rollDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().pitch,'f',2);
    usUi1_.pitchDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().altitude,'f',2);
    usUi1_.altitudeDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().vertical_speed,'f',2);
    usUi1_.verticalSpaceDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().ground_speed,'f',2);
    usUi1_.horizontalSpaceDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().heading,'f',2);
    usUi1_.headingDisplay->setText(tempData);

    tempData.setNum(quad1.GetDistanceToWP());
    usUi1_.waypointDisplay->setText(tempData);

    tempData.setNum(quad1.GetBatteryStatus().remaining*100);
    usUi1_.batteryProgressBar->setValue(tempData.toInt());

    tempData = "Quadrotor 1";
    mpUi1_.uavNameEdit->setText(tempData);

    mpUi1_.missionProgressBar->setValue(quad1.GetMissionProgress()*100);

    this->UpdatePFD();

    quad1.Run();
}

void MyPlugin::MissionChange(){

    missionSelectWidget1_->show();
}

void MyPlugin::MissionSelect(const int i){

    if(i == 3){
     msUi_.playsComboBox->setEnabled(true);

    }
    else{
     msUi_.playsComboBox->setEnabled(false);
    }
}

void MyPlugin::MissionSubmit(){
   ROS_INFO_STREAM("Mission Submitted");
   if(msUi_.missionComboBox->currentIndex() == 0){
        quad1.Arm(true);
   }
   else if(msUi_.missionComboBox->currentIndex() == 1){
        quad1.Arm(false);
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
           quad1.EnableOffboard();
  	       quad1.ScoutBuilding(0,6,3);
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

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void MyPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void MyPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
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

void MyPlugin::UpdatePFD()
{
  pfd_ui.widgetPFD->setRoll       ((quad1.GetFlightState().roll)*180);
  pfd_ui.widgetPFD->setPitch      ((quad1.GetFlightState().pitch)*90);
  pfd_ui.widgetPFD->setHeading    (quad1.GetFlightState().heading);
  pfd_ui.widgetPFD->setAirspeed   (quad1.GetFlightState().ground_speed);
  pfd_ui.widgetPFD->setAltitude   (quad1.GetFlightState().altitude);
  //pfd_ui->widgetPFD->setPressure (quad1.GetFlightState().roll);
  pfd_ui.widgetPFD->setClimbRate  (quad1.GetFlightState().vertical_speed);

  pfd_ui.widgetPFD->update();
}

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, MyPlugin, rqt_gcs::MyPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_EXPORT_CLASS(rqt_gcs::MyPlugin, rqt_gui_cpp::Plugin)
