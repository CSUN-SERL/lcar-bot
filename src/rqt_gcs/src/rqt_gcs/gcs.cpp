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
  missionCancelWidget_ = new QWidget();
  missionSelectWidget_ = new QWidget();
  missionProgressWidget1_ = new QWidget();
 // missionProgressWidget2_ = new QWidget();
 // missionProgressWidget3_ = new QWidget();
 // missionProgressWidget4_ = new QWidget();
  UavQuestionWidget_ = new QWidget();
  UavStatWidget1_ = new QWidget();
 //UavStatWidget2_ = new QWidget();


  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  mcUi_.setupUi(missionCancelWidget_);

  mpUi1_.setupUi(missionProgressWidget1_);
 //mpUi2_.setupUi(missionProgressWidget2_);


  msUi_.setupUi(missionSelectWidget_);
  uqUi_.setupUi(UavQuestionWidget_);

  usUi1_.setupUi(UavStatWidget1_);
  //usUi2_.setupUi(UavStatWidget2_);



  // add widget to the user interface
  context.addWidget(widget_);
  ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget1_);
  //ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget2_);
  //ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget3_);
  //ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget4_);
  ui_.UavStatLayout->addWidget(UavStatWidget1_);
  //ui_.UavStatLayout->addWidget(UavStatWidget2_);


   //setup mission progress widgets
   tempData = "QuadRotor 1" ;
   missionSelectWidget_->setWindowTitle(tempData);
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

    tempData.setNum(quad1.GetFlightState().yaw);
    usUi1_.yawDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().roll);
    usUi1_.rollDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().pitch);
    usUi1_.pitchDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().altitude);
    usUi1_.altitudeDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().vertical_speed);
    usUi1_.verticalSpaceDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().ground_speed);
    usUi1_.horizontalSpaceDisplay->setText(tempData);

    tempData.setNum(quad1.GetFlightState().heading);
    usUi1_.headingDisplay->setText(tempData);

    tempData.setNum(quad1.GetDistanceToWP());
    usUi1_.waypointDisplay->setText(tempData);

    tempData.setNum(quad1.GetBatteryStatus().remaining*100);
    usUi1_.batteryProgressBar->setValue(tempData.toInt());

    tempData = "Quadrotor 1";
    mpUi1_.uavNameEdit->setText(tempData);

    tempData.setNum(quad1.GetMissionProgress()*100);
    mpUi1_.missionProgressBar->setValue(tempData.toInt());

    quad1.Run();
}

void MyPlugin::MissionChange(){

    missionSelectWidget_->show();
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
        ROS_INFO_STREAM("ARMED");
   }
   else if(msUi_.missionComboBox->currentIndex() == 1){
        ROS_INFO_STREAM("DISARMED");
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
  	 quad1.ScoutBuilding(16,16,5);
           ROS_INFO_STREAM("SCOUT BUILDING");
         }
   }
   else if(msUi_.missionComboBox->currentIndex() == 4){
           ROS_INFO_STREAM("LOITER");
   }
   else if(msUi_.missionComboBox->currentIndex() == 5){
           ROS_INFO_STREAM("RETURN HOME");
   }

    missionSelectWidget_->close();
}

void MyPlugin::shutdownPlugin()
{
  // TODO unregister all publishers here =

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

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_gcs, MyPlugin, rqt_gcs::MyPlugin, rqt_gui_cpp::Plugin)
//PLUGINLIB_EXPORT_CLASS(rqt_gcs::MyPlugin, rqt_gui_cpp::Plugin)
