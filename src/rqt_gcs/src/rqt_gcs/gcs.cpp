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
  setObjectName("MyPlugin");
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
  missionProgressWidget2_ = new QWidget();
  missionProgressWidget3_ = new QWidget();
  missionProgressWidget4_ = new QWidget();
  UavQuestionWidget_ = new QWidget();
  UavStatWidget1_ = new QWidget();
  UavStatWidget2_ = new QWidget();
 // UavStatWidget3_ = new QWidget();
  //UavStatWidget4_ = new QWidget();


  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  mcUi_.setupUi(missionCancelWidget_);
  mpUi_.setupUi(missionProgressWidget1_);
  mpUi_.setupUi(missionProgressWidget2_);
  mpUi_.setupUi(missionProgressWidget3_);
  mpUi_.setupUi(missionProgressWidget4_);
  msUi_.setupUi(missionSelectWidget_);
  uqUi_.setupUi(UavQuestionWidget_);
  usUi_.setupUi(UavStatWidget1_);
  usUi_.setupUi(UavStatWidget2_);
  //usUi_.setupUi(UavStatWidget3_);
  //usUi_.setupUi(UavStatWidget4_);


  //label = new QLabel("Blah",widget_);

  // add widget to the user interface
  context.addWidget(widget_);
  ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget1_);
  ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget2_);
  ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget3_);
  ui_.MissionProgressIndividualLayout->addWidget(missionProgressWidget4_);
  ui_.UavStatLayout->addWidget(UavStatWidget1_);
  ui_.UavStatLayout->addWidget(UavStatWidget2_);
  //ui_.UavStatLayout->addWidget(UavStatWidget3_);
 // ui_.UavStatLayout->addWidget(UavStatWidget4_);


 // QObject::connect(ui_.CalculateButton,SIGNAL(clicked()),this,SLOT(Execute()));

}

void MyPlugin::Calculate(){

	//op = ui_.operand1->text();
  	//op2 = ui_.operand2->text();
	//ui_.sum->setText(op.append(op2));
	//ROS_INFO_STREAM("subscribed");
}


void MyPlugin::Execute(){

     // quadControl.Takeoff(10);

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
