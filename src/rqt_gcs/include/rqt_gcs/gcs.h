#ifndef rqt_gcs__MyPlugin_H
#define rqt_gcs__MyPlugin_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>

#include <rqt_gcs/simple_control.h>

#include <ui_gcs.h>
#include <ui_MissionCancel.h>
#include <ui_MissionProgress.h>
#include <ui_MissionSelect.h>
#include <ui_UavQuestion.h>
#include <ui_UavStat.h>
#include <ui_QuadStats.h>


#include <QWidget>
#include <QLabel>
#include <QString>
#include <QTimer>

namespace rqt_gcs{

class MyPlugin
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  MyPlugin();
  SimpleControl quad1;
  QString tempData;

  ros::Subscriber sub;
  ros::NodeHandle nh;
  void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected slots:
  virtual void Calculate();
  virtual void TimedUpdate();
  virtual void MissionChange();
  virtual void MissionSelect(const int);
  virtual void MissionSubmit();


  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::MyPluginWidget ui_;
  Ui::QuadStatsWidget quadUi1_;
 // Ui::QuadStatsWidget quadUi2_;
  //Ui::QuadStatsWidget quadUi3_;
 // Ui::QuadStatsWidget quadUi4_;

  Ui::MissionCancelWidget mcUi_;
  Ui::MissionProgressWidget mpUi1_;
 // Ui::MissionProgressWidget mpUi2_;
 // Ui::MissionProgressWidget mpUi3_;
 // Ui::MissionProgressWidget mpUi4_;
  Ui::MissionSelectWidget msUi_;
  Ui::UavQuestionWidget uqUi_;
  Ui::UavStatWidget usUi1_;
 // Ui::UavStatWidget usUi2_;


  QWidget* widget_;
  QWidget* missionCancelWidget1_;
  QWidget* missionSelectWidget1_;
  QWidget* missionProgressWidget1_;
 // QWidget* missionProgressWidget2_;
 // QWidget* missionProgressWidget3_;
 // QWidget* missionProgressWidget4_;
  QWidget* UavQuestionWidget1_;
  QWidget* UavStatWidget1_;
  //QWidget* UavStatWidget2_;
 // QWidget* UavStatWidget3_;
 // QWidget* UavStatWidget4_;

 // QWidget* quadWidget1Mission_;
  //QWidget* quadWidget1Stats_;

 // QWidget* quadWidget2_;
  //QWidget* quadWidget3_;
 // QWidget* quadWidget4_;

  QLabel* label;
  QTimer* updateTimer;

  int cur_uav = 1;

};
} // namespace
#endif // my_namespace__my_plugin_H
