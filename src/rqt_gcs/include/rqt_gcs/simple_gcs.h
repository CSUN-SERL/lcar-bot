#ifndef rqt_gcs__SimpleGCS_H
#define rqt_gcs__SimpleGCS_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_gcs/simple_control.h>

#include <image_transport/image_transport.h>
#include "opencv2/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

#include <ui_gcs.h>
#include <ui_MissionCancel.h>
#include <ui_MissionProgress.h>
#include <ui_MissionSelect.h>
#include <ui_UavQuestion.h>
#include <ui_UavStat.h>
#include <ui_QuadStats.h>
#include <ui_UAVCondition.h>
#include <ui_WidgetMain.h>
#include <ui_PFDWidget_custom.h>
#include <ui_ImageView.h>

#include <QWidget>
#include <QLabel>
#include <QString>
#include <QTimer>
#include <QMainWindow>
#include <QSignalMapper>

#define NUM_UAV 2 //Total number of UAV's in the system

namespace rqt_gcs{

  class SimpleGCS: public rqt_gui_cpp::Plugin
  {
  Q_OBJECT
  public:
    SimpleGCS();

    ros::Subscriber sub;
    ros::NodeHandle nh;
    image_transport::ImageTransport it_stereo{nh};
    void GetMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

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
    virtual void QuadSelect(int);

  private:
    void UpdatePFD();

    SimpleControl quadrotors[NUM_UAV] =  {SimpleControl{1}, SimpleControl{2}};

    cv::Mat conversion_mat_;
    image_transport::Subscriber sub_stereo = it_stereo.subscribe("stereo_cam/left/image_raw", 1, &SimpleGCS::ImageCallback, this);

    Ui::SimpleGCSWidget ui_;
    Ui::QuadStatsWidget quadUi1_;
    Ui::MissionCancelWidget mcUi_;
    Ui::MissionProgressWidget mpUi1_;
    Ui::MissionProgressWidget mpUi2_;
    Ui::MissionSelectWidget msUi_;
    Ui::UavQuestionWidget uqUi_;
    Ui::UavStatWidget usUi1_;
    Ui::ImageViewWidget ivUi_;
    Ui::PFDWidget pfd_ui;
    Ui::centralWidget central_ui_;
    Ui::UAVConditionWidget  condUi1_;
    Ui::UAVConditionWidget  condUi2_;

    QWidget* widget_;
    QWidget* missionCancelWidget1_;
    QWidget* missionSelectWidget1_;
    QWidget* missionProgressWidget1_;
    QWidget* missionProgressWidget2_;
    QWidget* UavQuestionWidget1_;
    QWidget* UavStatWidget1_;
    QWidget* ImageViewWidget_;
    QWidget* UavConditionWidget1_;
    QWidget* UavConditionWidget2_;
    QWidget* PFDQWidget;

    QLabel* label;
    QTimer* updateTimer;

    int cur_uav = 0;

    QString temp_data;
    QString quadId;
    QSignalMapper* signalMapper;

  };
} // namespace
#endif // my_namespace__my_plugin_H
