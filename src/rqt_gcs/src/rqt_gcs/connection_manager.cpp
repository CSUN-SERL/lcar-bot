#include <rqt_gcs/connection_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "connection_manager");
  ros::Rate loop_rate(10); //10Hz

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

}

ConnectionManager::ConnectionManager()  //Class constructor
{
}
