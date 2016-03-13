#include <rqt_gcs/connection_manager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "connection_manager");
  ros::Rate loop_rate(10); //10Hz

  ConnectionManager connection;

  while(ros::ok())
  {
    connection.PublishConnection();

    ros::spinOnce();
    loop_rate.sleep();
  }

}

int ConnectionManager::id = 0;

ConnectionManager::ConnectionManager()  //Class constructor
{
  ++id;
  std::string ns = DEF_NS + std::to_string(id); //UAV Namespace
  pub_connection = nh.advertise<std_msgs::Int8>(ns + "/connection_status",QUEUE_SIZE);
}

ConnectionManager::~ConnectionManager()
{
    //Default destructor
}

void ConnectionManager::PublishConnection()
{
  std_msgs::Int8 connection_status;
  connection_status.data = 1;

  pub_connection.publish(connection_status);
}
