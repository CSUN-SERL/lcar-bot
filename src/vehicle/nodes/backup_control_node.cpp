#include "vehicle/backup_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "backup_control");
  BackupControl bc;

  ros::spin();

  return 0;
}