#include "agv_robot/record_path.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_path");

  RecordPathNS::RecordPath record_path;
  record_path.MainLoop();

  return 0;
}

