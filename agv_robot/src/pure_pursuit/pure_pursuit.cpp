#include "agv_robot/pure_pursuit.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pure_pursuit");

  PurePursuitNS::PurePursuit pure_pursuit;
  pure_pursuit.MainLoop();

  return 0;
}
