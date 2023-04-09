#include "ros/ros.h"
#include "agv_robot/path.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "glabal_client");
  if (argc != 2)
  {
    ROS_INFO("usage: glabal_num X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<agv_robot::path>("set_path_num");

  agv_robot::path srv;
  srv.request.num = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("glabal sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call glabal_num");
    return 1;
  }

  return 0;
}
