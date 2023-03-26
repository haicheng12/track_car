#ifndef RECORD_PATH_H_
#define RECORD_PATH_H_

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Int32.h"
#include <nav_msgs/Path.h>
#include <fstream>
#include <vector>
#include "math.h"
#include <string>

#include "agv_robot/path.h"

struct Points //读取全局路径
{  
  double x;
  double y;
  double yaw;
};

namespace RecordPathNS
{
  class RecordPath
  {
    protected:
      ros::NodeHandle nh_;

      ros::Subscriber odom_sub_;
      ros::Subscriber current_pose_sub_;//接收当前位置

      ros::Publisher global_path_pub_;
      ros::Publisher global_num_pub_;

      ros::ServiceServer path_service_;
      ros::ServiceServer global_service_;

      void initial();//初始化
      void odomCallback(const nav_msgs::OdometryConstPtr& msg);//里程计
      void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);//回调当前位置
      void pubGlobalPath();
      void pubGlobalNum(int index);
      bool pathServer(agv_robot::path::Request &req,
                                         agv_robot::path::Response &res);
      bool globalServer(agv_robot::path::Request &req,
                                            agv_robot::path::Response &res);

    public:
      RecordPath();
      ~RecordPath();
      
      void MainLoop();

    private:
      std::ofstream outFile_;//保存文件

      std::vector<Points> vec_points_; //路径点存储

      double current_pose_x_;//当前位置
      double current_pose_y_;
      double current_pose_yaw_;
      double pre_pose_x_;//上一时刻位置
      double pre_pose_y_;
      double pre_pose_yaw_;

      int path_sum_;
      int path_index_;

      int global_sum_;
      int global_path_size_;

      bool is_pose_sub_;
      bool is_path_server_;
      bool receive_once_;
      bool is_record_path_;

      //param参数加载
      double interval_;
      bool is_use_sim_;

  };
}

#endif



