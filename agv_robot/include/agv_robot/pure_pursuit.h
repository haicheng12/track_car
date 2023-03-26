#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "agv_robot/robot_car_info.h" //小车状态
#include "agv_robot/path.h"

enum FollowState //跟线状态机
{
  RUN = 1, //正常行驶
  ARRIVE_GOAL ,        //到达终点
  STOP //暂停行使
};

namespace PurePursuitNS
{ 
  class PurePursuit 
  {
    protected:
      //ROS messages (topics)
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      ros::Subscriber global_path_sub_;
      ros::Subscriber global_num_sub_;
      ros::Subscriber odom_sub_;
      ros::Subscriber current_pose_sub_;//接收当前位置
      ros::Subscriber set_speed_sub_;
      ros::Subscriber car_stop_sub_;

      ros::Subscriber scan_sub_;
      ros::Subscriber scan_2D_sub_;

      ros::Publisher robot_car_info_pub_;
      ros::Publisher cmd_vel_pub_;

      void initial();
      void pubRobotCarInfo();//发布小车状态信息
      void globalPathCallback(const nav_msgs::Path &msg);
      void globalNumCallback(const std_msgs::Int32 &msg);
      void setSpeed(const std_msgs::Float64 &msg);//设定速度
      void carStop(const std_msgs::Bool &msg);
      void odomCallback(const nav_msgs::Odometry &msg);
      void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);//回调当前位置
      void scanCallback(const sensor_msgs::LaserScanConstPtr &msg); //雷达回调
      void scan2DCallback(const sensor_msgs::LaserScanConstPtr &msg); //雷达回调

      inline double angleLimit(double yaw);//角度限制
      double Xianfu(double value, double Amplitude);//限制最大速度幅度
      double Position_PID(double Position, double target);
      FollowState calculateTrack();

    public:
      PurePursuit();
      ~PurePursuit();
      
      void MainLoop();

    private:
      geometry_msgs::PoseStamped current_pose_msg_;
      geometry_msgs::TwistStamped current_vel_msg_;
      
      FollowState follow_state_;

      std::vector<double> r_x_;
      std::vector<double> r_y_;

      int pointNum ;  //保存路径点的个数
      int index_;

      double curvature_k_;

      bool is_sub_odom_;
      bool is_sub_path_;

      double Position_KP;//位置式PID控制器参数设定
      double Position_KI;
      double Position_KD;

      int global_num_;
      double current_vel_x_;

      double smallest_distance_;//定义了一个接收激光雷达扫描的距离
      double smallest_2D_distance_;
      bool is_pub_speed_; 
      bool is_pub_2d_speed_;

      bool is_stop_car_;

      //参数加载
      bool is_use_sim_;
      double car_velocity_;
      double point_distance_;
      double scan_distance_3D_;
      double scan_distance_2D_;
  };
}

#endif
