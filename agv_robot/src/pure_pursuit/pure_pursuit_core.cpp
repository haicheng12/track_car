#include "agv_robot/pure_pursuit.h"

namespace PurePursuitNS
{
  PurePursuit::PurePursuit()
  {
    nh_.getParam("car_velocity", car_velocity_);
    nh_.getParam("point_distance", point_distance_);
    nh_.getParam("scan_distance_3D", scan_distance_3D_);
    nh_.getParam("scan_distance_2D", scan_distance_2D_);
    nh_.getParam("is_use_sim", is_use_sim_);

    initial(); // 初始化

    global_path_sub_ = nh_.subscribe("/global_path", 1, &PurePursuit::globalPathCallback, this);
    global_num_sub_ = nh_.subscribe("/global_num", 1, &PurePursuit::globalNumCallback, this);
    set_speed_sub_ = nh_.subscribe("/set_speed", 1, &PurePursuit::setSpeed, this); // 设定机器人速度
    car_stop_sub_ = nh_.subscribe("/car_stop", 1, &PurePursuit::carStop, this);    // 是否暂停小车

    if (is_use_sim_) // 使用仿真
    {
      odom_sub_ = nh_.subscribe("/odom", 10, &PurePursuit::odomCallback, this); // 里程计
    }
    else // 雷达定位
    {
      current_pose_sub_ = nh_.subscribe("/ndt_pose", 10, &PurePursuit::currentPoseCallback, this); // 回调当前位置
      scan_sub_ = nh_.subscribe("/scan", 20, &PurePursuit::scanCallback, this);                    // 雷达回调
      scan_2D_sub_ = nh_.subscribe("/scan_2D", 20, &PurePursuit::scan2DCallback, this);            // 雷达回调
    }

    robot_car_info_pub_ = nh_.advertise<agv_robot::robot_car_info>("/robot_car_info_msg", 1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  PurePursuit::~PurePursuit()
  {
  }

  void PurePursuit::initial()
  {
    r_x_.clear();
    r_y_.clear();

    pointNum = 0; // 保存路径点的个数
    index_ = 1;
    global_num_ = 0;

    is_sub_odom_ = false;
    is_sub_path_ = false;

    Position_KP = 0.5; // 位置式PID控制器参数设定
    Position_KI = 0.0;
    Position_KD = 0.5;

    current_vel_x_ = car_velocity_;

    smallest_distance_ = 10.0; // 定义了一个接收激光雷达扫描的距离
    smallest_2D_distance_ = 10.0;
    is_pub_speed_ = true;
    is_pub_2d_speed_ = true;

    is_stop_car_ = false;
  }

  void PurePursuit::odomCallback(const nav_msgs::Odometry &msg)
  {
    current_pose_msg_.pose = msg.pose.pose;
    current_vel_msg_.twist = msg.twist.twist;

    is_sub_odom_ = true;
  }

  void PurePursuit::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) // 回调当前位置
  {
    current_pose_msg_.pose = msg->pose;

    is_sub_odom_ = true;
  }

  void PurePursuit::scanCallback(const sensor_msgs::LaserScanConstPtr &msg)
  {
    int arr_size = floor((msg->angle_max - msg->angle_min) / msg->angle_increment);
    for (int i = 900; i < 1240; i++) // 滤波 90
    {
      if (msg->ranges[i] < 0.1) // 滤波
      {
        continue;
      }
      if (msg->ranges[i] < smallest_distance_)
      {
        smallest_distance_ = msg->ranges[i];
      }
    }

    if (smallest_distance_ <= scan_distance_3D_) // 雷达避障的距离
    {
      is_pub_speed_ = false;
    }
    else
    {
      is_pub_speed_ = true;
    }
    smallest_distance_ = 10.0;
  }

  void PurePursuit::scan2DCallback(const sensor_msgs::LaserScan::ConstPtr &scan) // 雷达回调
  {
    int arr_size = floor((scan->angle_max - scan->angle_min) / scan->angle_increment);
    for (int i = 1200; i < 1400; i++) // right
    {
      if (scan->ranges[i] < smallest_2D_distance_)
      {
        smallest_2D_distance_ = scan->ranges[i];
      }
    }
    for (int i = 0; i < 200; i++) // left
    {
      if (scan->ranges[i] < smallest_2D_distance_)
      {
        smallest_2D_distance_ = scan->ranges[i];
      }
    }

    if (smallest_2D_distance_ <= scan_distance_2D_) // 雷达避障的距离#1.0
    {
      is_pub_2d_speed_ = false;
    }
    else
    {
      is_pub_2d_speed_ = true;
    }
    smallest_2D_distance_ = 10.0;
  }

  void PurePursuit::setSpeed(const std_msgs::Float64 &msg) // 设定速度
  {
    current_vel_x_ = msg.data;
  }

  void PurePursuit::carStop(const std_msgs::Bool &msg)
  {
    is_stop_car_ = msg.data;
  }

  void PurePursuit::globalPathCallback(const nav_msgs::Path &msg)
  {
    initial();

    pointNum = msg.poses.size();
    std::cout << "pointNum " << pointNum << std::endl;

    r_x_.clear();
    r_y_.clear();
    for (int i = 0; i < pointNum; i++)
    {
      r_x_.push_back(msg.poses[i].pose.position.x);
      r_y_.push_back(msg.poses[i].pose.position.y);
    }

    for (int i = 0; i < pointNum; i++)
    {
      double dis = sqrt(pow(r_x_[i] - current_pose_msg_.pose.position.x, 2) +
                        pow(r_y_[i] - current_pose_msg_.pose.position.y, 2));
      std::cout << "i " << i << " r_x_ " << r_x_[i] << " r_y_ " << r_y_[i] << " dis " << dis << std::endl;
    }

    is_sub_path_ = true;
  }

  void PurePursuit::globalNumCallback(const std_msgs::Int32 &msg)
  {
    global_num_ = msg.data;
  }

  inline double PurePursuit::angleLimit(double yaw) // 角度限制
  {
    double theta = yaw;
    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }
    return theta;
  }

  double PurePursuit::Xianfu(double value, double Amplitude) // 限制最大速度幅度
  {
    double temp;
    if (value > Amplitude)
      temp = Amplitude;
    else if (value < -Amplitude)
      temp = -Amplitude;
    else
      temp = value;

    return temp;
  }

  // 函数功能：位置式PID控制器
  // 入口参数：当前位置，目标位置
  // 返回  值：控制的速度
  // 根据位置式离散PID公式
  // Speed=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
  // e(k)代表本次偏差
  // e(k-1)代表上一次的偏差
  // ∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
  // Speed代表输出
  double PurePursuit::Position_PID(double Position, double target)
  {
    static double Bias, Speed, Integral_bias, Last_Bias;
    Bias = target - Position; // 计算偏差
    Integral_bias += Bias;    // 求出偏差的积分
    Integral_bias = Xianfu(Integral_bias, 0.5);
    Speed = Position_KP * Bias + Position_KI * Integral_bias + Position_KD * (Bias - Last_Bias); // 位置式PID控制器
    Last_Bias = Bias;                                                                            // 保存上一次偏差

    if (Speed > 0.5)
      Speed = 0.5;
    if (Speed < -0.5)
      Speed = -0.5;

    return Speed; // 增量输出
  }

  FollowState PurePursuit::calculateTrack()
  {
    double calRPY = tf::getYaw(current_pose_msg_.pose.orientation);
    // std::cout << "calRPY " << calRPY << std::endl;

    std::cout << "index_ " << index_ << std::endl;

    double alpha = angleLimit(atan2(r_y_[index_] - current_pose_msg_.pose.position.y, r_x_[index_] - current_pose_msg_.pose.position.x) - calRPY);
    std::cout << "alpha " << alpha << std::endl;

    // 当前点和目标点的距离dl
    double dl = sqrt(pow(r_y_[index_] - current_pose_msg_.pose.position.y, 2) +
                     pow(r_x_[index_] - current_pose_msg_.pose.position.x, 2));
    std::cout << "dl " << dl << std::endl;

    double dis = sqrt(pow(r_y_.back() - current_pose_msg_.pose.position.y, 2) +
                      pow(r_x_.back() - current_pose_msg_.pose.position.x, 2));
    std::cout << "dis " << dis << std::endl;

    curvature_k_ = 2 * sin(alpha) / dl; // 跟踪曲率 k = 2 * sin(a) / Ld
    // std::cout << "curvature_k_ " << curvature_k_ << std::endl;

    if (is_stop_car_)
    {
      return FollowState::STOP;
    }
    else
    {
      if (dl > point_distance_ && index_ < pointNum) // 行走过程中
      {
        return FollowState::RUN;
      }
      else if (dl <= point_distance_ && index_ < pointNum) // 更新下一个坐标点
      {
        ++index_;
        return FollowState::RUN;
      }
      else if (dis <= point_distance_ && index_ >= pointNum - 1) // 到达终点
      {
        return FollowState::ARRIVE_GOAL;
      }
      else
      {
        ROS_WARN("NOTHING!");
        return FollowState::RUN;
      }
    }
    return FollowState::RUN;
  }

  void PurePursuit::pubRobotCarInfo() // 发布小车状态信息
  {
    agv_robot::robot_car_info robot_car_info_msg;
    robot_car_info_msg.state = follow_state_;
    robot_car_info_msg.path_num = global_num_;
    if (follow_state_ == FollowState::ARRIVE_GOAL)
    {
      robot_car_info_msg.is_arrive_goal = true;
    }
    else
    {
      robot_car_info_msg.is_arrive_goal = false;
    }
    robot_car_info_msg.speed = current_vel_msg_.twist.linear.x;

    robot_car_info_pub_.publish(robot_car_info_msg);
  }

  void PurePursuit::MainLoop()
  {
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
      pubRobotCarInfo(); // 发布小车状态信息

      if (is_sub_odom_ && is_sub_path_)
      {
        follow_state_ = calculateTrack();

        switch (follow_state_)
        {
        case RUN: // 再行走的过程中添加旋转控制
        {
          ROS_INFO("RUN");
          double remaining_dis = sqrt(pow(r_y_.back() - current_pose_msg_.pose.position.y, 2) +
                                      pow(r_x_.back() - current_pose_msg_.pose.position.x, 2));
          double position_motor = Position_PID(-remaining_dis, 0.0);
          double theta = current_vel_x_ * curvature_k_;

          if (is_pub_speed_ && is_pub_2d_speed_)
          {
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = Xianfu(position_motor, current_vel_x_);
            vel_msg.angular.z = Xianfu(theta, 1.0);
            cmd_vel_pub_.publish(vel_msg);
            ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);
          }
          else
          {
            ROS_WARN("SCAN STOP");
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = Xianfu(theta, 1.0);
            cmd_vel_pub_.publish(vel_msg);
          }
          break;
        }
        case ARRIVE_GOAL:
        {
          ROS_INFO("ARRIVE_GOAL");
          geometry_msgs::Twist vel_msg;
          vel_msg.linear.x = 0.0;
          vel_msg.angular.z = 0.0;
          cmd_vel_pub_.publish(vel_msg);

          is_sub_path_ = false;
          break;
        }
        case STOP:
        {
          ROS_INFO("STOP");
          geometry_msgs::Twist vel_msg;
          vel_msg.linear.x = 0.0;
          vel_msg.angular.z = 0.0;
          cmd_vel_pub_.publish(vel_msg);
          // ROS_INFO("cmd_vel [%f] [%f]", vel_msg.linear.x, vel_msg.angular.z);
        }
        default:
          break;
        }
        is_sub_odom_ = false;
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}