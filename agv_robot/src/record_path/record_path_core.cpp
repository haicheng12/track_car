#include "agv_robot/record_path.h"

namespace RecordPathNS
{
    RecordPath::RecordPath()
    {
        nh_.getParam("is_use_sim", is_use_sim_);
        nh_.getParam("interval", interval_);

        initial(); // 初始化

        if (is_use_sim_) // 使用仿真
        {
            odom_sub_ = nh_.subscribe("/odom", 10, &RecordPath::odomCallback, this); // 里程计
        }
        else // 雷达定位
        {
            current_pose_sub_ = nh_.subscribe("/ndt_pose", 10, &RecordPath::currentPoseCallback, this); // 回调当前位置
        }

        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
        global_num_pub_ = nh_.advertise<std_msgs::Int32>("/global_num", 1);

        path_service_ = nh_.advertiseService("record_path_num", &RecordPath::pathServer, this);
        global_service_ = nh_.advertiseService("set_path_num", &RecordPath::globalServer, this);
    }

    RecordPath::~RecordPath()
    {
        outFile_.close();
    }

    void RecordPath::initial() // 初始化
    {
        current_pose_x_ = 0.0; // 当前位置
        current_pose_y_ = 0.0;
        current_pose_yaw_ = 0.0;
        pre_pose_x_ = 0.0; // 上一时刻位置
        pre_pose_y_ = 0.0;
        pre_pose_yaw_ = 0.0;

        path_sum_ = 0.0;

        is_pose_sub_ = false;
        is_path_server_ = false;
        receive_once_ = true;
        is_record_path_ = false;

        global_sum_ = 0;
        global_path_size_ = 0;
    }

    void RecordPath::odomCallback(const nav_msgs::OdometryConstPtr &msg) // 回调当前位置
    {
        current_pose_x_ = msg->pose.pose.position.x;
        current_pose_y_ = msg->pose.pose.position.y;
        current_pose_yaw_ = tf::getYaw(msg->pose.pose.orientation);
        is_pose_sub_ = true;
    }

    void RecordPath::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg) // 回调当前位置
    {
        current_pose_x_ = msg->pose.position.x;
        current_pose_y_ = msg->pose.position.y;
        current_pose_yaw_ = tf::getYaw(msg->pose.orientation);
        is_pose_sub_ = true;
    }

    bool RecordPath::pathServer(agv_robot::path::Request &req,
                                agv_robot::path::Response &res)
    {
        outFile_.close(); // 先关闭原有的

        int current_path_num = req.num;
        if (current_path_num >= 1 && current_path_num <= 1000)
        {
            is_record_path_ = true;

            std::string str1 = "/home/ubuntu/robot_csv/path_";
            std::string str2 = std::to_string(current_path_num);
            std::string str3 = ".csv";
            std::string str4 = str1 + str2 + str3;
            std::cout << str4 << std::endl;
            std::cout << "开始录制线路：" << current_path_num << std::endl;
            outFile_.open(str4, std::ios::app);
            if (!outFile_)
            {
                std::cout << "打开文件失败！" << std::endl;
                exit(1);
            }
            std::cout << "请移动机器人录制线路" << std::endl;
            path_index_ = 0;
            receive_once_ = false;

            ++path_sum_;
            res.sum = path_sum_;
            ROS_INFO("current_path_num path_sum_ [%d] [%d]", current_path_num, path_sum_);
            return true;
        }
        else if (current_path_num == 0)
        {
            std::cout << "暂停录制线路" << std::endl;
            outFile_.close();
            is_record_path_ = false;
            return true;
        }
        else
        {
            is_path_server_ = false;
            ROS_WARN("path num is error, please try again !!!");
            return false;
        }
        return true;
    }

    bool RecordPath::globalServer(agv_robot::path::Request &req,
                                  agv_robot::path::Response &res)
    {
        int global_path_num = req.num;
        if (global_path_num >= 1 && global_path_num <= 1000)
        {
            std::string str1 = "/home/ubuntu/robot_csv/path_";
            std::string str2 = std::to_string(global_path_num);
            std::string str3 = ".csv";
            std::string str4 = str1 + str2 + str3;

            std::cout << str4 << std::endl;

            // 读取保存的路径点
            std::ifstream data(str4);
            if (!data)
            {
                std::cout << "打开文件失败！" << std::endl;
                return false;
            }
            else
            {
                std::cout << "发布线路：" << global_path_num << std::endl;

                std::string line;
                std::vector<std::string> line_data;
                while (getline(data, line))
                {
                    std::stringstream lineStream(line);
                    std::string cell;
                    while (std::getline(lineStream, cell, ','))
                    {
                        line_data.push_back(cell);
                    }
                }

                vec_points_.clear();
                for (int i = 0; i < line_data.size(); i += 3)
                {
                    double _x = std::atof(line_data[i + 0].c_str());
                    double _y = std::atof(line_data[i + 1].c_str());
                    double _yaw = std::atof(line_data[i + 2].c_str());
                    Points pre_waypoints = {_x, _y, _yaw}; // 放入结构体中
                    vec_points_.push_back(pre_waypoints);
                }
                global_path_size_ = vec_points_.size();
                std::cout << "global_path_size_ " << global_path_size_ << std::endl;
                for (int i = 0; i < global_path_size_; i++)
                {
                    std::cout << i << " x " << vec_points_[i].x << " y " << vec_points_[i].y
                              << " yaw " << vec_points_[i].yaw << std::endl;
                }
            }

            pubGlobalPath();               // 发布线路点
            pubGlobalNum(global_path_num); // 发布线路编号

            ++global_sum_;
            res.sum = global_sum_;
            ROS_INFO("global_path_num global_sum_ [%d] [%d]", global_path_num, global_sum_);
            return true;
        }
        else if (global_path_num == 0)
        {
            std::cout << "暂停发布线路" << std::endl;
            return true;
        }
        else
        {
            ROS_WARN("path num is error, please try again !!!");
            return false;
        }
        return true;
    }

    void RecordPath::pubGlobalPath()
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        path.poses.clear();

        for (int i = 0; i < global_path_size_; i++)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = vec_points_[i].x;
            pose_stamped.pose.position.y = vec_points_[i].y;
            pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vec_points_[i].yaw);
            path.poses.push_back(pose_stamped);
        }
        global_path_pub_.publish(path); // 发布全局线路
    }

    void RecordPath::pubGlobalNum(int index)
    {
        std_msgs::Int32 msg;
        msg.data = index;
        global_num_pub_.publish(msg);
    }

    void RecordPath::MainLoop()
    {
        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            if (is_pose_sub_ && is_record_path_)
            {
                if (!receive_once_) // first subscribe 接收第一个数据
                {
                    // 位置信息以四元数的形式存储起来
                    outFile_ << current_pose_x_ << "," << current_pose_y_ << "," << current_pose_yaw_ << std::endl;
                    std::cout << "写入初始位置信息" << std::endl;

                    receive_once_ = true;

                    pre_pose_x_ = current_pose_x_; // 记录当前位置
                    pre_pose_y_ = current_pose_y_;
                    pre_pose_yaw_ = current_pose_yaw_;

                    std::cout << "pre_pose_x_ " << pre_pose_x_;
                    std::cout << " pre_pose_y_ " << pre_pose_y_;
                    std::cout << " pre_pose_yaw_ " << pre_pose_yaw_ << std::endl;
                }
                else
                {
                    // 计算这个距离数据，勾股定理计算
                    float distance = sqrt(pow((current_pose_x_ - pre_pose_x_), 2) +
                                          pow((current_pose_y_ - pre_pose_y_), 2));
                    // 如果小车移动了0.5米
                    if (distance >= interval_)
                    {
                        ++path_index_;
                        // 写入需要录取的数据
                        outFile_ << current_pose_x_ << "," << current_pose_y_ << "," << current_pose_yaw_ << std::endl;
                        std::cout << "正在进行第 " << path_index_ << " 个数据写入......" << std::endl;

                        pre_pose_x_ = current_pose_x_; // 更新当前位置
                        pre_pose_y_ = current_pose_y_;
                        pre_pose_yaw_ = current_pose_yaw_;
                        std::cout << "current_pose_x_ " << pre_pose_x_;
                        std::cout << " current_pose_y_ " << pre_pose_y_;
                        std::cout << " current_pose_yaw_ " << pre_pose_yaw_ << std::endl;
                    }
                }

                is_pose_sub_ = false;
            }
            // else
            // {
            //     ROS_WARN("has no current pose !!!");
            // }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
