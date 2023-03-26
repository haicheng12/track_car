```
开发人员：杨工

工作记录：
——2023年3月25号
1、代码重构，精简代码
2、跟线逻辑重构，解决小车到达终点启停无法继续前进的BUG
3、原有的3D和2D雷达避障保留，并且提取外参作为可修改参数
4、增加到达终点减速逻辑，位置PID控制
5、原有的小车当前消息状态通过topic发布
6、原有的RVIZ仿真环境保留，方便调试代码使用
7、发布全局线路改为ROS服务的形式，支持1-1000号的线路
8、使用ROS服务的形式录制线路，支持1-1000号的线路，提取两个路线点的间隔出来为可配置参数
9、增加行使途中可以更改速度的接口
10、支持缠绕的线路，录制线路时候有重合的线路不影响行使
11、增加小车行使过程中暂停的功能，取消暂停可以重新行使
```


**整体功能：

依赖多线激光雷达给出来的位置信息，录制线路，跟线行使


**纯跟踪参考：
```
原理介绍：https://blog.csdn.net/wang073081/article/details/104627100
代码参考：https://github.com/NeXTzhao/planning
```


**文件架构：
```
agv_robot包
src
record_path 录制线路，发布线路合一起，通过ROS服务调用接口。
pure_pursuit 跟线程序

msg 自定义的消息
robot_car_info.msg
uint16 state #小车状态机，1表示正常行使状态，2表示到达终点，3表示暂停
uint16 path_num #第几条线路
bool is_arrive_goal #是否到达终点
float64 speed #小车速度

srv 自定义的服务
path.csv
uint16 num #调用第几条线路
---
uint16 sum #调用服务的总次数
```

**仿真测试录制线路：

仿真环境启动：
```
$ roslaunch agv_robot sim_navigation.launch
```
键盘遥控：
```
$ rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard
```

线路录制服务启动（线路文件保存在/home/robot_csv/path_x）：
```
$ roslaunch agv_robot record_path.launch
```

如果使用真实小车，is_use_sim改为false
可以适当增加或在减少录制线路时候两个点的间隔，以适应更多的环境
```
<launch>
  <param name="interval" type="double" value="0.5" /> <!--录制时候两个点的间隔，单位：米-->
  <param name="is_use_sim" type="bool" value="true" /> <!--是否使用仿真-->
  <node pkg="agv_robot" type="record_path" name="record_path" output="screen">
  </node>
</launch>
```

选择录制第几条线路（目前支持1-1000）：
如果想录制第一条线路：
```
$ rosrun agv_robot record_client 1
```
如果想录制第二条线路：
```
$ rosrun agv_robot record_client 2
```
也就是说，不用关闭record_path程序，调用服务可以开始继续录制
如果要暂停录制线路，不用关闭record_path程序，发送：
```
$ rosrun agv_robot record_client 0
```
可以暂停录制线路，同时record_path程序挂在终端，随时可以发布新的线路编号来重新录制线路


**发布线路：

发布第几条线路（目前支持1-1000）：
如果想发布第一条线路：
```
$ rosrun agv_robot global_client 1
```
如果想发布第一条线路：
```
$ rosrun agv_robot global_client 2
```
如果要暂停发布线路，不用关闭record_path程序，发送：
```
$ rosrun agv_robot global_client 0
```
可以暂停发布线路，同时record_path程序挂在终端，随时可以发布新的线路编号来重新发布线路


**跟线：
```
<launch>
  <param name="car_velocity" type="double" value="0.4" /> <!--小车行走的速度，单位：米/秒-->
  <param name="point_distance" type="double" value="0.3" /> <!--相隔两个跟随点的误差范围，单位：米-->
  <param name="scan_distance_3D" type="double" value="0.5" /> <!--3D雷达停障碍距离，单位：米-->
  <param name="scan_distance_2D" type="double" value="0.8" /> <!--2D雷达停障碍距离，单位：米-->
  <param name="is_use_sim" type="bool" value="true" /> <!--是否使用仿真-->
  <node pkg="agv_robot" type="pure_pursuit" name="pure_pursuit" output="screen">
  </node>
</launch>
```

**启动跟线：
```
$ roslaunch agv_robot pure_pursuit.launch
```

可以通过命令来查看小车当前的状态信息：
```
$ rostopic echo /robot_car_info_msg
state: 3 #表示小车状态
path_num: 1 #表示第几条线路
is_arrive_goal: False #表示达到终点与否
speed: 0.4 #表示小车速度
---
state: 3
path_num: 1
is_arrive_goal: False
speed: 0.4
```

#行使过程中设置线速度大小（建议设置为0.1-0.5，单位：m/s）：
```
$ rostopic pub -1 /set_speed std_msgs/Float64 "data: 0.1" 
```

#行使过程中暂停小车或继续前进
暂停小车：
```
$ rostopic pub -1 /car_stop std_msgs/Bool "data: true"
```
继续前进：
```
$ rostopic pub -1 /car_stop std_msgs/Bool "data: false"
```


**提高跟线精度的方法：

把录制线路的间隔调小，把小车行使时候的线速度调小，把相隔两个跟随点的误差范围调小，
已去掉纠偏功能（理论上没这个必要）


**真实履带小车调试方法：

启动录制线路（不用关闭，发布线路也是这个程序）：
```
$ roslaunch agv_robot record_path.launch
```
录制第几条线路：
```
$ rosrun agv_robot record_client 1
```
遥控小车完成线路录制，如果要暂停录制，命令：
```
$ rosrun agv_robot record_client 0
```

启动跟线：
```
$ roslaunch agv_robot pure_pursuit.launch
```
启动发布线路命令：
```
$ rosrun agv_robot global_client 1
```
等待小车跟线完成，如果要暂停录跟线，命令：
```
$ rosrun agv_robot global_client 0
```
设置速度命令：
```
$ rostopic pub  /set_speed std_msgs/Float64 "data: 0.1" 
```



















