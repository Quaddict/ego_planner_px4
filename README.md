# ego_planner_px4
运行效果见B站：https://www.bilibili.com/video/BV1AdX8YaEzS/?spm_id_from=333.1387.homepage.video_card.click

## 功能介绍
实现了ego-Planner 与 PX4 的联合仿真

## package介绍
1. ego-planner
   源自于ZJU-FAST-Lab的开源项目，编译前需要安装相关的功能包，具体可以去ego-planner的github仓库去看看。
   
2.offboard_px4
  该功能包中包含了本人所使用到的仿真环境配置和无人机型号，可以进行一个参考

## 程序运行步骤
  1.把indoor.launch文件放在PX4-Autopilot中运行,启动仿真环境，会出现一架携带深度相机的无人机
  
    roslaunch px4 indoor.launch

  2. 因为仿真中双目相机的VIO定位会存在漂移的问题，所以通过直接获取gazebo中无人机的位姿真值进行定位

    rosrun offboard_px4 get_local_pose

  3.因为在ego-planner中pose_type使用的是相机的坐标系，所以需要转换相机位姿的坐标系方向
  
    python ego_transfer.py iris 0

  4.切换offboard模式，并接收ego-planner发出的目标路径点的信息
  
    rosrun offboard_px4 ego_to_mavros

  5. 输入完以上命令后，先在QGC上控制无人机起飞，切换offboard模式，然后再启动ego-planner命令
     
    roslaunch ego-planner single_px4.launch

  6.运行rviz可视化
  
    roslaunch ego-planner rviz.launch
  
