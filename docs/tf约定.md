# tf 约定

## 样版
map---------->odom--------->|
                            |------> base_link(云台坐标系)--->下位机     
lidar_frame(mid360坐标系)--->|
                            |
camera_farme(相机坐标系)----->|

1. map-->odom 由重定位确定 

2. lidar_frame---->base_link or odom--->lidar_frame-->base_link 这个约定取决于开源的里程计
    也有可能开源的里程计不考虑实际机器人和雷达位置的一个变换，目前(2026.11.16)使用的有
    这个主要是用于描述实际机器人和雷达位置有一个固定的位置变换，导航模块需要的是机器人的位置而不是雷达的位置，所以增加一个固定变换

3. odom->base_link 由里程计发布(系名不一定一样)

4. 导航地图需要的是  
    a. 建图时：odom->base_link
    b. 导航时：map->odom->base_link 
其中目标点应该发布到世界坐标系下 ，导航时-->map ; 建图时-->odom

世界坐标系由[planner参数](src/navigation/config/planner.yaml) 中的ros2:frame_id决定

传入导航模块的点云应该是世界坐标系的，一般里程计输出的点云即可

5. camera_farme---->base_link 
    哨兵的机械设计是大小yaw,雷达在大yaw上，相机,电控imu在小yaw上,实际比赛时大小yaw会有一个相对运动
    考虑到这个相对运动，要从[serial](src/decision/src/serial) 让下位机发来，主要用于决策的追击

6. 发布/cmd_vel---->下位机  速度的参考系是base_link(云台坐标系)，但是mpc做控制时使用的是世界坐标系，在发布给serial前变换为base_link

7. 下位机结算云台坐标系---->底盘坐标系，不由导航处理



