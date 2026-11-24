# NavX

## env

[构建环境](docs/build.md)

## run

### mid360驱动

```sh
source install/setup.sh
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

#### 配置
launch文件中的xfer_format 为发布的点云话题类型
publish_freq为发布频率

## 建图

docs/建图.md

## 导航

docs/导航.md

### 定位

### super lio
```sh
source install/setup.sh
ros2 launch super_lio Livox_mid360.py
```

### z-localization

[backend.yaml](src/location/z-localization/config/backend.yaml)中的common下body: 表示了雷达和机器人自身的一个静态tf变换


#### 无先验地图 建图模式
```sh
source install/setup.sh
ros2 launch z_localization_bringup  bringup.launch.py   frontend_config:="$PWD/src/location/z-localization/config/lio.yaml"   backend_config:="$PWD/src/location/z-localization/pcd/site.yaml"   mode:=mapping rviz:=true
```
use_sim_time:=true ---> 使用ros2 bag 回放

```sh
ros2 bag play /absolute/path/to/bag --clock
```

##### 保存地图

回放结束后停止输入，待末尾观测和地图更新完成再保存：

```bash
ros2 service call /stop std_srvs/srv/Trigger "{}"

# 地图更新完成后执行
ros2 service call /backend_node/save_map std_srvs/srv/Trigger "{}"
```
返回 `success: true` 后再退出节点，结果在 `pcd/site_raw/`。
保留完整地图包，后续清理需要其中的关键帧和局部点云；同一目录再次保存会更新原有结果。


##### 自动清理动态残影

```sh
source install/setup.sh
ros2 run backend clean_map_bundle \
  --in "$PWD/pcd/site_raw" \
  --out "$PWD/pcd/site_auto_clean" \
  --lidar-origin -0.011 -0.02329 0.04412
```
这里的雷达光心位置对应默认 Mid-360 外参，在 IMU 系中表达。
清理及后续生成工具的输出目录须不存在或为空。

##### CloudCompare 手工收尾
如果是ubuntu apt下载的 CloudCompare无法加载pcd
```sh
# 安装 PDAL
sudo apt install pdal
pdal translate pcd/site_auto_clean/map.pcd map.ply
```

##### 网格生成地图包与描述子

**雷达水平安装时，作者建议精修后重新按网格生成。** 按作者的使用经验，这样生成的描述子通常比建图时直接保存的质量更好。

下面以 X：`0–28 m`、Y：`0–15 m`、采样 Z 坐标 `0 m`、雷达离地高度 `0.35 m` 为例，范围和高度按调整后的地图填写。

```sh
ros2 run backend build_pcd_map_bundle \
  --input "$PWD/pcd/site_refined.pcd" \
  --out-bundle "$PWD/pcd/site_prior" \
  --x-min 0 --x-max 28 --y-min 0 --y-max 15 \
  --sensor-z 0.0 --bundle-step 1.5 \
  --map-voxel 0.15 --scan-voxel 0.4 \
  --descriptor-type scan_context --sc-lidar-height 0.35
```

`--sensor-z` 是地图中的采样高度坐标，`--sc-lidar-height` 是离地高度，两者不一定相等。
本例按水平姿态、固定高度生成；描述子配置须与定位时一致，其余选项见[工具参数](src/location/z-localization/src/backend/docs/parameters.md#离线工具参数)。


#### 有先验地图 重定位+定位建图
```sh
ros2 launch bringup bringup.launch.py \
  frontend_config:="$PWD/config/lio.yaml" \
  backend_config:="$PWD/pcd/site.yaml" \
  mode:=localization prior_map:="$PWD/pcd/site_prior" \
  rviz:=true
```




### lightning-lm(不使用)
[解决编译卡死的问题](https://blog.csdn.net/qq_35102059/article/details/134083996?fromshare=blogdetail&sharetype=blogdetail&sharerId=134083996&sharerefer=PC&sharesource=2301_80216316&sharefrom=from_link)
编译卡死一般是swap分区太小了
参考[swap文档](docs/swap.md)

### ma_nav

#### 运行

```sh
source install/setup.sh
ros2 launch ma_nav run.launch.py
```

建图or导航
planner.yaml中的mapping_model 更改

保存服务的话题在planner.yaml中  save_map_srv_topic save_tunnel_regions_srv_topic

#### 保存地图
```sh
ros2 service call /save_global_map std_srvs/srv/Trigger "{}"
```

#### 保存地图的隧道语义
```sh
ros2 service call /save_tunnel_regions std_srvs/srv/Trigger "{}"
```


```sh
rviz2 -d nav.rviz
```

### decision,serial

```sh
source install/setup.sh
ros2 launch ma_decision run.launch.py
```

devision中config的param.yaml 选择是否开启决策，串口是一定打开的

### tool

```sh
source install/setup.sh
ros2 launch map_edit map_edit.launch.py 
```


### 问题

1. super lio 在接收点云 奇异值 会中断程序

2. nav