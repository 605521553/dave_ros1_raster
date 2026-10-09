# OBN 巡航采集

本次速度从 0.1 m/s 提高到 **0.5 m/s（五倍）**，下移扫描路线。
名义底面 world Z=-10；扫描段基准 Z=-8.6，中心离底约 1.4 m。
遇到岩石和悬浮颗粒时平缓横向避让或局部抬高，实际航点 Z 范围
[-8.590001515662946, -7.734458966875962]，相对名义底面约 1.41–2.27 m。
这里是机器人中心高度；实际局部地形高度不同。

八条南北扫描带覆盖地图；半圆掉头放在南北外围，采用现有 **cubic** 插值器，
降低提速后的航向变化尖峰。不要覆盖为 lipb，否则需重新验证。
浮力、质量、惯量、水动力、推进器、TAM、PID、传感器和地图均保持现有配置。
共享浮心仍为 [0,0,0.3]，中性浮力仍开启。

## 启动

先在原场景、控制器、轨迹和记录器终端按 Ctrl+C 停止旧任务，避免重复 PID。
两个终端依次运行，待机器人生成、仿真正常后启动巡航：

```bash
roslaunch bygd_rov obn_rov.launch lockstep:=false
roslaunch bygd_rov obn_inspection_trajectory.launch start_controller:=true record:=true
```

已单独启动 PID 时，第二条去掉 start_controller:=true。只巡航用 record:=false。
默认 5 秒采集一组 RGB/灰度声呐，提速后约间隔 2.5 m；需要更密采集可设
sample_period:=1.0（约 0.5 m）。默认输出 datasets/obn 独立会话目录。

起点 world ENU：[-12.0, -12.0, -7.99978243293304]，yaw=1.580932 rad；终点 [12.0, -12.0, -7.9986885930569835]。
两个机器人启动入口已对齐起点。单程扫描结束后保持终点，不自动返航或循环。
从其他位置出发时自动插入的接近路径不属于已验证路线。

## 检查结果

- 529 个航点，曲线长度约 354.39 m，预计 11.81 仿真分钟。
- 精确序列化 YAML 的 cubic 曲线检查 24001 个点，对照七种材质的
  504284 个三角面 AABB，包括悬浮颗粒。
- 半径 0.45 m 的机身/夹爪包络扣除采样间距后的保守最小净空
  0.702 m。
- 包含外置虚拟推进器的全姿态包络保守最小净空
  0.063 m；推进器碰撞球半径为 1 微米。
- 实际参考四元数全部有限，无局部反向折返，最大航向变化率约
  27.18 度/秒。
- 30 个网格节点、南部节点、回收篮表面代表点均有进入 RGB/声呐共同视场的机会。
  回收篮中心代表点未进入共同视场，但采样表面点有  1861 个进入机会。
  检查不包含遮挡，不保证定时采样会完整覆盖每个部件。

这是离线几何、配置和参考轨迹验证，尚未完成实际 Gazebo/PID/海流飞行测试。
净空不包含控制跟踪误差和超调，实际试跑需观察。更换插值器、地图或机器人几何后需重验。

## 本次修改

- config/obn/inspection_waypoints.yaml：五倍速度、降低高度、平滑绕障和外围掉头。
- launch/obn/obn_inspection_trajectory.launch：默认插值器改为 cubic。
- launch/obn/obn_rov.launch、obn_upload_rov.launch：出生位置和朝向对齐新起点。
- validation/obn/plan_inspection_route.py：新路线生成和完整曲线检查，通过后发布 YAML。
- validation/obn/check_motion_reference.py、check_inspection_coverage.py：适配 cubic 并验证。
- 更新 inspection_route_report.json、inspection_coverage_report.json、
  motion_reference_comparison.json、inspection_route_map.png 和本说明。
- 新增 validation/obn/smooth_route_before_lowering.yaml：保存本次修改前的 0.1 m/s 路线。

## 验证工具

```bash
python3 $(rospack find bygd_rov)/validation/obn/plan_inspection_route.py
python3 $(rospack find bygd_rov)/validation/obn/check_motion_reference.py
python3 $(rospack find bygd_rov)/validation/obn/check_inspection_coverage.py
```

更早的 previous_inspection_waypoints.yaml 含已知急转折返，仅供诊断。
材质网格 world pose 为 0 0 -10 0 0 0，scale=1。反射率配置保持现有值。


## 同步保存模型与材质信息

OBN 声纳现在加载独立的 `bygd_sonar_labels` 插件。
原 NPS 插件源代码和动态库保持原样；OBN 的 1024×600 扇形显示、固定 dB 窗口和 gamma
已移植到独立插件，传感器物理参数、地图、材质反射率及巡检路线没有改动。

仍使用这两个入口：

```bash
source /opt/dave_ws/env_ros1.sh
roslaunch bygd_rov obn_rov.launch
roslaunch bygd_rov obn_inspection_trajectory.launch start_controller:=true record:=true
```

第二条在另一终端运行，等待第一条生成机器人后再启动。
第一条也支持 `lockstep:=false`。两条不要同时设置 `record:=true`。
第一条如果单独使用 `record:=true`，同样启动新采集器，默认输出 datasets/obn。

每组现在包含 `*_rgb.png`、`*_sonar.png` 和 `*_objects.json`。
RGB/声纳近似同步（默认 30 ms）；清单与声纳时间戳及 frame_id 必须完全一致。
每项记录模型 ID、显式材质标签和采样命中数，允许一帧同时出现多个类别。
七类映射为 rock/sand/steel/polymer/rubber/cable/glass，对应 `obn_` 前缀模型。
这里的 cable 沿用场景定义的部件分类；类别并非真实材质识别结果。
命中数不表示物体数量或声纳回波强度比例；当前没有逐亮点归属或独立节点实例标签。

`object_hits_topic:=object_hits` 和 `object_wait_timeout:=2.0` 可传给任一采集入口。
前者选择订阅话题（插件仍默认发布 object_hits）；后者是清单等待的墙钟超时秒数。
缺失/无效清单不会与其他帧拼接，有效空清单可以正常保存。
CSV 增加清单路径和时间戳；metadata 在退出时记录保存、缺失和无效计数。
会话同时保存 OBN 配置及实际 robot_description，便于追溯映射。

已通过独立插件构建、启动配置检查，以及开启 PID/轨迹的短时 OBN 采集：
默认 5 秒采样间隔保存 3 组，RGB 为 640×480、声纳为 1024×600，七类模型映射正确，
声纳与清单严格同帧，缺失与无效计数均为零。
此项是采集功能验证，未完成整条路线的运行性能或碰撞验证。
报告和样例在 `bygd_sonar_labels/validation/obn_integration/`。

### 构建与回退

```bash
source /opt/dave_ws/env_ros1.sh
cd /opt/dave_ws
catkin build bygd_sonar_labels --no-deps -j 2
source devel/setup.bash
```

修改前的文件副本保存在 `bygd_sonar_labels/validation/obn_integration/before/`。
回退采集链路时，先停止仿真与记录器，再将 OBN sensors.xacro 的插件 filename
改回 `libnps_multibeam_sonar_ros_plugin.so`，并将两个 OBN 启动文件中的采集 include
改回 `$(find bygd_rov)/launch/offshore/record_pairs.launch`。
同时移除传给旧采集 launch 的 `object_hits_topic`、`object_wait_timeout` 两个参数。
也可核对备份恢复上述三个文件；恢复时保留此后其他任务的修改。
原声纳插件无需恢复或重新修改。
