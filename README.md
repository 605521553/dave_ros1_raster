# BYGD 自定义 ROV：场景、运动与同位置双传感器

全部运行命令在 dave_noetic_raster 容器内执行；每个终端先 source /opt/dave_ws/env_ros1.sh。
修改只位于本包。不会自动启动 PID，避免与用户的原终端 3 产生重复控制器。

## 推荐启动（合并原终端 1、2）

```bash
roslaunch bygd_rov offshore_rov.launch record:=true
```

默认模型名/命名空间仍为 rexrov，但外形是 meshes/ROV/rov_lowpoly.obj。
默认出生位置 (4,-8,-6)，yaw=0，面向 (10,-8) 的单桩，便于看到有效双传感器数据。
可用 x:=... y:=... z:=... yaw:=... 调整。首次记录包含控制器启动前的运动阶段。
record 默认 false；true 每仿真秒保存一组，sample_period 可调整。

终端 3（保持原命令）：

```bash
roslaunch uuv_trajectory_control rov_pid_controller.launch uuv_name:=rexrov
```

终端 4（保持原工具，坐标改为当前浅水场景）：

```bash
roslaunch uuv_control_utils start_circular_trajectory.launch   uuv_name:=rexrov radius:=1 center_x:=3 center_y:=-8 center_z:=-6   max_forward_speed:=0.3
```

不要照搬旧世界的 center_z=-20：当前海床约 -10 米，那会要求机器人钻入海床。
圆形轨迹期间机头沿轨迹转向，传感器也随之转动，不保证全程对准单桩。
也可使用原航点工具加载本包的固定朝向路线：

```bash
roslaunch uuv_control_utils send_waypoints_file.launch uuv_name:=rexrov filename:=/opt/dave_ws/src/bygd_rov/config/offshore/offshore_waypoints.yaml
```

控制器不具备自动避障。

## 保留四终端拆分方式

1. `roslaunch bygd_rov offshore_world.launch lockstep:=true`
2. `roslaunch bygd_rov upload_bygd_rov.launch`
3. 原 PID 命令。
4. 原圆形/航点发送工具。

组合入口和拆分入口二选一，不能同时启动两个世界或两个同名 ROV。
如果修改 namespace，原 PID 还需指定 model_name:=rexrov；当前默认接口无需修改。

## RGB-D / 光栅声呐

- `/rexrov/rgbd/rgb/image_raw`：640×480 RGB，无水下后处理。
- `/rexrov/rgbd/depth/image_raw`：32FC1，米制深度，保留无效值。
- `/rexrov/rgbd/rgb/camera_info`、`/rexrov/rgbd/depth/camera_info`：内参。
- `/rexrov/sonar/sonar_image`：声呐显示图（当前 512×399，宽=波束，高=距离采样）。
- `/rexrov/sonar/sonar_image_raw`：原生 ProjectedSonarImage，保存于 rosbag。
- `/rexrov/sonar/raster/depth`、`raster/points`：光栅插件内部深度/点云。

两个传感器使用同一个 sensor_mount_link：xyz=(0.25,0,0.10)，rpy=(0,0.15,0)。
两个 optical frame 之间是单位变换；图像仍不是逐像素对应。
RGB-D HFOV=1.05 rad，近远裁剪=0.1/20 m，内参由 FOV 推导 fx=fy≈552.467，cx=320、cy=240。
声呐使用 libnps_multibeam_sonar_ros_plugin.so 的 depth/raster/CUDA 实现，不使用 ray 版本。
HFOV≈130°，512×25 的透视深度采样对应实际 VFOV≈11.96°，声学垂直波束宽度配置 12°。
因为投影方式不同，不能同时照搬 ROS 2 的 512×300 射线采样和 130°×12° 视场。
其余初始参数：900 kHz、29.9 kHz、1500 m/s、源级 220、成像距离 10 m、增益 0.02。
raySkips=1，常反射率；不把视觉颜色当成已标定的声学反射率。

采集入口默认 lockstep=true，优先保证传感器的仿真时间 20 Hz。
这不保证每墙钟秒 20 帧：大场景/GPU 渲染可能使仿真慢于现实。
可设置 lockstep:=false，但传感器会跳帧，不能只凭 update_rate 声称实际达到 20 Hz。
插件按订阅激活计算，开始查看或记录话题后才会持续生成传感器输出。

原光栅插件的原生 ping.beam_directions 使用 (cos(a),sin(a),0)，但 header 使用 optical frame。
本包不修改插件原始消息；若要把该数组用于光学坐标投影，应转换为 (old_y,0,old_x)。
不要直接将原始数组当成符合 optical frame 的三维方向；图像/点云消息使用光学坐标。

## 保存配对数据

未使用 record:=true 时可单独启动：

```bash
roslaunch bygd_rov record_pairs.launch max_pairs:=20 sample_period:=1.0
```

每次创建 datasets/日期_时间_随机ID/。可设置 output_dir 为容器内其它持久化路径。
保存 RGB PNG、米制深度 NPY、声呐 PNG、pairs.csv、paired_messages.bag、配置快照和 metadata.json。
只对新帧一对一匹配：RGB/深度必须同时间戳，声呐显示图/原始 ping 必须同时间戳；
两个传感器之间默认允许 30 ms 时间差，CSV 记录真实时间差，不篡改消息头。
实际测试可得到零时间差，但不保证所有机器、所有帧都严格同步。
写盘使用有限后台队列，过载丢弃新组并报告，不用旧帧补齐数据。
max_pairs=0 持续保存，Ctrl+C 会等待已接收组写入并关闭 bag。
Bag 还保存相机内参、静态 TF 和最近的 pose_gt（保留自身时间戳，不冒充插值位姿）。

## 修改入口

- config/offshore/rov_model.yaml：外形资源及公共传感器安装位姿。
- config/offshore/sensors.yaml：传感器参数；修改后需重新生成机器人。
- urdf/offshore/bygd_rov.xacro：自定义外形、简化碰撞和运动底座。
- urdf/offshore/sensors.xacro：双传感器插件、话题及 optical TF。
- launch/offshore/upload_bygd_rov.launch：仅机器人生成。
- launch/offshore/offshore_rov.launch：场景与机器人组合入口。
- scripts/record_pairs.py：无后处理的同步采集。

重要：当前动力学仍是 RexROV 兼容底座（约 1863 kg、原水动力和八个不可见虚拟推进器），
仅用于复用原 PID/TAM 和验证移动采集。外形尺寸约 0.55 m，二者物理尺度并不匹配。
它不是已标定的真实小型 ROV；后续替换真实动力学必须同步修改浮力、惯量、推进器、TAM 和 PID。

## 验证

validation/ 下保存 Gazebo 实际模型截图、同位置 RGB/声呐样例及运行报告。
测试采用独立 ROS/Gazebo 端口，结束后关闭，不占用用户常规的 11311 世界。

### 本机实测结果（2026-09-30）

- 自定义模型在 Gazebo 正常显示；八个虚拟推进器不可见。
- RGB-D 与声呐 optical TF 为零位移、单位旋转；内参 fx=fy≈552.467，cx=320、cy=240。
- 锁步稳态：RGB/声呐均为仿真时间 20 Hz，墙钟输出约 4.4 Hz。
- 锁步圆形运动采样窗口：位移约 0.958 m，最大位置跟踪误差约 0.0268 m。
- 多次记录共 23 组有效配对，测试组时间戳差均为 0 ms，深度与原生 ping 均保存。
- 原 send_waypoints_file.launch 能载入本包路线并开始跟踪。
- 结果只代表这些测试窗口；并非对全部轨迹、实时性能或真实动力学的保证。
