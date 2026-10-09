# ROV 轻量模型

## 使用

运行模型：`package://bygd_rov/meshes/ROV/rov_lowpoly.obj`，同目录保留 `rov_lowpoly.mtl`。
使用 `scale="1 1 1"`，模型已经统一为米制、X 前/Y 左/Z 上，无需额外旋转。
原始 FBX 已移至 `source/20.fbx`，未修改内容，SHA-256 见 processing_report.json。
原点保留 CAD 导入原点，不代表真实质心，也没有按含夹爪的包围盒重新居中。
原 FBX 只有一个材质，没有贴图；导出保留该材质，未人为添加颜色。

## 处理

使用 D:/PyBlender/.venv/Scripts/python.exe 和 bpy 5.2.2 LTS。
先焊接 1 微米内重复顶点、修正法向并三角化，再按零件表面积分配约 5 万三角面。
小零件保留最低面数，最后合并为一个显示网格。螺纹等微小细节有所简化。
原始 1,609,479 三角面，输出 49,749 三角面，OBJ 约 3.67 MB。
与原始轴的关系：x_new=-y_old, y_new=x_old, z_new=z_old。
转换脚本：bygd_rov/scripts/prepare_rov_mesh.py。
验证脚本：bygd_rov/scripts/validate_rov_mesh.py。
可用 `--target-triangles 50000` 调整目标，再运行验证脚本。

## RGB-D 与前视声呐公共安装位姿

配置见 bygd_rov/config/offshore/rov_model.yaml 和 bygd_rov/config/underwater_wreck/rov_model.yaml。目前是供后续机器人描述使用的元数据，未启动传感器。
相对本体 CAD 原点：xyz=[0.25, 0, 0.10] m，rpy=[0, 0.15, 0] rad。
位于机身前上方中线、前横梁前侧；沿 +X 观察，向下俯视约 8.59 度。
夹爪在下方伸得更远，因此安装点无需移到整个模型最大 X 之外。
两个传感器使用同一安装位姿；相机 optical frame 仍需标准轴变换。
若以后改变模型原点、比例或碰撞模型，需要重新核对该位置。

## 验证与限制

- OBJ 重新导入确认坐标/尺寸，并在 ROS 1 容器内用 Assimp 读取验证。
- 模型尺寸约 0.548695 × 0.380665 × 0.335573 m（长/宽/高，含夹爪）。
- 原模型到减面模型 5014 点抽样：95% 表面距离误差约 0.844 mm，最大约 8.62 mm。
  这是采样统计，不是严格的全表面误差上界。
- RGB-D 60.2° 水平视场（4:3）检查 3185 条射线，声呐 130°×12° 检查 3275 条射线，
  原模型、减面模型及重新导入 OBJ 均未检测到机身遮挡。
  这是几何采样验证，实际 Gazebo 传感器加载后仍需核验图像、TF 和自遮挡。
- preview/sensor_mount.png 的橙色点和线仅为位置/方向标记，不包含在 OBJ 内。
- FBX 备份和 preview 不会安装到运行资源目录；未将显示网格作为碰撞网格。
