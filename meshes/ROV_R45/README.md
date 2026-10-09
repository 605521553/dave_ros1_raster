# R-45 调整版轻量模型

推荐模型：r45_lowpoly.blend、r45_lowpoly.fbx；ROS 显示网格：rov_lowpoly.obj。OBJ 必须与 rov_lowpoly.mtl 和 textures 目录一起使用。

原始模型 1,933,687 三角面、1,124 个网格对象。输出 498,933 三角面，减少约 74.20%，保留外壳、框架、推进器、支撑杆、孔位和紧固件；微小表面细节有所简化。没有刻意移除 ROV 零件。

先焊接 1 微米内重复顶点、修正法向和三角化，再按表面积分配面数；每个零件至少保留原三角面数的 15% 或 64 面（不超过原面数），最后合并网格。完全相同的材质节点及参数合并，保留 39 种不同材质与两张原有内嵌贴图。Blender 材质节点保存在 blend 中；OBJ/MTL 仅表达其支持的材质参数，渲染器间光照和外观仍可能不同。

导入场景原有 672 个空对象与 6 台相机，均已移除。交付 blend/FBX 中只有一个 ROV 网格，没有地面、相机、灯光、空对象或预览标记。原始 FBX 未修改。

采用米制，沿用旧模型轴旋转 x_new=-y_old、y_new=x_old、z_new=z_old，然后以最终网格包围盒中心居中。对象位置和旋转均为 0，缩放为 1，几何包围盒中心为 (0,0,0)。尺寸约 0.681213 × 0.500101 × 0.448216 米。这是几何中心，不是实测质心；旧机器人原点、安装位姿和碰撞参数不能直接视为适用于本模型。

重新导入 OBJ 和 FBX 检查通过，FBX 三角面数 498,933。原模型到减面模型 5,015 点采样距离：平均约 0.0217 mm，95% 分位约 0.0829 mm，最大约 0.2663 mm。此为采样结果，不是全表面严格误差上界。没有缺失贴图。

ROS 资源目录：/home/ubuntu/dave_ros1_raster/ws/src/bygd_rov/meshes/ROV_R45。网格引用：package://bygd_rov/meshes/ROV_R45/rov_lowpoly.obj，scale="1 1 1"，visual origin="0 0 0"。提供 urdf/r45_visual.urdf 供显示加载；该文件未定义碰撞、质量、惯量或传感器。未替换既有场景的机器人配置，未测试实际 Gazebo 帧率。

复现顺序（使用 D:/PyBlender/.venv/Scripts/python.exe）：scripts/prepare_r45.py、scripts/finalize_r45.py、scripts/validate_r45.py。详细统计见 processing_report.json 和 validation_report.json；preview 仅含验证图片，不是场景对象。

ROS 容器 dave_noetic_raster 内 Assimp 成功读取 OBJ，check_urdf 成功解析 r45_visual.urdf。
