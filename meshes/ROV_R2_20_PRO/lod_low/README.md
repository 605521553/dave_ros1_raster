# R2-20 Pro 蓝白轻量模型

推荐使用本目录的 rov_lowpoly.obj，并将 rov_lowpoly.mtl 放在同一目录。另提供 r2_20_pro_lowpoly.fbx 和 .blend。

原始模型：3,724,680 三角面、3,824 个网格零件、93,003,280 字节 FBX。推荐版：108,074 三角面，减少 97.10%，合并为一个网格，保留并合并为 6 个材质；FBX 约 4.68 MB，OBJ 约 8.67 MB。

低细节版位于 D:/PyBlender/output_r2_20_pro：64,085 三角面，减少 98.28%；FBX 约 3.04 MB。小零件与推进器细节更粗，适合远距离显示。

参照旧 20 模型：焊接 1 微米内重复顶点、重算法向、三角化、按表面积分配各零件减面预算，保留小零件最低面数，然后合并。新模型零件更多，目标预算并非严格上限。推荐版目标为 100,000 面，低细节版目标为 50,000 面。

米制，保留 CAD 原点，沿用旧脚本轴变换 x_new=-y_old、y_new=x_old、z_new=z_old。推荐版尺寸约 0.502586 × 0.411640 × 0.356672 米。新模型前向、真实质心及传感器安装位姿需要结合机器人配置确认；未套用旧模型的传感器位置。

OBJ 已重新导入并核对尺寸。原始到推荐版的 5,001 个顶点采样：平均表面距离 0.410 mm，95% 分位 1.482 mm，最大 18.400 mm。推荐版到原始采样：95% 分位 0.382 mm，最大 3.672 mm。这是采样距离统计，不是全表面误差上界；小零件和螺纹细节有所损失。FBX 重新导入面数一致。低细节版 FBX 重新导入少一个退化三角面。

运行资源复制到 /home/ubuntu/dave_ros1_raster/ws/src/bygd_rov/meshes/ROV_R2_20_PRO；其中主目录为推荐版，lod_low 为低细节版。ROS 引用 package://bygd_rov/meshes/ROV_R2_20_PRO/rov_lowpoly.obj，scale="1 1 1"。已有 ROV 目录及配置未修改，未验证 Gazebo 实际帧率或碰撞性能。

处理脚本：D:/PyBlender/scripts/prepare_r2_20_pro_balanced.py；验证脚本：D:/PyBlender/scripts/validate_r2_20_pro_balanced.py。详细统计见 processing_report.json、validation_report.json，预览见 preview。


2026-10-09 更新：几何包围盒中心已移到原点，对象位置/旋转为 0、缩放为 1，场景仅保留一个 ROV 网格。先前关于保留 CAD 原点的说明已失效；先前 preview 图片仍为原坐标版本。居中前输出备份见 D:/PyBlender/diagnostics/r2_before_centering。
