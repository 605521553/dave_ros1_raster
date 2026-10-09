# Four-material underwater wreck inspection

## Labeled sonar integration

The scene now uses `libbygd_multibeam_sonar_labeled_ros_plugin.so` and the
`bygd_sonar_labels` recorder, matching the OBN integration. Sonar display is
1024x600, 130 degrees horizontal, approximately 20 degrees vertical, 20 m range,
1.2 MHz, with the fixed -30 to 60 simulation-amplitude dB window and gamma 1.5.
The existing shared camera/sonar mounting pitch and inspection route are retained.
Model/material mappings are wreck_stone/stone, wreck_wood/wood,
wreck_metal/metal, wreck_soil/soil. Hits are published on
`/rexrov/sonar/object_hits` and exactly joined to sonar timestamp/frame ID.

Start the world, then trajectory and recording in separate terminals:

```bash
roslaunch bygd_rov underwater_wreck_rov.launch
roslaunch bygd_rov underwater_wreck_inspection_trajectory.launch start_controller:=true record:=true
```

Default output is `datasets/underwater_wreck/<session>/`: RGB/sonar PNG,
per-pair `*_objects.json`, `pairs.csv`, metadata and configuration snapshots.
Hit counts count sampled rays, not individual objects or displayed brightness.
Do not enable record in both launches; that would create duplicate recorders.
Static Xacro/launch expansion and recorder exact-join checks pass; a new live
wreck recording is required after restarting to validate scene-specific output.

Original scene remains available using `roslaunch bygd_rov offshore_rov.launch`.
All scene, sensor, mesh and route modifications here apply to the new scene.

## Start and collect

```bash
roslaunch bygd_rov underwater_wreck_rov.launch lockstep:=false
roslaunch uuv_trajectory_control rov_pid_controller.launch uuv_name:=rexrov
roslaunch uuv_control_utils send_waypoints_file.launch uuv_name:=rexrov filename:=$(rospack find bygd_rov)/config/underwater_wreck/inspection_waypoints.yaml interpolator:=lipb
roslaunch bygd_rov record_pairs.launch sonar_image_topic:=sonar_image_gray sample_period:=5
```

Run these in separate terminals. The new default spawn is (4,-8,-8.4), yaw
-pi/2, matching the validated approach. Do not replay from an arbitrary
position: the UUV planner inserts an approach path which has not been checked.
Do not run a circular trajectory simultaneously with this waypoint route.

An alternative route wrapper is:

```bash
roslaunch bygd_rov underwater_wreck_inspection_trajectory.launch uuv_name:=rexrov record:=true
# If no controller exists, add start_controller:=true
```

Use only one recorder. Output files are saved in per-session directories
under bygd_rov/datasets. RGB-D and sonar acquisition is configured at 5 Hz;
recording selects approximately one synchronized pair every 5 seconds of
simulation time. All new-scene recording entries default to sample_period=5.
The shared standalone record_pairs.launch default is also 5; the original
offshore_rov.launch retains its explicit sample_period=1 setting.
The recorder snapshots the new scene sensor/model YAML when the new upload
launch publishes the namespace-specific bygd_scene_config_dir parameter.

## Camera and route

The new scene uses its own config/underwater_wreck/rov_model.yaml. RGB-D
and sonar remain co-located and now look down at pitch 0.35 rad (20.05 deg),
previously 0.15 rad (8.59 deg). Sonar range and image resolutions are unchanged.

The route contains 126 points at 0.8 m/s (four times the old 0.2 m/s).
It passes along both sides of the pipeline field, then follows the wreck's
east side more closely and loops around the north, west and south sides.
The route is about 172 m long, lasting about 215 seconds of simulation time.
Altitude starts at Z=-8.4 and is raised locally as needed, as high as -7.35,
to avoid rocks and suspended objects. The seabed is near Z=-10.
Heading is an offset from travel direction; +pi/2 faces inward on the wreck loop.

Static validation samples the actual LIPB trajectory at 8001 points against
5837 original per-object AABBs. A 0.45 m sphere encloses the ROV body/gripper
and visible mesh. Estimated minimum free clearance is about 0.60 m.
This is not a completed high-speed PID/current collision test. The original
0.2 m/s current is preserved; controller drift and overshoot consume clearance.
No geometry or buoyancy changes were made to obtain this route.

## Four materials and geometry

Four root Gazebo models are loaded: wreck_stone, wreck_wood, wreck_metal,
wreck_soil. Each has exactly one visual material and one imported submesh.
Original sand/sediment becomes soil; rock and shells become stone;
rust and steel become metal. The floating category originally used a growth
material with no reliable wood/metal labels, so it is assigned to wood as a
simulation approximation. Visual colors are unified per material.

All category geometry is preserved (179416 faces). Four derived OBJ/MTL
pairs in meshes/underwater_wreck_scene/materials4 contain the regrouped
faces with remapped indices. Original source files and older optimized
assets are retained. Each model has pose 0 0 -10 0 0 0; all local poses are
zero and scale is 1 1 1. No centering, rescaling or decimation is performed.
Sea surface, physics, illumination, GUI camera, current and buoyancy retain
their original settings.

Reflectivity is configured in config/underwater_wreck/reflectivity.csv:
stone 0.0012, wood 0.0006, metal 0.0040, soil 0.0002. These are simulated
coefficients, not measured properties. The sonar explicitly tracks these
four roots using fiducial tags, avoiding unrelated surface/robot models.
The existing NPS plugin and CSV relative-path resolution are unchanged.
Its expensive per-model pixel lookup and depth-triggered refresh remain
limitations; fewer models alone does not guarantee sustained 5 Hz output.

Regenerate material assets:

```bash
python3 $(rospack find bygd_rov)/scripts/build_wreck_material_meshes.py
```

Geometry, imported-submesh, path and recording results are stored under
validation/underwater_wreck/. No external package source or plugin library
was modified. Existing CMake install rules include all new runtime assets.

## RGB and sonar image-only output

The shared recorder now subscribes only to RGB and the selected sonar image.
Each accepted pair produces *_rgb.png and *_sonar.png. pairs.csv records their
acquisition stamps and difference; metadata.json and configuration YAMLs remain.
No camera-depth NPY, raw-sonar stream or duplicate rosbag is recorded. Existing
historical datasets are retained. The simulator's depth/raster sensors remain
because the sonar algorithm requires depth rendering; removing NPY output does
not eliminate that GPU computation.
