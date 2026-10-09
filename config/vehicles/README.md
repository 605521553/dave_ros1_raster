# Vehicle geometry profiles

OBN and underwater_wreck load `config/vehicles/<rov_variant>/model.yaml`.
Both default to r45; `rov_variant:=r20` selects the preserved legacy geometry.

Required fields for every profile:
- visual_mesh: package URI (keep adjacent OBJ MTL and textures)
- visual_scale, visual_xyz, visual_rpy: three-element arrays
- collisions: list of boxes, each with name, xyz, rpy, size
- sensor_xyz: shared RGB-D/sonar mounting origin

Scene configuration retains sensor pitch, optical rotation and sensor settings.
Both profiles currently reuse RexROV dynamics and controllers. Future physical
parameters can be added separately; current loading does not consume them.

To add a vehicle, create another directory with the same model.yaml schema,
then pass `rov_variant:=<directory_name>`; no per-vehicle Xacro branches needed.
Validate collision envelope, sensor self-occlusion and route clearances before use.
The labeled recorder snapshots either profile as vehicle_model.yaml and records
vehicle_variant; robot_description.urdf is the complete effective configuration.
