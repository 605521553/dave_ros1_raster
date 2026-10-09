#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/dave_ws/devel/setup.bash
for scene in obn underwater_wreck; do
  for variant in r20 r45; do
    xacro /opt/dave_ws/src/bygd_rov/urdf/$scene/bygd_rov.xacro namespace:=rexrov rov_variant:=$variant > /tmp/${scene}_${variant}.urdf
    check_urdf /tmp/${scene}_${variant}.urdf >/dev/null
  done
  roslaunch --nodes bygd_rov ${scene}_rov.launch rov_variant:=r45 record:=true
  roslaunch --nodes bygd_rov ${scene}_rov.launch rov_variant:=r20
done
python3 /opt/dave_ws/src/bygd_sonar_labels/validation/test_exact_join.py
