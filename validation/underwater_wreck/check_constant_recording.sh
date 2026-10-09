#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/dave_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11329 GAZEBO_MASTER_URI=http://localhost:11349
xacro /opt/dave_ws/src/bygd_rov/urdf/underwater_wreck/bygd_rov.xacro > /tmp/wreck_constant.urdf
python3 -c "from pathlib import Path; p=Path('/tmp/wreck_constant.urdf'); p.write_text(p.read_text().replace('<constantReflectivity>false</constantReflectivity>', '<constantReflectivity>true</constantReflectivity>'))"
cat > /tmp/wreck_constant.launch <<'EOF'
<launch>
 <include file="$(find bygd_rov)/launch/underwater_wreck/world.launch"><arg name="gui" value="false"/><arg name="lockstep" value="false"/></include>
 <group ns="rexrov"><param name="robot_description" textfile="/tmp/wreck_constant.urdf"/><node pkg="gazebo_ros" type="spawn_model" name="spawn" args="-urdf -param robot_description -model rexrov -x 4 -y -8 -z -6"/></group>
</launch>
EOF
roslaunch /tmp/wreck_constant.launch > /tmp/wreck_record_world.log 2>&1 &
world_pid=$!
rec_pid=''
cleanup() {
 if [ -n "$rec_pid" ]; then kill -INT "$rec_pid" 2>/dev/null || true; fi
 kill -INT "$world_pid" 2>/dev/null || true
 wait "$world_pid" 2>/dev/null || true
 if [ -n "$rec_pid" ]; then wait "$rec_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
sleep 7
roslaunch bygd_rov record_pairs.launch sonar_image_topic:=sonar_image_gray output_dir:=/tmp/wreck-record-test > /tmp/wreck_record_pairs.log 2>&1 &
rec_pid=$!
rosnode info /gazebo > /tmp/wreck_gazebo_topics.txt
python3 /opt/dave_ws/src/bygd_rov/validation/underwater_wreck/recording_diagnostics.py
cat /tmp/wreck_record_pairs.log | tail -10
