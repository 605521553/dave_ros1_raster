#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/dave_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11329 GAZEBO_MASTER_URI=http://localhost:11349
roslaunch bygd_rov "rov.launch" gui:=false lockstep:=false > /tmp/wreck_record_world.log 2>&1 &
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
