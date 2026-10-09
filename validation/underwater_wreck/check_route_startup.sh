#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/dave_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11329 GAZEBO_MASTER_URI=http://localhost:11349
roslaunch bygd_rov rov.launch gui:=false paused:=false lockstep:=false > /tmp/wreck_route_world.log 2>&1 &
world_pid=$!
route_pid=''
cleanup() {
  if [ -n "$route_pid" ]; then kill -INT "$route_pid" 2>/dev/null || true; fi
  kill -INT "$world_pid" 2>/dev/null || true
  wait "$world_pid" 2>/dev/null || true
  if [ -n "$route_pid" ]; then wait "$route_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
sleep 7
roslaunch bygd_rov inspection_trajectory.launch start_controller:=true > /tmp/wreck_route_controller.log 2>&1 &
route_pid=$!
sleep 23
grep -E 'waypoints|Waypoints|trajectory|Trajectory|ERROR|Error|success|Failed|failed|Exception|Traceback' /tmp/wreck_route_controller.log | tail -45 || true
