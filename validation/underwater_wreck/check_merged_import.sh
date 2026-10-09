#!/usr/bin/env bash
set -e
g++ -std=c++17 /opt/dave_ws/src/bygd_rov/validation/underwater_wreck/check_merged_import.cpp -o /tmp/check_merged_import $(pkg-config --cflags --libs gazebo)
/tmp/check_merged_import /opt/dave_ws/src/bygd_rov/meshes/underwater_wreck_scene/optimized/*.obj > /opt/dave_ws/src/bygd_rov/validation/underwater_wreck/merged_import_report.txt
cat /opt/dave_ws/src/bygd_rov/validation/underwater_wreck/merged_import_report.txt
