#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /opt/dave_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11329
roscore -p 11329 > /tmp/image_only_core.log 2>&1 &
core_pid=$!
rec_pid=''
cleanup() {
 if [ -n "$rec_pid" ]; then kill -INT "$rec_pid" 2>/dev/null || true; wait "$rec_pid" 2>/dev/null || true; fi
 kill -INT "$core_pid" 2>/dev/null || true; wait "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2
rosparam set use_sim_time true
roslaunch bygd_rov record_pairs.launch sonar_image_topic:=sonar_image_gray output_dir:=/tmp/image-only-record-test max_pairs:=3 > /tmp/image_only_recorder.log 2>&1 &
rec_pid=$!
python3 /opt/dave_ws/src/bygd_rov/validation/underwater_wreck/replay_image_pair_test.py
sleep 1
python3 - <<'PY'
from pathlib import Path
import json,csv,cv2
p=sorted(Path('/tmp/image-only-record-test').iterdir())[-1]
m=json.loads((p/'metadata.json').read_text()); rows=list(csv.DictReader((p/'pairs.csv').open()))
assert m['written']==3 and m['error'] is None
assert len(list(p.glob('*_rgb.png')))==len(list(p.glob('*_sonar.png')))==3
assert not list(p.glob('*.npy')) and not list(p.glob('*.bag'))
assert len(m['topics'])==2 and all('depth' not in t for t in m['topics'])
assert all(abs(float(r['max_delta_ms'])-10)<.001 for r in rows)
assert cv2.imread(str(p/'000000_sonar.png'),cv2.IMREAD_UNCHANGED).ndim==2
report=dict(written=3,matching_tolerance_ms=10,period_s=m['sample_period'],npy_files=0,bag_files=0,subscriptions=m['topics'],error=None)
Path('/opt/dave_ws/src/bygd_rov/validation/underwater_wreck/image_only_recording_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
PY
