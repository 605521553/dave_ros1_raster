"""Capture synchronized raster depth, sonar fan and raw beams for orientation QA."""
import rospy
import message_filters
import cv2
import numpy as np
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage
from cv_bridge import CvBridge
from pathlib import Path
import threading
import time
import os
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

rospy.init_node('verify_sonar_orientation')
out = Path(__file__).parent / 'orientation_fix'
out.mkdir(exist_ok=True)
bridge = CvBridge()
side = os.environ.get('TEST_TARGET_SIDE')
if side:
    out = out / side
    out.mkdir(exist_ok=True)
    try:
        rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)('orientation_target')
    except rospy.ServiceException:
        pass
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = 105, (2 if side == 'left' else -2), -6.7
    pose.orientation.w = 1
    sdf = '''<sdf version="1.6"><model name="orientation_target"><static>true</static><link name="target"><visual name="v"><geometry><box><size>1 1 3</size></box></geometry></visual><collision name="c"><geometry><box><size>1 1 3</size></box></geometry></collision></link></model></sdf>'''
    rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)('orientation_target', sdf, '', pose, 'world')

# Hold the initial launch pose during capture (no controller is launched).
rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
set_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
state = ModelState(model_name='rexrov', reference_frame='world')
state.pose.position.x = 100 if side else 4
state.pose.position.y = 0 if side else -8
state.pose.position.z = -6
state.pose.orientation.w = 1
def hold_pose():
    while not rospy.is_shutdown():
        try:
            set_pose(state)
        except rospy.ServiceException:
            return
        time.sleep(0.02)
threading.Thread(target=hold_pose, daemon=True).start()
time.sleep(2)

def capture(depth, fan, raw):
    capture.frames += 1
    if capture.frames < 20:
        return
    d = bridge.imgmsg_to_cv2(depth, 'passthrough')
    cv2.imwrite(str(out / 'sonar.png'), bridge.imgmsg_to_cv2(fan, 'bgr8'))
    np.save(str(out / 'depth.npy'), d)
    a = np.frombuffer(bytes(raw.image.data), np.uint8).reshape(-1, raw.image.beam_count)
    np.save(str(out / 'raw.npy'), a)
    peak = np.array(raw.ranges)[a.argmax(axis=0)]
    expected = np.nanmedian(d, axis=0)
    valid = (expected > 0.2) & (expected < 9.5) & (a.max(axis=0) > 5)
    report = {
        'valid_columns': int(valid.sum()),
        'median_range_error_same': float(np.median(abs(peak[valid] - expected[valid]))),
        'median_range_error_mirrored': float(np.median(abs(peak[::-1][valid] - expected[valid]))),
        'first_direction': str(raw.beam_directions[0]),
        'last_direction': str(raw.beam_directions[-1]),
    }
    if side:
        energy = a.sum(axis=0)
        left, right = int(energy[:256].sum()), int(energy[256:].sum())
        fan_pixels = bridge.imgmsg_to_cv2(fan, 'bgr8').sum(axis=2)
        fan_left, fan_right = int(fan_pixels[:, :256].sum()), int(fan_pixels[:, 256:].sum())
        report.update(raw_left=left, raw_right=right, fan_left=fan_left, fan_right=fan_right)
        report['passed'] = bool((left > right and fan_left > fan_right) if side == 'left' else (right > left and fan_right > fan_left))
    (out / 'report.txt').write_text(str(report))
    print(report, flush=True)
    rospy.signal_shutdown('captured')

capture.frames = 0
subs = [message_filters.Subscriber('/rexrov/sonar/' + topic, cls) for topic, cls in
        [('raster/depth', Image), ('sonar_image', Image), ('sonar_image_raw', ProjectedSonarImage)]]
sync = message_filters.TimeSynchronizer(subs, 30)
sync.registerCallback(capture)
rgb = rospy.wait_for_message('/rexrov/rgbd/rgb/image_raw', Image, timeout=60)
cv2.imwrite(str(out / 'rgb.png'), bridge.imgmsg_to_cv2(rgb, 'bgr8'))
rospy.spin()
