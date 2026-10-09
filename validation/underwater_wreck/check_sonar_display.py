import json
from pathlib import Path
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
rospy.init_node('check_sonar_display', anonymous=True)
msg=rospy.wait_for_message('/rexrov/sonar/sonar_image_gray', Image, timeout=40)
image=CvBridge().imgmsg_to_cv2(msg, 'mono8')
out=Path('/opt/dave_ws/src/bygd_rov/validation/underwater_wreck')
cv2.imwrite(str(out/'sonar_display_1024x600.png'),image)
assert image.shape==(600,1024), image.shape
assert not image[:16,:].any()
assert not image[:,:16].any()
assert not image[:,-16:].any()
report={'width':msg.width,'height':msg.height,'encoding':msg.encoding,
        'nonzero_pixels':int((image>0).sum()),'border_check':'passed'}
(out/'sonar_display_report.json').write_text(json.dumps(report,indent=2))
print(report)
