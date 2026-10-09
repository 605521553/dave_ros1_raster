import json, time
from pathlib import Path
import numpy as np
import rospy
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage
from cv_bridge import CvBridge
import xml.etree.ElementTree as ET
rospy.init_node('diagnose_live_sonar_range',anonymous=True)
bridge=CvBridge(); data={}
def callback(key):
    def cb(msg):
        data.setdefault(msg.header.stamp.to_nsec(),{})[key]=msg
    return cb
subs=[rospy.Subscriber('/rexrov/sonar/'+topic,typ,callback(key),queue_size=2)
      for topic,typ,key in [('raster/depth',Image,'depth'),('sonar_image_raw',ProjectedSonarImage,'raw'),('sonar_image_gray',Image,'gray')]]
end=time.time()+40
samples=[]
while time.time()<end and len(samples)<3:
    for stamp,d in list(data.items()):
        if len(d)==3:
            depth=bridge.imgmsg_to_cv2(d['depth']); good=depth[np.isfinite(depth)&(depth>0)]
            raw=d['raw']; ranges=np.array(raw.ranges)
            a=np.frombuffer(bytes(raw.image.data),np.uint8).reshape(len(ranges),raw.image.beam_count)
            bands=[]
            for lo,hi in [(0,5),(5,10),(10,20),(20,30),(30,40)]:
                v=a[(ranges>=lo)&(ranges<hi)]
                bands.append({'range':[lo,hi],'geometry_pixels':int(((good>=lo)&(good<hi)).sum()),'raw_nonzero':int((v>0).sum()),'raw_max':int(v.max()) if v.size else 0})
            h,w=depth.shape
            fl=w/(2*np.tan(2.2689280276/2))
            yy,xx=np.indices(depth.shape)
            axial=depth/np.sqrt(1+((xx-w/2)/fl)**2+((yy-h/2)/fl)**2)
            samples.append({'stamp':stamp,'far_clip_axial_pixels':int(np.isclose(axial,40,atol=.01).sum()),'geometry_max':float(good.max()) if good.size else None,'geometry_percentiles':np.percentile(good,[0,50,95,100]).tolist() if good.size else [],'depth_shape':list(depth.shape),'raw_range_max':float(ranges[-1]),'bands':bands})
            del data[stamp]
    time.sleep(.1)
model=ET.fromstring(rospy.get_param('/rexrov/robot_description'))
sensor=model.find(".//sensor[@name='forward_sonar_raster']")
report={'samples':samples,'loaded_sensor_xml':ET.tostring(sensor,encoding='unicode')}
out=Path('/opt/dave_ws/src/bygd_rov/validation/underwater_wreck/live_range_report.json')
out.write_text(json.dumps(report,indent=2)); print(json.dumps({'samples':samples},indent=2))
