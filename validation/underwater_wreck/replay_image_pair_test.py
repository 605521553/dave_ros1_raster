import rospy,rosbag,time,copy
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
rospy.init_node('image_only_pair_test')
source='/tmp/wreck-record-test/20261008_074029_22be0d/paired_messages.bag'
images={}
with rosbag.Bag(source) as bag:
 for topic,msg,_ in bag.read_messages(topics=['/rexrov/rgbd/rgb/image_raw','/rexrov/sonar/sonar_image_gray']):
  images.setdefault(topic,msg)
pubs={t:rospy.Publisher(t,Image,queue_size=10) for t in images}
clock=rospy.Publisher('/clock',Clock,queue_size=10,latch=True)
end=time.monotonic()+8
while not all(p.get_num_connections() for p in pubs.values()):
 if time.monotonic()>end: raise RuntimeError('Recorder did not subscribe')
 time.sleep(.1)
for t in [10.,12.,15.1,20.2]:
 clock.publish(Clock(rospy.Time.from_sec(t))); time.sleep(.2)
 for topic,pub in pubs.items():
  msg=copy.deepcopy(images[topic]); msg.header.stamp=rospy.Time.from_sec(t+(.01 if 'sonar' in topic else 0)); pub.publish(msg)
 time.sleep(.5)
