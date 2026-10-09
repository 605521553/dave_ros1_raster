import rospy,time,json
from sensor_msgs.msg import Image
from marine_acoustic_msgs.msg import ProjectedSonarImage
from rosgraph_msgs.msg import Clock
rospy.init_node('inspect_recording',anonymous=True)
streams={k:[] for k in ['rgb','depth','gray','raw','clock']}
def cb(msg,k):
 if k=='clock': streams[k].append(msg.clock.to_nsec())
 else: streams[k].append(msg.header.stamp.to_nsec())
subs=[]
for topic,typ,k in [('/rexrov/rgbd/rgb/image_raw',Image,'rgb'),('/rexrov/rgbd/depth/image_raw',Image,'depth'),('/rexrov/sonar/sonar_image_gray',Image,'gray'),('/rexrov/sonar/sonar_image_raw',ProjectedSonarImage,'raw'),('/clock',Clock,'clock')]:
 subs.append(rospy.Subscriber(topic,typ,cb,callback_args=k,queue_size=100,buff_size=16*1024*1024))
time.sleep(55)
report={k:dict(count=len(v),first=v[:3],last=v[-3:]) for k,v in streams.items()}
rgb=set(streams['rgb']); depth=set(streams['depth']); gray=set(streams['gray']); raw=set(streams['raw'])
report['exact_rgb_depth_matches']=len(rgb & depth); report['exact_gray_raw_matches']=len(gray & raw)
rd=sorted(rgb & depth); gr=sorted(gray & raw)
report['nearest_cross_sensor_deltas_ms']=[min(abs(t-s) for s in rd)/1e6 for t in gr[-20:]] if rd else []
for k in ('rgb','depth','gray','raw'):
 v=streams[k]
 report[k]['sim_hz']=(len(v)-1)*1e9/(v[-1]-v[0]) if len(v)>1 and v[-1]>v[0] else None
from pathlib import Path
(Path(__file__).resolve().parent/'optimized_recording_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
