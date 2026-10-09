#!/usr/bin/env python3
"""Save synchronized RGB and sonar PNG pairs using simulated acquisition stamps.
Only two image streams are subscribed; no camera-depth NPY or duplicate bag is written.
"""
import csv
import json
import queue
import shutil
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Recorder:
    def __init__(self):
        self.namespace=rospy.get_param('~namespace','rexrov').strip('/')
        prefix='/'+self.namespace
        sonar_topic=rospy.get_param('~sonar_image_topic','sonar_image')
        # Relative names are resolved under the sensor namespace; absolute names are accepted.
        if not sonar_topic.startswith('/'):
            sonar_topic=prefix+'/sonar/'+sonar_topic
        self.topics=[prefix+'/rgbd/rgb/image_raw',sonar_topic]
        self.period=float(rospy.get_param('~sample_period',5.0))
        self.slop=float(rospy.get_param('~sync_slop',0.03))
        self.limit=int(rospy.get_param('~max_pairs',0))
        if self.period<0 or self.slop<=0 or self.limit<0:
            raise ValueError('sample_period >= 0, sync_slop > 0, max_pairs >= 0 required')
        self.path=Path(rospy.get_param('~output_dir')).expanduser()/(
            datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
        self.path.mkdir(parents=True,exist_ok=False)
        self.bridge=CvBridge();self.work=queue.Queue(maxsize=20)
        self.accepted=0;self.written=0
        self.start_sim_ns=rospy.Time.now().to_nsec()
        self.last_stamp=None;self.dropped=0;self.error=None;self.closing=False
        self.started=time.monotonic();self.last_pair_wall=None
        package=Path(__file__).resolve().parents[1]
        # Source path can be injected for catkin's devel executable wrapper.
        package=Path(rospy.get_param('~package_dir',str(package)))
        scene_config=Path(rospy.get_param(prefix+'/bygd_scene_config_dir', str(package/'config'/'offshore')))
        for name in ['rov_model.yaml','sensors.yaml']:
            source=scene_config/name
            if source.exists():shutil.copy2(str(source),str(self.path/name))
        self.metadata={'namespace':self.namespace,'config_dir':str(scene_config),'topics':self.topics,'sample_period':self.period,
            'sonar_image_topic':sonar_topic,'sonar_png_encoding':'same as source (bgr8 or mono8)',
            'sync_slop':self.slop,'saved_data':'RGB PNG and sonar PNG only; CSV and configuration metadata',
            'synchronization':'approximate RGB-sonar acquisition stamps within sync_slop'}
        self.csv_file=(self.path/'pairs.csv').open('w',newline='')
        self.csv=csv.writer(self.csv_file)
        self.csv.writerow(['index','rgb_stamp_ns','sonar_stamp_ns',
                           'max_delta_ms','rgb_file','sonar_file'])
        self.worker=threading.Thread(target=self.write_loop,daemon=True);self.worker.start()
        self.filters=[message_filters.Subscriber(t,typ,queue_size=60,buff_size=16*1024*1024)
                      for t,typ in zip(self.topics,[Image,Image])]
        self.sync=message_filters.ApproximateTimeSynchronizer(self.filters,60,self.slop)
        self.sync.registerCallback(self.pair)
        self.timer=rospy.Timer(rospy.Duration(5),self.status)
        rospy.on_shutdown(self.close)
        rospy.loginfo('Saving RGB / sonar pairs to %s',self.path)

    def pair(self,*msgs):
        if self.closing or (self.limit and self.accepted>=self.limit):return
        stamps=[m.header.stamp.to_nsec() for m in msgs]
        # Lazy sensor plugins may publish one cached frame on re-subscription.
        if self.start_sim_ns==0:self.start_sim_ns=rospy.Time.now().to_nsec()
        if min(stamps)<self.start_sim_ns:return
        stamp=stamps[0]
        if self.last_stamp is not None:
            if stamp<=self.last_stamp:return
            if (stamp-self.last_stamp)*1e-9<self.period-1e-9:return
        try:self.work.put_nowait((self.accepted,msgs))
        except queue.Full:
            self.dropped+=1;rospy.logwarn_throttle(5,'Writer queue full: skipping new pair');return
        self.accepted+=1;self.last_stamp=stamp;self.last_pair_wall=time.monotonic()

    def write_loop(self):
        try:
            while True:
                item=self.work.get()
                if item is None:break
                idx,msgs=item
                rgb,sonar=msgs
                rgb_name='%06d_rgb.png'%idx;sonar_name='%06d_sonar.png'%idx
                if not cv2.imwrite(str(self.path/rgb_name),self.bridge.imgmsg_to_cv2(rgb,'bgr8')):raise IOError('RGB image write failed')
                if sonar.encoding not in ('bgr8','mono8'):
                    raise ValueError('Expected sonar encoding bgr8 or mono8, got '+sonar.encoding)
                self.metadata['sonar_image_encoding']=sonar.encoding
                if not cv2.imwrite(str(self.path/sonar_name),self.bridge.imgmsg_to_cv2(sonar,'passthrough')):raise IOError('Sonar image write failed')
                stamps=[m.header.stamp.to_nsec() for m in msgs]
                self.csv.writerow([idx]+stamps+[(max(stamps)-min(stamps))/1e6,rgb_name,sonar_name])
                self.csv_file.flush();self.written+=1
                if self.limit and self.written>=self.limit:
                    rospy.signal_shutdown('Requested pair count reached');break
        except Exception as exc:
            self.error=str(exc);rospy.logerr('Pair recorder failed: %s',exc)
            rospy.signal_shutdown('Pair recorder failed')
        finally:
            self.csv_file.close()
            self.metadata.update(accepted=self.accepted,written=self.written,dropped_writer_queue=self.dropped,error=self.error)
            (self.path/'metadata.json').write_text(json.dumps(self.metadata,indent=2))

    def status(self,event):
        rospy.loginfo('Pairs saved=%d, queued=%d, dropped=%d',self.written,self.work.qsize(),self.dropped)
        if self.accepted==0 and time.monotonic()-self.started>20:
            rospy.logwarn('No synchronized pairs yet: check sensor topics and simulation clock')

    def close(self):
        if self.closing:return
        self.closing=True
        if threading.current_thread() is not self.worker and self.worker.is_alive():
            # Drain accepted work before closing the bag. Never duplicate old frames.
            self.work.put(None);self.worker.join()


if __name__=='__main__':
    rospy.init_node('record_sensor_pairs')
    recorder=Recorder()
    rospy.spin()
    if recorder.worker.is_alive():recorder.worker.join()
    if recorder.error:raise SystemExit(1)
