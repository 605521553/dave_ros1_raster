from pathlib import Path
import numpy as np, json
from uuv_waypoints import WaypointSet
from uuv_trajectory_generator.path_generator.cs_interpolator import CSInterpolator
from uuv_trajectory_generator.path_generator.lipb_interpolator import LIPBInterpolator
p=Path(__file__).resolve().parents[2]
report={}
for name,path in [('previous',p/'validation/obn/smooth_route_before_lowering.yaml'),('updated',p/'config/obn/inspection_waypoints.yaml')]:
 wp=WaypointSet(); assert wp.read_from_file(str(path))
 pts=np.array([wp.get_waypoint(i).pos for i in range(wp.num_waypoints)])
 yaw0=np.arctan2(pts[1,1]-pts[0,1],pts[1,0]-pts[0,0])
 ip=CSInterpolator() if name=='updated' else LIPBInterpolator(); ip.init_waypoints(wp,init_rot=np.array([0,0,np.sin(yaw0/2),np.cos(yaw0/2)])); assert ip.init_interpolator()
 if name=='updated':
  d=ip.generate_pos(1e-8)-ip.generate_pos(0); yaw0=np.arctan2(d[1],d[0]); ip._init_rot=np.array([0,0,np.sin(yaw0/2),np.cos(yaw0/2)])
 s=np.linspace(0,1,24001); q=np.array([ip.generate_pos(float(v)) for v in s])
 quat=np.array([ip.generate_quat(float(v)) for v in s]); x,y,z,w=quat.T
 raw_yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
 invalid=int((~np.isfinite(quat).all(axis=1)).sum())
 yaw=np.unwrap(raw_yaw) if invalid==0 else None
 dt=ip._duration/(len(q)-1)
 v=np.diff(pts[:,:2],axis=0)
 turn=np.abs(np.arctan2(v[:-1,0]*v[1:,1]-v[:-1,1]*v[1:,0],(v[:-1]*v[1:]).sum(1)))
 tangent_yaw=np.unwrap(np.arctan2(np.diff(q[:,1]),np.diff(q[:,0])))
 report[name]=dict(nonfinite_reference_quaternion_samples=invalid,max_tangent_yaw_rate_deg_s=float(np.rad2deg(np.abs(np.diff(tangent_yaw)/dt)).max()),waypoints=len(pts),max_actual_reference_yaw_rate_deg_s=float(np.rad2deg(np.abs(np.diff(yaw)/dt)).max()) if yaw is not None else None,reversals_over_170_deg=int((turn>np.deg2rad(170)).sum()),max_waypoint_turn_deg=float(np.rad2deg(turn).max()),max_reference_vertical_speed_m_s=float(np.abs(np.diff(q[:,2])/dt).max()),max_reference_heading_step_deg=float(np.rad2deg(np.abs(np.diff(yaw))).max()) if yaw is not None else None)
assert report['updated']['max_actual_reference_yaw_rate_deg_s']<30.,report
assert report['updated']['reversals_over_170_deg']==0
assert report['updated']['nonfinite_reference_quaternion_samples']==0
report['limitations']='Actual configured interpolator reference quaternion and vertical speed checks, not a Gazebo/PID flight test.'
(p/'validation/obn/motion_reference_comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps(report,indent=2,allow_nan=False))


