from pathlib import Path
import numpy as np
p=Path(__file__).resolve().parents[2]
boxes=[]; labels=[]
for f in sorted((p/'meshes/underwater_wreck_scene').glob('*.obj')):
 if f.stem=='underwater_wreck_scene': continue
 vs=[]; name=''
 def add():
  if vs:
   v=np.array(vs); v[:,2]-=10; boxes.append([v.min(axis=0),v.max(axis=0)]); labels.append(f.stem+':'+name)
 for line in f.open():
  if line.startswith('o '): add(); vs=[]; name=line.strip()[2:]
  elif line.startswith('v '): vs.append(list(map(float,line.split()[1:4])))
 add()
boxes=np.array(boxes)
from uuv_waypoints import WaypointSet
from uuv_trajectory_generator.path_generator.lipb_interpolator import LIPBInterpolator
wp=WaypointSet(); assert wp.read_from_file(str(p/'config/underwater_wreck/inspection_waypoints.yaml'))
interp=LIPBInterpolator(); interp.init_waypoints(wp); assert interp.init_interpolator()
q=np.array([interp.generate_pos(float(s)) for s in np.linspace(0,1,8001)])
minimum=100.; closest=None
for start in range(0,len(q),64):
 chunk=q[start:start+64]
 delta=np.maximum(np.maximum(boxes[None,:,0,:]-chunk[:,None,:],chunk[:,None,:]-boxes[None,:,1,:]),0)
 dist=np.linalg.norm(delta,axis=2); idx=np.unravel_index(dist.argmin(),dist.shape)
 if dist[idx]<minimum: minimum=float(dist[idx]); closest=(labels[idx[1]],chunk[idx[0]].tolist())
import json, hashlib
max_gap=float(np.linalg.norm(np.diff(q,axis=0),axis=1).max())
# Point-to-AABB distance is a conservative bound on distance to its geometry.
# The radius 0.45 encloses the visual mesh and physical body/gripper boxes.
# Sampling is an approximate check, not a proof of dynamic collision avoidance.
report=dict(waypoint_count=wp.num_waypoints,interpolator='lipb',samples=len(q),
 object_aabbs=len(boxes),min_center_aabb_distance_m=minimum,
 enclosing_rov_radius_m=.45,approx_min_surface_clearance_m=minimum-.45-max_gap/2,
 tracking_allowance_m=.55,closest_object=closest[0],closest_position=closest[1],
 max_sample_gap_m=max_gap,approx_path_length_m=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),
 duration_s=float(interp._duration),route_sha256=hashlib.sha256((p/'config/underwater_wreck/inspection_waypoints.yaml').read_bytes()).hexdigest(),
 limitations='Static sampled geometry only; no full PID/current flight test or arbitrary initial-position approach check')
assert report['approx_min_surface_clearance_m']>.55,report
(p/'validation/underwater_wreck/inspection_route_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))

# Export a static route map alongside the verification report.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
fig,ax=plt.subplots(figsize=(9,7))
for box,label in zip(boxes,labels):
 cat=label.split(':')[0]
 if cat not in ('shipwreck','pipes'): continue
 lo,hi=box
 ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],facecolor=('#967254' if cat=='shipwreck' else '#31958c'),edgecolor='none',alpha=.3))
ax.plot(q[:,0],q[:,1],color='#1d4ed8',linewidth=1.8,label='LIPB inspection route')
points=np.array([wp.get_waypoint(i).pos for i in range(wp.num_waypoints)])
ax.scatter(points[:,0],points[:,1],s=12,color='#1d4ed8')
for i in [0,10,20,30,40,50,60,70,80,90,100,110,125]:
 ax.annotate(str(i+1),points[i,:2],xytext=(5,5),textcoords='offset points',fontsize=8)
for i in [1400,2700,3900,5000,6500,7400]:
 ax.annotate('',xy=q[i+50,:2],xytext=q[i,:2],arrowprops=dict(arrowstyle='->',color='#1d4ed8'))
ax.scatter([4],[-8],s=65,color='#ef4444',zorder=5,label='Start / finish (4, -8, -8.4)')
ax.text(7,-7,'Pipeline field',fontsize=10,color='#17665f')
ax.text(-13,4,'Shipwreck',fontsize=10,color='#785539')
ax.set(xlim=(-22,15),ylim=(-22,22),xlabel='World X (m)',ylabel='World Y (m)',title='Near-bottom inspection: pipe field and wreck perimeter')
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='upper right',fontsize=8)
fig.tight_layout(); fig.savefig(p/'validation/underwater_wreck/inspection_route_map.png',dpi=150)
