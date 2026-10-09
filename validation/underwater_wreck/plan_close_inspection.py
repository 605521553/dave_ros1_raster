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
from uuv_waypoints import WaypointSet, Waypoint
from uuv_trajectory_generator.path_generator.lipb_interpolator import LIPBInterpolator
import json, math
# Pipeline west/east passes, then a close loop around the wreck perimeter.
anchors=[(4,-8,0),(4,-17,0),(4,-20,0),(8.8,-20,0),(8.8,-17,0),(8.8,-10,0),(8.8,-3,0),(8.8,2,0),(4,2,0),(4,-3,0),(2,-8,0),(0,-12,0),(-4,-14,1.57),(-7.5,-13,1.57),(-7.5,-10,1.57),(-6,-6,1.57),(-4.8,-1,1.57),(-4.5,5,1.57),(-4.3,11,1.57),(-4,16.5,1.57),(-7,18.2,1.57),(-12,18.2,1.57),(-17,16,1.57),(-17,11,1.57),(-17,6,1.57),(-17,0,1.57),(-17,-6,1.57),(-17.5,-10,1.57),(-18.5,-13,1.57),(-18.5,-16,1.57),(-12,-16,1.57),(-6,-16,1.57),(-3,-13,1.57),(0,-10,0),(4,-8,0)]
pts=[]
for a,b in zip(anchors[:-1],anchors[1:]):
 n=int(np.ceil(np.linalg.norm(np.array(b[:2])-np.array(a[:2]))/1.5))
 for j in range(n):
  t=j/n; pts.append([a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]),-8.4,a[2]+t*(b[2]-a[2])])
pts.append([4,-8,-8.4,0]); pts=np.array(pts)
def distances(q):
 delta=np.maximum(np.maximum(boxes[None,:,0,:]-q[:,None,:],q[:,None,:]-boxes[None,:,1,:]),0)
 return np.linalg.norm(delta,axis=2)
# Minimize height at each point while reserving 0.60 m around a 0.45 m ROV sphere.
for i,pt in enumerate(pts):
 for z in np.arange(-8.4,-7.35,.05):
  if distances(np.array([[pt[0],pt[1],z]])).min()>=1.05: pts[i,2]=z; break
 else: raise RuntimeError('No low collision-free altitude at '+str(pt))
for iteration in range(25):
 wp=WaypointSet()
 for x,y,z,h in pts: wp.add_waypoint(Waypoint(x=float(x),y=float(y),z=float(z),max_forward_speed=.8,heading_offset=float(h),inertial_frame_id='world'))
 interp=LIPBInterpolator(); interp.init_waypoints(wp); assert interp.init_interpolator()
 q=np.array([interp.generate_pos(float(s)) for s in np.linspace(0,1,8001)])
 bad=[]; minimum=100; closest=None
 for start in range(0,len(q),64):
  chunk=q[start:start+64]; d=distances(chunk); mins=d.min(axis=1)
  idx=np.unravel_index(d.argmin(),d.shape)
  if d[idx]<minimum: minimum=float(d[idx]); closest=(labels[idx[1]],chunk[idx[0]].tolist())
  bad.extend(chunk[mins<1.04])
 if not bad: break
 for v in bad:
  nearest=np.argsort(np.linalg.norm(pts[:,:2]-v[:2],axis=1))[:3]
  pts[nearest,2]=np.minimum(pts[nearest,2]+.05,-7.35)
else: raise RuntimeError('Path not safe after altitude refinement: '+str(closest))
file=p/'config/underwater_wreck/inspection_waypoints.yaml'
file.write_text('# Close inspection, world ENU, speed 0.8 m/s. Adaptive obstacle clearance.\n# Heading is offset from travel direction; positive pi/2 looks into wreck loop.\ninertial_frame_id: world\nwaypoints:\n'+''.join('  - {point: [%.4f, %.4f, %.4f], max_forward_speed: 0.8, heading: %.5f, use_fixed_heading: false}\n'%tuple(pt) for pt in pts))
report={'waypoints':len(pts),'speed_m_s':.8,'cruise_z_range':[float(pts[:,2].min()),float(pts[:,2].max())], 'path_length_m':float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),'duration_s':float(interp._duration),'min_center_aabb_distance_m':minimum,'approx_min_surface_clearance_m':minimum-.45,'closest':closest,'validation':'8001 static LIPB samples against original per-object AABBs; no complete high-speed PID/current validation'}
(p/'validation/underwater_wreck/close_inspection_report.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
