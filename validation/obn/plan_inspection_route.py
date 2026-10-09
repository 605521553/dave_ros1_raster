from pathlib import Path
import numpy as np, json, hashlib
from scipy.spatial import cKDTree
from uuv_waypoints import WaypointSet, Waypoint
from uuv_trajectory_generator.path_generator.cs_interpolator import CSInterpolator
p=Path(__file__).resolve().parents[2]
lo=[]; hi=[]; cats=[]
for f in sorted((p/'meshes/obn_scene/Material').glob('*.obj')):
 v=[]; faces=[]
 for l in f.open():
  if l.startswith('v '): v.append(list(map(float,l.split()[1:4])))
  elif l.startswith('f '):
   ids=[int(t.split('/')[0])-1 for t in l.split()[1:]]
   for i in range(1,len(ids)-1): faces.append([ids[0],ids[i],ids[i+1]])
 tri=np.array(v)[np.array(faces)]; tri[:,:,2]-=10
 lo.append(tri.min(1)); hi.append(tri.max(1)); cats.extend([f.stem]*len(tri))
lo=np.concatenate(lo); hi=np.concatenate(hi); mid=(lo+hi)/2
half=np.linalg.norm((hi-lo)/2,axis=1); maxhalf=half.max(); tree=cKDTree(mid)
def distance(q):
 ids=tree.query_ball_point(q,2.0+maxhalf)
 if not ids: return 2.,-1
 d=np.linalg.norm(np.maximum(np.maximum(lo[ids]-q,q-hi[ids]),0),axis=1)
 j=d.argmin(); return float(d[j]),ids[j]
# Continuous survey strips with explicit semicircular turns, no local retracing.
# XY obstacle avoidance uses broad smooth lateral offsets, not grid-cell zigzags.
# Keep regular turns outside the particle field so XY detours cannot collapse
# their radius. Straight survey strips continue through the map at lower depth.
xs=[-12.,-8.75,-5.25,-1.75,1.75,5.25,8.75,12.]; base=[]
for i,x in enumerate(xs):
 start=-12. if i%2==0 else 27.; end=27. if i%2==0 else -12.
 for y in np.linspace(start,end,53)[:-1]: base.append([x,y,-8.6])
 base.append([x,end,-8.6])
 if i+1<len(xs):
  r=(xs[i+1]-x)/2; center=(xs[i+1]+x)/2
  angles=np.linspace(np.pi,0,17) if i%2==0 else np.linspace(np.pi,2*np.pi,17)
  for t in angles[1:]: base.append([center+r*np.cos(t),end+r*np.sin(t),-8.6])
base=np.array(base);base=base[np.r_[True,np.linalg.norm(np.diff(base,axis=0),axis=1)>1e-6]]
edge=np.clip(np.minimum(base[:,1]+12.,27.-base[:,1])/5.,0,1)
turn_taper=3*edge**2-2*edge**3
base[:,2]+=.6*(1-turn_taper)
pts=base.copy()
arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(base,axis=0),axis=1))]
tangent=np.gradient(base[:,:2],axis=0); tangent/=np.linalg.norm(tangent,axis=1)[:,None]
normal=np.column_stack((-tangent[:,1],tangent[:,0],np.zeros(len(base))))
for iteration in range(180):
 result=[distance(q) for q in pts]; distances=np.array([r[0] for r in result])
 index=int(distances.argmin())
 if distances[index]>=1.18: break
 obstacle=result[index][1]
 # Evaluate broad lateral detours and gradual altitude changes against all
 # obstacles; never insert an unchecked vertical transit through particles.
 width=4.5
 weights=np.exp(-.5*((arc-arc[index])/width)**2)
 active=np.flatnonzero(weights>.001)
 options=[normal*(weights*turn_taper)[:,None]*d for d in (-2.4,-1.6,-.8,-.4,-.2,.2,.4,.8,1.6,2.4)]
 for height in (.1,.2,.35):
  delta=np.zeros_like(pts); delta[:,2]=weights*height; options.append(delta)
 best=None; best_cost=np.inf
 for delta in options:
  candidate=pts+delta
  if np.max(np.linalg.norm(candidate[:,:2]-base[:,:2],axis=1))>5.0 or candidate[:,2].max()>-7.7: continue
  ds=distances.copy(); ds[active]=np.array([distance(candidate[j])[0] for j in active])
  penalty=np.maximum(1.3-ds,0)
  offset=candidate-base
  cost=100*np.sum(penalty**2)+.025*np.sum(offset[:,:2]**2)+2.0*np.sum(offset[:,2]**2)
  if cost<best_cost: best_cost=cost; best=candidate
 if best is None: raise RuntimeError('No smooth near-bottom detour')
 pts=best
 print('smooth detour',iteration,'minimum center clearance',distances[index], 'at',pts[index].tolist(),flush=True)
else: raise RuntimeError('Smooth avoidance did not converge')
# Preserve smoothness and reject local reversals before serializing.
v=np.diff(pts[:,:2],axis=0)
turns=np.abs(np.arctan2(v[:-1,0]*v[1:,1]-v[:-1,1]*v[1:,0],(v[:-1]*v[1:]).sum(1)))
assert np.rad2deg(turns).max()<40., 'Unexpected sharp waypoint turn: '+str(np.rad2deg(turns).max())
def interpolate():
 wp=WaypointSet()
 for x,y,z in pts: wp.add_waypoint(Waypoint(x=float(x),y=float(y),z=float(z),max_forward_speed=.5,heading_offset=0.,inertial_frame_id='world'))
 ip=CSInterpolator(); ip.init_waypoints(wp); assert ip.init_interpolator()
 return ip
ip=interpolate(); q=np.array([ip.generate_pos(float(s)) for s in np.linspace(0,1,8001)])
distances=np.array([distance(v)[0] for v in q]); print('Low-altitude route minimum center clearance:',distances.min(),flush=True)
assert distances.min()>1.06, 'cubic curve needs more lateral clearance'
destination=p/'config/obn/inspection_waypoints.yaml'
route=p/'validation/obn/.candidate_waypoints.yaml'
route.write_text('# OBN near-bottom mapping, world ENU. Speed 0.5 m/s; travel-direction heading.\n# Validated only with cubic and the configured start position.\ninertial_frame_id: world\nwaypoints:\n'+''.join('  - {point: [%.4f, %.4f, %.4f], max_forward_speed: 0.5, heading: 0.0, use_fixed_heading: false}\n'%tuple(pt) for pt in pts))
# Validate the exact serialized route, including decimal rounding.
wp=WaypointSet(); assert wp.read_from_file(str(route))
ip=CSInterpolator(); ip.init_waypoints(wp); assert ip.init_interpolator()
q=np.array([ip.generate_pos(float(s)) for s in np.linspace(0,1,24001)])
results=[distance(v) for v in q]; ds=np.array([v[0] for v in results]); j=ds.argmin()
gap=float(np.linalg.norm(np.diff(q,axis=0),axis=1).max()); clearance=float(ds.min()-.45-gap/2)
assert clearance>.55,clearance
yaw=np.unwrap(np.arctan2(np.diff(q[:,1]),np.diff(q[:,0])))
dt=ip._duration/(len(q)-1)
yaw_rate=np.diff(yaw)/dt
max_yaw_rate=float(np.rad2deg(np.abs(yaw_rate)).max())
assert max_yaw_rate<30., 'Reference heading changes too quickly: '+str(max_yaw_rate)
# Match spawn orientation to the exact cubic start tangent. A waypoint secant
# would introduce a heading jump at the first reference sample.
d=ip.generate_pos(1e-8)-ip.generate_pos(0)
start_yaw=float(np.arctan2(d[1],d[0]))
ip._init_rot=np.array([0,0,np.sin(start_yaw/2),np.cos(start_yaw/2)])
quat=np.array([ip.generate_quat(float(s)) for s in np.linspace(0,1,24001)])
assert np.isfinite(quat).all(), 'Invalid reference quaternion'
x,y,z,w=quat.T
reference_yaw=np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))
actual_yaw_rate=float(np.rad2deg(np.abs(np.diff(reference_yaw)/dt)).max())
assert actual_yaw_rate<30.,actual_yaw_rate
route.replace(destination); route=destination
report=dict(waypoints=len(pts),speed_m_s=.5,world_z_range=[float(pts[:,2].min()),float(pts[:,2].max())],start=pts[0].tolist(),start_yaw=start_yaw,path_length_m=float(np.linalg.norm(np.diff(q,axis=0),axis=1).sum()),duration_s=float(ip._duration),triangle_aabbs=len(lo),samples=len(q),min_center_triangle_aabb_distance_m=float(ds.min()),enclosing_rov_radius_m=.45,max_sample_gap_m=gap,conservative_surface_clearance_m=clearance,closest_material=cats[results[j][1]],closest_position=q[j].tolist(),route_sha256=hashlib.sha256(route.read_bytes()).hexdigest(),limitations='Static full cubic path against triangle AABBs of all seven loaded material meshes, including particles. No dynamic PID/current test or arbitrary-position approach validation. Sphere encloses visible model and body/gripper collisions; virtual actuator spheres excluded.')
report.update(max_actual_reference_yaw_rate_deg_s=actual_yaw_rate,interpolator='cubic',end=pts[-1].tolist(),max_reference_yaw_rate_deg_s=max_yaw_rate,max_waypoint_turn_deg=float(np.rad2deg(turns).max()),reversals_over_170_deg=int((turns>np.deg2rad(170)).sum()))
(p/'validation/obn/inspection_route_report.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,(ax,az)=plt.subplots(1,2,figsize=(13,7),gridspec_kw={'width_ratios':[1.2,1]})
for f in sorted((p/'meshes/obn_scene/Category').glob('*.obj')):
 if 'seabed' in f.stem or 'sediment' in f.stem: continue
 v=np.array([list(map(float,l.split()[1:4])) for l in f.open() if l.startswith('v ')])
 stride=max(1,len(v)//3500); ax.scatter(v[::stride,0],v[::stride,1],s=2,alpha=.3,label=f.stem)
ax.plot(q[:,0],q[:,1],color='navy',lw=1,label='cubic route'); ax.scatter(pts[:,0],pts[:,1],s=2,color='navy')
ax.scatter(*pts[0,:2],color='red',s=50,label='Start'); ax.scatter(*pts[-1,:2],color='green',s=50,label='Finish')
for i in range(0,len(pts),45): ax.annotate(str(i+1),pts[i,:2],fontsize=8)
ax.set_aspect('equal'); ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='OBN full-area mapping'); ax.legend(fontsize=7,loc='upper left'); ax.grid(alpha=.2)
arclength=np.r_[0,np.cumsum(np.linalg.norm(np.diff(q,axis=0),axis=1))]
az.plot(arclength,q[:,2]); az.set(xlabel='Distance along route (m)',ylabel='World Z (m)',title='Gradual altitude changes over solid obstacles'); az.grid(alpha=.2); az.set_ylim(-8.9,-7.4); az.ticklabel_format(axis='y',style='plain',useOffset=False)
fig.tight_layout(); fig.savefig(p/'validation/obn/inspection_route_map.png',dpi=150)







