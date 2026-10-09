from pathlib import Path
import json
p=Path(__file__).resolve().parents[2]
# Reuse the mesh-loading and distance helpers without regenerating the route.
planner=p/'validation/obn/plan_inspection_route.py'
exec(compile(planner.read_text(encoding='utf-8-sig').split('# Continuous survey strips')[0],str(planner),'exec'))
wp=WaypointSet(); assert wp.read_from_file(str(p/'config/obn/inspection_waypoints.yaml'))
ip=CSInterpolator(); ip.init_waypoints(wp); assert ip.init_interpolator()
s=np.linspace(0,1,6001); q=np.array([ip.generate_pos(float(t)) for t in s])
yaw=np.arctan2(np.gradient(q[:,1]),np.gradient(q[:,0])); yaw[0]=float(np.arctan2(q[1,1]-q[0,1],q[1,0]-q[0,0]))
cos=np.cos(yaw); sin=np.sin(yaw)
actuators=np.array([[-.890895,.334385,.528822],[-.890895,-.334385,.528822],[.890895,.334385,.528822],[.890895,-.334385,.528822],[-.412125,.505415,.129],[-.412125,-.505415,.129],[.412125,.505415,.129],[.412125,-.505415,.129]])
mins=[]; gaps=[]
for x,y,z in actuators:
 centers=q+np.column_stack((cos*x-sin*y,sin*x+cos*y,np.full(len(q),z)))
 minimum=.8
 for center in centers:
  ids=tree.query_ball_point(center,.8+maxhalf)
  if ids:
   d=np.linalg.norm(np.maximum(np.maximum(lo[ids]-center,center-hi[ids]),0),axis=1).min()
   minimum=min(minimum,float(d))
 mins.append(minimum); gaps.append(float(np.linalg.norm(np.diff(centers,axis=0),axis=1).max()))
# Test joint RGB/sonar field of view at the 30 regular nodes and the south node.
# This is a geometric opportunity check, not an occlusion/rendering guarantee.
targets=[[x,y,-9.6] for y in (1,5,9,13,17,21) for x in (-7,-3.5,0,3.5,7)]
node_vertices=np.array([list(map(float,l.split()[1:4])) for l in (p/'meshes/obn_scene/Category/03_obn_nodes.obj').open() if l.startswith('v ')])
south=node_vertices[node_vertices[:,1]<-3]; south_center=(south.min(0)+south.max(0))/2
targets.append([float(south_center[0]),float(south_center[1]),float(south_center[2]-10)]); targets.append([4,-3,-9.3])
sensor=q+np.column_stack((.25*cos,.25*sin,np.full(len(q),.1)))
coverage=[]
for target in targets:
 d=np.array(target)-sensor
 forward=d[:,0]*cos+d[:,1]*sin; lateral=-d[:,0]*sin+d[:,1]*cos
 down=-d[:,2]; viewforward=np.cos(.35)*forward+np.sin(.35)*down
 viewdown=-np.sin(.35)*forward+np.cos(.35)*down
 hf=np.arctan2(lateral,viewforward); vf=np.arctan2(viewdown,viewforward)
 dist=np.linalg.norm(d,axis=1)
 visible=(viewforward>0)&(np.abs(hf)<.525)&(np.abs(vf)<np.deg2rad(6))&(dist<10)&(dist>.1)
 coverage.append(dict(target=target,joint_fov_samples=int(visible.sum()),nearest_path_distance_m=float(np.linalg.norm(np.array(target)-q,axis=1).min())))
# Check actual recovery basket surface rather than only its interior center.
basket=np.array([list(map(float,l.split()[1:4])) for l in (p/'meshes/obn_scene/Category/04_recovery_basket.obj').open() if l.startswith('v ')])
basket=basket[::max(1,len(basket)//2000)].copy(); basket[:,2]-=10
basket_visible=[]
for target in basket:
 d=target-sensor; forward=d[:,0]*cos+d[:,1]*sin; lateral=-d[:,0]*sin+d[:,1]*cos; down=-d[:,2]
 viewforward=np.cos(.35)*forward+np.sin(.35)*down; viewdown=-np.sin(.35)*forward+np.cos(.35)*down
 visible=(viewforward>0)&(np.abs(np.arctan2(lateral,viewforward))<.525)&(np.abs(np.arctan2(viewdown,viewforward))<np.deg2rad(6))&(np.linalg.norm(d,axis=1)<10)
 if visible.any(): basket_visible.append(dict(target=target.tolist(),joint_fov_samples=int(visible.sum())))
print('Basket surface visible representatives:',len(basket_visible),flush=True)
report=dict(samples=len(q),virtual_thruster_min_triangle_aabb_distances_m=mins,max_thruster_sample_gap_m=gaps,virtual_thruster_radius_m=.000001,all_targets_have_joint_fov=all(t['joint_fov_samples']>0 for t in coverage[:-1]) and bool(basket_visible),targets=coverage,limitations='Nominal horizontal travel attitude; sampled virtual-thruster clearance and joint field of view only. No ray occlusion or dynamic tracking verification.')
report['basket_surface_tested_points']=len(basket); report['basket_surface_visible_points']=len(basket_visible); report['basket_surface_example']=basket_visible[0] if basket_visible else None
body=json.loads((p/'validation/obn/inspection_route_report.json').read_text())
actuator_radius=float(np.linalg.norm(actuators,axis=1).max())+.000001
report['all_attitude_actuator_enclosing_radius_m']=actuator_radius
report['all_attitude_actuator_conservative_clearance_m']=body['min_center_triangle_aabb_distance_m']-actuator_radius-body['max_sample_gap_m']/2
print(json.dumps({k:v for k,v in report.items() if k!='targets'},indent=2))
(p/'validation/obn/inspection_coverage_report.json').write_text(json.dumps(report,indent=2)+'\n')
# An enclosing sphere provides a bound independent of heading discontinuities.
assert report['all_attitude_actuator_conservative_clearance_m']>.05
assert report['all_targets_have_joint_fov']


