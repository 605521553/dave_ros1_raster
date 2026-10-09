from pathlib import Path
import subprocess, xml.etree.ElementTree as E, json
p=Path(__file__).resolve().parents[2]
old=E.parse(p.parent/'uuv_simulator/uuv_descriptions/urdf/rexrov.gazebo.xacro').find('.//hydrodynamic_model')
report={}
for scene in ('offshore','underwater_wreck','obn'):
 root=E.fromstring(subprocess.check_output(['xacro',str(p/'urdf'/scene/'bygd_rov.xacro'),'namespace:=rexrov']))
 hydro=root.find(".//plugin[@name='uuv_plugin']/link")
 assert hydro.findtext('neutrally_buoyant')=='1'
 assert list(map(float,hydro.findtext('center_of_buoyancy').split()))==[0,0,.3]
 for e in old: assert hydro.findtext('hydrodynamic_model/'+e.tag).split()==e.text.split()
 base=root.find("link[@name='rexrov/base_link']")
 assert float(base.find('inertial/mass').get('value'))==1862.87
 assert base.find('inertial/inertia').attrib==dict(ixx='525.39',ixy='1.44',ixz='33.41',iyy='794.20',iyz='2.6',izz='691.23')
 assert base.find("collision[@name='body_collision']/geometry/box").get('size')=='0.44 0.381 0.336'
 assert base.find("collision[@name='gripper_collision']/geometry/box").get('size')=='0.12 0.10 0.05'
 report[scene]='PASS: Xacro, neutral flag, original CoB lever, original mass/inertia/Fossen coefficients/collisions'
report['limitations']='Configuration checks only; no dynamic ROS/Gazebo test.'
(p/'validation/obn/buoyancy_compatibility_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
