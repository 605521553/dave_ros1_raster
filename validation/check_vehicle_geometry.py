from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
root=Path('/opt/dave_ws/src/bygd_rov')
verts=[]
for line in (root/'meshes/ROV_R45/rov_lowpoly.obj').open():
    if line.startswith('v '): verts.append([float(x) for x in line.split()[1:4]])
v=np.array(verts); lo=v.min(0); hi=v.max(0)
for scene in ['obn','underwater_wreck']:
    for variant in ['r20','r45']:
        tree=ET.parse('/tmp/%s_%s.urdf'%(scene,variant))
        link=tree.find(".//link[@name='rexrov/base_link']")
        mesh=link.find('visual/geometry/mesh').get('filename')
        assert ('ROV_R45' in mesh)==(variant=='r45')
        assert link.find('inertial/mass').get('value')=='1862.87'
        mount=tree.find(".//joint[@name='rexrov/sensor_mount_joint']/origin")
        if variant=='r45':
            size=np.array([float(x) for x in link.find('collision/geometry/box').get('size').split()])
            assert np.all(lo>=-size/2) and np.all(hi<=size/2)
            assert float(mount.get('xyz').split()[0])>hi[0]
        else: assert len(link.findall('collision'))==2
    print(scene,'R20/R45 geometry, collision and retained mass PASS')
for line in (root/'meshes/ROV_R45/rov_lowpoly.mtl').read_text().splitlines():
    if line.startswith('map_'): assert (root/'meshes/ROV_R45'/line.split()[-1]).exists()
print('R45 bounds',lo.tolist(),hi.tolist(),'MTL textures PASS')
