from pathlib import Path
import xml.etree.ElementTree as ET
import math
p=Path(__file__).resolve().parents[2]
report=[]
for f in sorted((p/'meshes/obn_scene/Material').glob('*.obj')):
    counts={'v':0,'vt':0,'vn':0}; faces=[]; used=set(); libraries=[]
    lo=[math.inf]*3; hi=[-math.inf]*3
    for line in f.read_text().splitlines():
        bits=line.split()
        if not bits: continue
        kind=bits[0]
        if kind in counts:
            counts[kind]+=1
            if kind=='v':
                xyz=list(map(float,bits[1:4])); assert all(math.isfinite(v) for v in xyz)
                lo=[min(a,b) for a,b in zip(lo,xyz)]; hi=[max(a,b) for a,b in zip(hi,xyz)]
        elif kind=='f':
            assert len(bits)>=4
            for token in bits[1:]:
                for key,index in zip(('v','vt','vn'),token.split('/')):
                    if index:
                        index=int(index); assert index!=0
                        if index<0: assert -index<=counts[key]
                        else: faces.append((key,index))
        elif kind=='mtllib': libraries.extend(bits[1:])
        elif kind=='usemtl': used.add(bits[1])
    assert all(i<=counts[key] for key,i in faces)
    declared=set()
    for lib in libraries:
        path=f.parent/lib; assert path.is_file(),path
        declared.update(line.split()[1] for line in path.read_text().splitlines() if line.startswith('newmtl '))
    assert used<=declared,(f,used,declared)
    nfaces=sum(1 for line in f.open() if line.startswith('f '))
    report.append(f'{f.name}: vertices={counts["v"]}, faces={nfaces}, materials={sorted(used)}, min={lo}, max={hi}')
for folder,pattern in [('launch','*.launch'),('urdf','*.xacro'),('worlds','*.world')]:
    for f in (p/folder/'obn').glob(pattern): ET.parse(f)
world=ET.parse(p/'worlds/obn/obn.world').find('world')
roots={model.get('name') for model in world.findall('model') if model.get('name').startswith('obn_')}
sensor=ET.parse(p/'urdf/obn/sensors.xacro')
fiducials={e.text for e in sensor.iter('fiducial')}
csv={line.split(',')[0] for line in (p/'config/obn/reflectivity.csv').read_text().splitlines()[3:]}
assert roots==fiducials==csv
for uri in world.iter('uri'):
    if uri.text.startswith('model://bygd_rov/'): assert (p/uri.text.removeprefix('model://bygd_rov/')).is_file()
for name in ('sensors.yaml','rov_model.yaml'):
    assert (p/'config/obn'/name).read_text()==(p/'config/underwater_wreck'/name).read_text().replace('underwater_wreck','obn')
report.append('PASS: OBJ indices, finite coordinates, MTL references, XML syntax, world mesh paths, seven model/fiducial/CSV names and unchanged sensor/model settings.')
report.append('Runtime Gazebo rendering, Xacro expansion and sonar output have not been verified.')
(p/'validation/obn/static_validation.txt').write_text('\n'.join(report)+'\n')
print('\n'.join(report))

