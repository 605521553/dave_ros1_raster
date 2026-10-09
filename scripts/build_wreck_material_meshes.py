#!/usr/bin/env python3
"""Build four material models from original category meshes without changing geometry."""
from pathlib import Path
from collections import Counter
import json
from merge_wreck_meshes import parse, CATEGORIES, PACKAGE
MAPPING={'rock':'stone','shell':'stone','growth':'wood','rust':'metal','steel':'metal','sand':'soil'}
COLORS={'stone':(.22,.23,.18),'wood':(.22,.12,.055),'metal':(.19,.085,.027),'soil':(.30,.26,.18)}
def main():
 source=PACKAGE/'meshes/underwater_wreck_scene'; dest=source/'materials4'; dest.mkdir(exist_ok=True)
 data={k:dict(records=[],faces=[],maps={},counts=dict(v=0,vt=0,vn=0)) for k in COLORS}
 expected={k:Counter() for k in COLORS}; source_faces=0; assignments={}
 for category in CATEGORIES:
  records,groups,_,_,_=parse(source/(category+'.obj'))
  arrays={k:[line for line in records if line.split()[0]==k] for k in ('v','vt','vn')}
  assignments[category]={}
  for (material,smoothing),faces in groups.items():
   target=MAPPING[material]; d=data[target]; assignments[category][target]=assignments[category].get(target,0)+len(faces)
   for face in faces:
    corners=[]; geometry=[]
    for token in face.split()[1:]:
     refs=token.split('/'); mapped=[]; values=[]
     for i,key in enumerate(('v','vt','vn')):
      if i>=len(refs): break
      if not refs[i]: mapped.append(''); values.append(None); continue
      old=int(refs[i]); index=(category,key,old)
      if index not in d['maps']:
       d['counts'][key]+=1; d['maps'][index]=d['counts'][key]; d['records'].append(arrays[key][old-1])
      mapped.append(str(d['maps'][index])); values.append(arrays[key][old-1])
     corners.append('/'.join(mapped)); geometry.append(tuple(values))
    d['faces'].append((smoothing,'f '+' '.join(corners))); expected[target][(smoothing,tuple(geometry))]+=1; source_faces+=1
 report={'material_mapping':MAPPING,'category_face_assignments':assignments,'source_faces':source_faces,'outputs':{}}
 for target,d in data.items():
  lines=['# Four-material derived asset: coordinates, normals, UVs and faces preserved.','mtllib '+target+'.mtl','o '+target]+d['records']+['usemtl '+target]
  prev=None
  for smoothing,face in d['faces']:
   if smoothing!=prev: lines.append('s '+smoothing); prev=smoothing
   lines.append(face)
  f=dest/(target+'.obj'); f.write_text('\n'.join(lines)+'\n')
  kd=' '.join(map(str,COLORS[target])); (dest/(target+'.mtl')).write_text('newmtl '+target+'\nKa 0.25 0.25 0.25\nKd '+kd+'\nKs 0.15 0.15 0.15\nNs 10\nd 1\nillum 2\n')
  records,groups,_,objects,counts=parse(f); arrays={k:[line for line in records if line.split()[0]==k] for k in ('v','vt','vn')}; actual=Counter()
  for (_,smoothing),faces in groups.items():
   for face in faces:
    geometry=[]
    for token in face.split()[1:]:
     refs=token.split('/'); geometry.append(tuple(arrays[key][int(refs[i])-1] if refs[i] else None for i,key in enumerate(('v','vt','vn')) if i<len(refs)))
    actual[(smoothing,tuple(geometry))]+=1
  assert actual==expected[target],target
  report['outputs'][target]={'objects':objects,'vertices':counts['v'],'faces':len(d['faces']),'geometry_preserved':True}
 assert sum(v['faces'] for v in report['outputs'].values())==source_faces
 (PACKAGE/'validation/underwater_wreck/material4_mesh_report.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
if __name__=='__main__': main()
