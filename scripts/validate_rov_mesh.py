"""Round-trip OBJ import, geometry deviation and sensor self-occlusion checks."""
from pathlib import Path
import bpy, json, math
from mathutils import Matrix
from prepare_rov_mesh import bounds,bvh,visibility,render,OUT

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=str(OUT/'rov_lowpoly.obj'),forward_axis='Y',up_axis='Z')
reduced=[o for o in bpy.context.scene.objects if o.type=='MESH']
rtbounds=bounds(reduced)
reduced_tree=bvh(reduced)
vis=visibility(reduced_tree)
render(reduced,'obj_reimport.png')
reduced_vertices=[o.matrix_world@v.co for o in reduced for v in o.data.vertices]
bpy.ops.wm.read_factory_settings(use_empty=True)
source=OUT/'source/20.fbx'
if not source.exists():source=OUT/'20.fbx'
bpy.ops.import_scene.fbx(filepath=str(source))
original=[o for o in bpy.context.scene.objects if o.type=='MESH']
rot=Matrix.Rotation(math.pi/2,4,'Z')
for o in original:
    transform=rot@o.matrix_world.copy();o.parent=None;o.data=o.data.copy();o.data.transform(transform);o.matrix_world=Matrix.Identity(4)
original_tree=bvh(original)
original_vertices=[o.matrix_world@v.co for o in original for v in o.data.vertices]
def deviation(vertices,tree):
    distances=[]
    for v in vertices[::max(1,len(vertices)//5000)]:
        _,_,_,d=tree.find_nearest(v)
        if d is not None:distances.append(d)
    distances.sort()
    return dict(samples=len(distances),mean_m=sum(distances)/len(distances),p95_m=distances[int(.95*(len(distances)-1))],max_m=max(distances))
report=dict(obj_reimport_bounds_m=rtbounds,sensor_visibility=vis,original_to_reduced=deviation(original_vertices,reduced_tree),reduced_to_original=deviation(reduced_vertices,original_tree))
(OUT/'validation_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
