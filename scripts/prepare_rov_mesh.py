"""Convert the original FBX into a small, ROS-axis OBJ using D:/PyBlender bpy.
Run with D:/PyBlender/.venv/Scripts/python.exe. No upstream packages are edited.
The CAD origin and metre scale are retained; rotate +90 degrees around Z so
original -Y (window/gripper end) becomes ROS +X. Camera geometry is not added.
"""
import argparse, hashlib, json, math
from pathlib import Path
import bpy, bmesh
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'meshes/ROV'

def triangles(objects):
    return sum(len(p.vertices)-2 for o in objects for p in o.data.polygons)

def bounds(objects):
    lo=[float('inf')]*3;hi=[-float('inf')]*3
    for o in objects:
        for v in o.data.vertices:
            p=o.matrix_world@v.co
            for i in range(3):lo[i]=min(lo[i],p[i]);hi[i]=max(hi[i],p[i])
    return {'min':lo,'max':hi,'size':[b-a for a,b in zip(lo,hi)]}

def bvh(objects):
    verts=[];faces=[]
    for o in objects:
        offset=len(verts)
        verts.extend(o.matrix_world@v.co for v in o.data.vertices)
        faces.extend(tuple(offset+i for i in p.vertices) for p in o.data.polygons)
    return BVHTree.FromPolygons(verts,faces)

def visibility(tree):
    origin=Vector((.25,0,.10));rot=Matrix.Rotation(.15,3,'Y')
    results={}
    # Full RGB perspective frustum, and full sonar angular coverage.
    for label,nx,ny,hfov,vfov in [('rgbd',65,49,1.05,None),('sonar',131,25,math.radians(130),math.radians(12))]:
        hits=0;nearest=None
        for j in range(ny):
            for i in range(nx):
                yy=(2*i/(nx-1)-1)*math.tan(hfov/2)
                zz=(2*j/(ny-1)-1)*(math.tan(vfov/2) if vfov else math.tan(hfov/2)*480/640)
                direction=(rot@Vector((1,yy,zz))).normalized()
                hit,_,_,distance=tree.ray_cast(origin,direction,20)
                if hit is not None:
                    hits+=1;nearest=distance if nearest is None else min(nearest,distance)
        results[label]={'rays':nx*ny,'self_hits':hits,'nearest_hit_m':nearest}
    return results

def render(objects,name,with_mount=False):
    s=bpy.context.scene;s.render.engine='BLENDER_WORKBENCH'
    s.render.resolution_x=1100;s.render.resolution_y=900;s.render.resolution_percentage=100
    s.display.shading.light='STUDIO';s.display.shading.color_type='MATERIAL'
    s.display.shading.show_shadows=True;s.display.shading.show_cavity=True;s.display.shading.cavity_type='BOTH'
    s.display.shading.background_type='WORLD'
    if not s.world:s.world=bpy.data.worlds.new('PreviewWorld')
    s.world.color=(.18,.18,.18)
    if with_mount:
        mat=bpy.data.materials.new('SensorPositionMarker');mat.diffuse_color=(1,.12,.02,1)
        bpy.ops.mesh.primitive_uv_sphere_add(segments=20,ring_count=12,radius=.009,location=(.25,0,.10))
        marker=bpy.context.object;marker.name='PREVIEW_ONLY_SENSOR_ORIGIN';marker.data.materials.append(mat)
        # Short optical-axis cylinder, orange marker only, never exported.
        a=Vector((.25,0,.10));direction=Matrix.Rotation(.15,3,'Y')@Vector((1,0,0));length=.09
        bpy.ops.mesh.primitive_cylinder_add(vertices=12,radius=.002,depth=length,location=a+direction*length/2)
        rod=bpy.context.object;rod.rotation_euler=direction.to_track_quat('Z','Y').to_euler();rod.data.materials.append(mat)
    bpy.ops.object.camera_add(location=(.85,.65,.48));cam=bpy.context.object
    cam.rotation_euler=(Vector((.025,0,-.035))-cam.location).to_track_quat('-Z','Y').to_euler()
    cam.data.type='ORTHO';cam.data.ortho_scale=.73;s.camera=cam
    (OUT/'preview').mkdir(exist_ok=True)
    s.render.filepath=str(OUT/'preview'/name);bpy.ops.render.render(write_still=True)
    bpy.data.objects.remove(cam,do_unlink=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--target-triangles',type=int,default=50000);args=ap.parse_args()
    source=OUT/'source/20.fbx'
    if not source.exists():source=OUT/'20.fbx'
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=str(source))
    objects=[o for o in bpy.context.scene.objects if o.type=='MESH']
    original={'objects':len(objects),'triangles':triangles(objects),'bounds_imported_m':bounds(objects),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
    rot=Matrix.Rotation(math.pi/2,4,'Z')
    for idx,o in enumerate(objects):
        transform=rot@o.matrix_world.copy();o.parent=None;o.data=o.data.copy();o.data.transform(transform);o.matrix_world=Matrix.Identity(4);o.name='rov_part_%03d'%idx
        # CAD carries split normals; decimation regenerates them from geometry.
        if o.data.has_custom_normals:
            bpy.context.view_layer.objects.active=o
            bpy.ops.mesh.customdata_custom_splitnormals_clear()
    for m in bpy.data.materials:m.name='rov_body_material'
    # FBX CAD faces contain duplicated vertices at patch boundaries. Welding is
    # essential before decimation; otherwise independent patches tear apart.
    for o in objects:
        bm=bmesh.new();bm.from_mesh(o.data)
        bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-6)
        bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
        bm.to_mesh(o.data);bm.free();o.data.update()
    original['bounds_ros_m']=bounds(objects)
    print('Input',original,flush=True)
    render(objects,'original_ros_axes.png')
    original['visibility']=visibility(bvh(objects))
    bpy.ops.object.select_all(action='DESELECT')
    # Triangulate welded CAD geometry before collapse. Avoid planar dissolve:
    # CAD patches with holes can become invalid ngons during aggressive collapse.
    for o in objects:
        bm=bmesh.new();bm.from_mesh(o.data)
        bmesh.ops.triangulate(bm,faces=list(bm.faces))
        bm.to_mesh(o.data);bm.free();o.data.update()
    counts=[sum(len(p.vertices)-2 for p in o.data.polygons) for o in objects]
    # Allocate by physical surface area, not original CAD tessellation density.
    # Otherwise screw threads consume the budget and the main frame loses shape.
    minimum=[min(16,n) for n in counts]
    targets=list(map(float,minimum))
    areas=[sum(p.area for p in o.data.polygons) for o in objects]
    budget=max(0,args.target_triangles-sum(minimum))
    active={i for i,n in enumerate(counts) if n>targets[i]}
    while budget>1e-6 and active:
        weight=sum(areas[i] for i in active)
        shares={i:budget*(areas[i]/weight if weight else 1/len(active)) for i in active}
        used=0
        for i in active:
            addition=min(shares[i],counts[i]-targets[i]);targets[i]+=addition;used+=addition
        budget-=used
        active={i for i in active if counts[i]-targets[i]>1e-6}
    for o,n,target in zip(objects,counts,targets):
        if n>target and len(o.data.polygons)>3:
            bpy.context.view_layer.objects.active=o;o.select_set(True)
            mod=o.modifiers.new('Budget_decimation','DECIMATE');mod.ratio=max(.001,min(1,target/n));mod.use_collapse_triangulate=True
            bpy.ops.object.modifier_apply(modifier=mod.name);o.select_set(False)
    print('Reduced triangles',triangles(objects),flush=True)
    # Join components only after per-part simplification. This retains small parts.
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects:o.select_set(True)
    bpy.context.view_layer.objects.active=objects[0];bpy.ops.object.join();robot=bpy.context.object;robot.name='bygd_rov_visual'
    # Keep CAD hard edges while smoothly shading curved regions.
    bm=bmesh.new();bm.from_mesh(robot.data)
    for f in bm.faces:f.smooth=True
    for e in bm.edges:
        e.smooth=e.is_manifold and e.calc_face_angle(0)<math.radians(30)
    bm.to_mesh(robot.data);bm.free();robot.data.update()
    output=OUT/'rov_lowpoly.obj'
    bpy.ops.wm.obj_export(filepath=str(output),export_selected_objects=True,forward_axis='Y',up_axis='Z',export_materials=True,export_normals=True,export_uv=False,export_triangulated_mesh=True,path_mode='RELATIVE')
    # Discard loose CAD construction edges; only triangle surfaces are needed.
    text=output.read_text()
    output.write_text(''.join(line for line in text.splitlines(keepends=True) if not line.startswith('l ')))
    stats={'original':original,'output':{'triangles':triangles([robot]),'bounds_ros_m':bounds([robot]),'obj_bytes':output.stat().st_size,'mtl_bytes':output.with_suffix('.mtl').stat().st_size,'visibility':visibility(bvh([robot]))},'coordinate_mapping':'ROS x=-imported_y, ROS y=imported_x, ROS z=imported_z; CAD origin retained','mount':{'xyz_m':[.25,0,.10],'rpy_rad':[0,.15,0]},'target_triangles':args.target_triangles}
    (OUT/'processing_report.json').write_text(json.dumps(stats,indent=2),encoding='utf-8')
    render([robot],'lowpoly_ros_axes.png');render([robot],'sensor_mount.png',True)
    print(json.dumps(stats,indent=2),flush=True)

if __name__=='__main__':main()
