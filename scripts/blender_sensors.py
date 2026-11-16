"""Blender OS1 geometric scanner. Use Blender --python, or bpy Python."""
import sys,json,argparse
from pathlib import Path
import bpy,numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
sys.path.insert(0,str(Path(__file__).resolve().parent))
from sensor_rays import ray_directions,save_scan,head_visibility
def profile():return json.loads(bpy.data.texts['os1_rev8_profile.json'].as_string())
def demo_world():
    if bpy.data.objects.get('SensorTest_Ground'):return
    def cube(name,position,size,color):
        bpy.ops.mesh.primitive_cube_add(size=1,location=position);o=bpy.context.object;o.name=name;o.dimensions=size
        mat=bpy.data.materials.new(name);mat.diffuse_color=(*color,1);mat.use_nodes=True;mat.node_tree.nodes['Principled BSDF'].inputs['Base Color'].default_value=(*color,1);o.data.materials.append(mat)
    cube('SensorTest_Ground',(0,0,-.05),(40,40,.1),(.23,.24,.25));cube('SensorTest_Target',(5,0,1.8),(1,2,2),(.08,.3,.7));cube('SensorTest_Column',(7,2,1),(1,1,2),(.65,.25,.08));bpy.context.view_layer.update()
def scan_frame(config=None):
    config=config or profile();bpy.context.view_layer.update();deps=bpy.context.evaluated_depsgraph_get();vertices=[];faces=[]
    for source in bpy.context.scene.objects:
        if source.type!='MESH' or source.hide_render or source.get('promise_os1_point_display',False) or source.name.startswith('Ouster_OS1_Rev8'):continue
        obj=source.evaluated_get(deps);mesh=obj.to_mesh();mesh.calc_loop_triangles();offset=len(vertices);vertices.extend(obj.matrix_world@v.co for v in mesh.vertices);faces.extend(tuple(offset+i for i in tri.vertices) for tri in mesh.loop_triangles);obj.to_mesh_clear()
    tree=BVHTree.FromPolygons(vertices,faces,all_triangles=True);pose=bpy.data.objects[config['frame']].matrix_world;origin=np.array(pose.translation);rotation=np.array(pose.to_3x3());rays=ray_directions(config);world=rays@rotation.T;ranges=np.full(len(rays),np.nan);p=Vector(origin)
    for i,direction in enumerate(world):
        hit=tree.ray_cast(p,Vector(direction),config['max_range_m'])
        if hit[0] is not None and hit[3]>=config['min_range_m']:ranges[i]=hit[3]
    ranges[~head_visibility(config)]=np.nan
    return ranges,origin,rotation
def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,default=Path('captures/os1'));p.add_argument('--start',type=int,default=1);p.add_argument('--end',type=int,default=1);p.add_argument('--step',type=int,default=3);p.add_argument('--demo-world',action='store_true');p.add_argument('--blend',type=Path)
    a=p.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else sys.argv[1:]);out=a.output.resolve()
    if a.blend:bpy.ops.wm.open_mainfile(filepath=str(a.blend.resolve()))
    if a.demo_world:demo_world()
    out.mkdir(parents=True,exist_ok=True);c=profile()
    for frame in range(a.start,a.end+1,a.step):
        bpy.context.scene.frame_set(frame);ranges,origin,rotation=scan_frame(c);save_scan(out/f'{frame:06}.npz',c,ranges,origin,rotation,(frame-1)/bpy.context.scene.render.fps);print('OS1',frame,len(ranges),int(np.isfinite(ranges).sum()),flush=True)
    (out/'profile.json').write_text(json.dumps(c,indent=2))
if __name__=='__main__':main()
