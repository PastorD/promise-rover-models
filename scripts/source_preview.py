import bpy, pathlib, math
from mathutils import Vector
ROOT=pathlib.Path(__file__).resolve().parents[1]
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'source_assets/promise_reference.blend'))
s=bpy.context.scene
s.render.engine='CYCLES'; s.cycles.samples=12; s.cycles.use_denoising=True
s.render.resolution_x=1300;s.render.resolution_y=1000;s.render.resolution_percentage=100
s.world=bpy.data.worlds.new('Studio');s.world.use_nodes=True;s.world.node_tree.nodes['Background'].inputs[0].default_value=(.35,.35,.35,1);s.world.node_tree.nodes['Background'].inputs[1].default_value=.8
for pos,power,size in [((2,-5,6),1600,5),((-4,-1,4),1000,4),((1,4,5),1800,3)]:
    bpy.ops.object.light_add(type='AREA',location=pos);o=bpy.context.object;o.data.energy=power;o.data.shape='DISK';o.data.size=size;o.rotation_euler=(Vector((0,0,1))-o.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(4,-6,3.6));cam=bpy.context.object;cam.rotation_euler=(Vector((0,0,1))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=4.5;s.camera=cam
s.render.film_transparent=True;s.render.filepath=str(ROOT/'references/promise_reference_render.png');bpy.ops.render.render(write_still=True)
