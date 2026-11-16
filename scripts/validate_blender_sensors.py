"""Check rebuilt Blender sensors, wheel animation, and VisionSIM RGB/depth."""
import json, math, importlib.metadata
from pathlib import Path
import bpy, numpy as np, OpenEXR
from mathutils import Vector
from PIL import Image
from blender_sensors import profile, demo_world, scan_frame
from sensor_rays import ray_directions

ROOT=Path(__file__).resolve().parents[1]
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'blender/promise_wheel_animation.blend'))
scene=bpy.context.scene;scene.frame_set(1);bpy.context.view_layer.update()
c=profile();sensor=bpy.data.objects[c['frame']]
names=['stereo_left','stereo_right','dashcam_left','dashcam_right']
for name in names:
    assert bpy.data.objects[name].type=='CAMERA'
    assert bpy.data.objects[name].parent.name=='head_pitch_link'
assert sensor.parent.name=='os1_link'
assert sensor.parent.parent.name=='head_pitch_link'
before=np.array(sensor.matrix_world)
head=bpy.data.objects['head_pitch_link'];head.rotation_euler[1]=.2;bpy.context.view_layer.update()
assert not np.allclose(before,np.array(sensor.matrix_world))
head.rotation_euler[1]=0;bpy.context.view_layer.update()
wheels=[o for o in bpy.data.objects if o.name.endswith('_wheel') and o.type=='EMPTY'];assert len(wheels)==6
scene.frame_set(1);initial={o.name:np.array(o.matrix_local) for o in wheels}
scene.frame_set(121)
assert all(not np.allclose(initial[o.name],np.array(o.matrix_local)) for o in wheels)
scene.frame_set(1);demo_world();ranges,origin,rotation=scan_frame(c)
index=100*c['columns'];expected=(4.5-origin[0])/(ray_directions(c)[index]@rotation.T)[0]
assert abs(ranges[index]-expected)<1e-5
captures={}
for name in names:
    folder=ROOT/'previews/visionsim_smoke'/name
    png=next((folder/'frames').rglob('*.png'));exr=next((folder/'depths').rglob('*.exr'))
    rgb=np.array(Image.open(png));assert rgb.shape[:2]==(180,320) and rgb.std()>1
    with OpenEXR.File(str(exr)) as f:
        channel=next(iter(f.channels().values())).pixels
        value=float(channel[90,160] if channel.ndim==2 else channel[90,160,0])
    assert abs(value-3.665)<1e-4
    captures[name]={'resolution':[320,180],'target_depth_m':value}
report={'status':'PASS','blender':bpy.app.version_string,'visionsim':importlib.metadata.version('visionsim'),'animated_wheels':6,'turret_parenting':'PASS','lidar_rays':len(ranges),'lidar_target_expected_m':expected,'lidar_target_measured_m':float(ranges[index]),'visionsim_cameras':captures}
(ROOT/'references/blender_validation.json').write_text(json.dumps(report,indent=2)+'\n')
# Preview only: these view changes are never saved to the animation .blend.
for o in list(bpy.data.objects):
    if o.name.startswith('SensorTest_'):bpy.data.objects.remove(o,do_unlink=True)
bpy.ops.object.camera_add(location=(1.9,-.85,2.35));camera=bpy.context.object
camera.rotation_euler=(Vector((.69,-.164,1.98))-camera.location).to_track_quat('-Z','Y').to_euler();camera.data.type='ORTHO';camera.data.ortho_scale=.72;scene.camera=camera
scene.render.resolution_x=1200;scene.render.resolution_y=900;scene.render.resolution_percentage=100;scene.cycles.samples=24;scene.render.filepath=str(ROOT/'previews/turret_sensors.png');scene.render.film_transparent=True
bpy.ops.render.render(write_still=True);print(json.dumps(report,indent=2))
