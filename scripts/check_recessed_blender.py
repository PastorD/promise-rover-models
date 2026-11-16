"""Verify fit, tilt and the actual aperture ray visibility in Blender."""
from pathlib import Path
import json, math
import bpy,numpy as np
from head_visibility import calculate_visibility

ROOT=Path(__file__).resolve().parents[1]
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'blender/promise_wheel_animation.blend'))
bpy.context.view_layer.update()
profile=json.loads(bpy.data.texts['os1_rev8_profile.json'].as_string())
lidar=bpy.data.objects['os1_link'];frame=bpy.data.objects['os1_lidar_frame']
angle=math.degrees(math.atan2(-frame.matrix_world.to_3x3()[2][0],frame.matrix_world.to_3x3()[0][0]))
assert abs(angle-22.5)<1e-5
case=next(o for o in bpy.data.objects if o.parent==lidar and o.type=='MESH')
v=np.array([case.matrix_world@vertex.co for vertex in case.data.vertices])
lo,hi=np.array(profile['enclosure']['outer_bounds_m'])
assert np.all(v.min(0)>lo) and np.all(v.max(0)<hi)
assert not bpy.data.objects.get('OS1_turret_mount')
head=bpy.data.objects['head_pitch_link'];objects=[o for o in bpy.data.objects if o.parent==head and o.type=='MESH']
encoded,report=calculate_visibility(objects,np.array(frame.matrix_world.translation),np.array(frame.matrix_world.to_3x3()),profile)
report.update(status='PASS',mount_pitch_down_deg=angle,optical_origin_m=list(frame.matrix_world.translation),case_bounds_m=[v.min(0).tolist(),v.max(0).tolist()],housing_front_recess_mm=float((hi[0]-v[:,0].max())*1000),front_aperture_mm=[150,92])
(ROOT/'references/recessed_mount_validation.json').write_text(json.dumps(report,indent=2)+'\n')
(ROOT/'data/head_visibility_mask.json').write_text(json.dumps({'head_visible_mask_encoding':'zlib+packbits-little-base64','head_visible_mask_b64':encoded,'head_visibility':report},indent=2)+'\n')
print(json.dumps(report,indent=2),flush=True)
