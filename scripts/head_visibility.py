"""Static shadow cast by the rigid head enclosure, excluding the lidar's case."""
import base64, zlib
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from sensor_rays import ray_directions


def calculate_visibility(objects, origin, rotation, profile):
    vertices=[];faces=[]
    for obj in objects:
        if obj.type!='MESH':continue
        mesh=obj.data;mesh.calc_loop_triangles();offset=len(vertices)
        vertices.extend(obj.matrix_world@v.co for v in mesh.vertices)
        faces.extend(tuple(offset+i for i in t.vertices) for t in mesh.loop_triangles)
    tree=BVHTree.FromPolygons(vertices,faces,all_triangles=True)
    rays=ray_directions(profile)@np.asarray(rotation).T;p=Vector(origin)
    visible=np.array([tree.ray_cast(p,Vector(v),1.0)[0] is None for v in rays])
    grid=visible.reshape(profile['channels'],profile['columns']);step=360/profile['columns']
    full_columns=np.all(grid,axis=0);half=0
    while half<profile['columns']//4 and full_columns[half+1] and full_columns[-half-1]:half+=1
    assert np.all(grid[:,0]),'Front aperture blocks central forward rays'
    assert not np.any(grid[:,profile['columns']//2]),'Rear wall must block backward rays'
    report={'total_rays':int(visible.size),'head_clear_rays':int(visible.sum()),'head_blocked_rays':int((~visible).sum()),'forward_center_all_128_channels_clear':bool(grid[:,0].all()),'rear_center_all_channels_blocked':bool((~grid[:,profile['columns']//2]).all()),'fully_clear_horizontal_sector_deg':[-half*step,half*step],'native_vertical_fov_deg':profile['vertical_fov_deg'],'scope':'Only geometry rigidly attached to head_pitch_link; excludes OS1 case. Other rover/environment occlusion is dynamic.'}
    encoded=base64.b64encode(zlib.compress(np.packbits(visible,bitorder='little').tobytes(),9)).decode('ascii')
    return encoded,report
