"""Cross-format mount orientation and aperture-output regression checks."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util, json, struct, sys
import numpy as np, yourdfpy
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
from sensor_rays import ray_directions,head_visibility,valid_sensor_points

ROOT=Path(__file__).resolve().parents[1];P=ROOT/'ros2/promise_description'
spec=json.loads((ROOT/'data/robot_spec.json').read_text());c=spec['lidar']
robot=yourdfpy.URDF.load(str(P/'urdf/promise.urdf'),load_meshes=False)
expected=robot.get_transform(c['frame']).copy();expected[:3,3]+=spec['links']['base_link']['anchor']
pitch=np.degrees(np.arctan2(-expected[2,0],expected[0,0]));assert abs(pitch-22.5)<1e-6

raw=(ROOT/'glb/promise.glb').read_bytes();size=struct.unpack_from('<I',raw,12)[0];g=json.loads(raw[20:20+size]);parents={child:i for i,n in enumerate(g['nodes']) for child in n.get('children',[])}
def transform(index):
    n=g['nodes'][index]
    if 'matrix' in n:m=np.array(n['matrix']).reshape(4,4,order='F')
    else:
        m=np.eye(4);m[:3,:3]=Rotation.from_quat(n.get('rotation',[0,0,0,1])).as_matrix()@np.diag(n.get('scale',[1,1,1]));m[:3,3]=n.get('translation',[0,0,0])
    return transform(parents[index])@m if index in parents else m
index=next(i for i,n in enumerate(g['nodes']) if n.get('name')==c['frame'])
conversion=np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1.]])
glb_error=float(np.max(np.abs(transform(index)-conversion@expected@conversion.T)));assert glb_error<2e-6,glb_error
assert json.loads(g['nodes'][index]['extras']['profile_json'])['head_visible_mask_b64']==c['head_visible_mask_b64']

stage=Usd.Stage.Open(str(ROOT/'usd/promise_scene.usda'));prim=stage.GetPrimAtPath('/World/PROMISE/os1_link/'+c['frame']+'/sensor')
usd_transform=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
usd_expected=expected.copy();usd_expected[2,3]+=.02
usd_error=float(np.max(np.abs(usd_transform-usd_expected)));assert usd_error<2e-6,usd_error
joint=UsdPhysics.FixedJoint(stage.GetPrimAtPath('/World/PROMISE/Joints/os1_mount_joint'));q=joint.GetLocalRot0Attr().Get();rot=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
assert np.max(np.abs(rot-expected[:3,:3]))<2e-6
assert prim.GetCustomData()['headVisibleMaskBase64']==c['head_visible_mask_b64']

# All nominal rays on a virtual 10 m sphere: only physically open rays survive.
visible=head_visibility(c);points=ray_directions(c)*10
assert np.array_equal(valid_sensor_points(c,points),visible)
module_spec=importlib.util.spec_from_file_location('lidar_filter',P/'scripts/lidar_filter.py');module=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(module)
for endian in ['<','>']:
    h,w=c['channels'],c['columns'];point_step=20;row_step=w*point_step+8
    dtype=np.dtype({'names':['x','y','z','intensity'],'formats':[endian+'f4']*4,'offsets':[0,4,8,12],'itemsize':point_step})
    data=bytearray([0xee])*(h*row_step);a=np.ndarray((h,w),dtype=dtype,buffer=data,strides=(row_step,point_step))
    for i,n in enumerate(['x','y','z']):a[n]=points[:,i].reshape(h,w)
    a['intensity']=42
    fields=[SimpleNamespace(name=n,offset=4*i,count=1,datatype=7) for i,n in enumerate(['x','y','z','intensity'])]
    msg=SimpleNamespace(height=h,width=w,point_step=point_step,row_step=row_step,fields=fields,is_bigendian=endian=='>',data=bytes(data),is_dense=True)
    result=module.filter_cloud(msg,c);out=np.ndarray((h,w),dtype=dtype,buffer=result.data,strides=(row_step,point_step))
    assert np.array_equal(np.isfinite(out['x']).ravel(),visible)
    assert np.all(out['intensity']==42) and msg.data==bytes(data)
    for row in range(h):assert result.data[row*row_step+w*point_step:(row+1)*row_step]==b'\xee'*8
bridge=json.loads((P/'config/bridge.yaml').read_text())
assert any(b['ros_topic_name']=='/promise/os1/points_raw' for b in bridge)
report={'status':'PASS','revision':spec['revision'],'urdf_mount_pitch_down_deg':float(pitch),'glb_max_transform_error':glb_error,'usd_max_transform_error':usd_error,'usd_fixed_joint_orientation':'PASS','mask_rays':int(visible.size),'mask_clear_rays':int(visible.sum()),'ros_pointcloud_filter':'PASS; organized clouds, both byte orders, padded rows, auxiliary fields preserved','gazebo_runtime':'NOT RUN','isaac_runtime':'NOT RUN; raw RTX output needs documented post-filter'}
(ROOT/'references/recessed_exports_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
