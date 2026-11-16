"""Verify sensor poses, real rendered camera data and a known lidar target."""
import json, os, struct, time, xml.etree.ElementTree as ET
from pathlib import Path
os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco, numpy as np, yourdfpy
from PIL import Image
from pxr import Usd, UsdGeom
from mujoco_sensors import OS1Scanner
from sensor_rays import ray_directions

ROOT=Path(__file__).resolve().parents[1]
P=ROOT/'ros2/promise_description'
spec=json.loads((ROOT/'data/robot_spec.json').read_text())
config=spec['lidar']; base=np.array(spec['links']['base_link']['anchor'])
robot=yourdfpy.URDF.load(str(P/'urdf/promise.urdf'),load_meshes=False)
m=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco/promise.xml')); d=mujoco.MjData(m)
position_errors=[]; rotation_errors=[]
for yaw,pitch in [(0.,0.),(.25,-.15),(-.3,.2)]:
    q={spec['links']['mast_yaw_link']['joint']:np.float64(yaw),spec['links']['head_pitch_link']['joint']:np.float64(pitch)}
    robot.update_cfg(q);mujoco.mj_resetData(m,d);d.qpos[:3]=base
    for name,value in q.items():d.qpos[m.joint(name).qposadr[0]]=value
    mujoco.mj_forward(m,d)
    pairs=[]
    for c in spec['cameras']:
        i=m.camera(c['name']).id
        pairs.append((c['name']+'_optical_frame',d.cam_xpos[i],d.cam_xmat[i].reshape(3,3)@np.diag([1,-1,-1])))
    i=m.site(config['frame']).id;pairs.append((config['frame'],d.site_xpos[i],d.site_xmat[i].reshape(3,3)))
    for frame,position,rotation in pairs:
        tf=robot.get_transform(frame)
        position_errors.append(float(np.max(np.abs(tf[:3,3]+base-position))))
        rotation_errors.append(float(np.max(np.abs(tf[:3,:3]-rotation))))
assert max(position_errors)<1e-7 and max(rotation_errors)<1e-7

tree=ET.parse(ROOT/'mujoco/promise.xml')
ET.SubElement(tree.getroot().find('worldbody'),'geom',name='sensor_test_target',type='box',pos='5 0 1.8',size='.5 1 1',rgba='.08 .3 .7 1')
path=ROOT/'mujoco/.sensor_test.xml';tree.write(path)
try:mt=mujoco.MjModel.from_xml_path(str(path))
finally:path.unlink()
dt=mujoco.MjData(mt);mujoco.mj_forward(mt,dt)
scanner=OS1Scanner(mt);start=time.monotonic();assert scanner.update(dt);elapsed=time.monotonic()-start
index=100*config['columns'];expected=(4.5-scanner.origin[0])/(ray_directions(config)[index]@scanner.rotation.T)[0]
assert abs(scanner.ranges[index]-expected)<1e-5
before=scanner.ranges.copy();dt.time=.05;assert not scanner.update(dt)
assert np.allclose(before,scanner.ranges,equal_nan=True)
dt.time=.1;assert scanner.update(dt)
out=ROOT/'previews/mujoco_sensors';out.mkdir(exist_ok=True);scanner.save(out/'os1.npz')
camera_results={}
with mujoco.Renderer(mt,height=180,width=320) as renderer:
    for c in spec['cameras']:
        renderer.disable_depth_rendering();renderer.update_scene(dt,camera=c['name']);rgb=renderer.render().copy()
        Image.fromarray(rgb).save(out/(c['name']+'.png'))
        renderer.enable_depth_rendering();renderer.update_scene(dt,camera=c['name']);depth=renderer.render().copy()
        measured=float(depth[90,160]);assert abs(measured-(4.5-c['position'][0]))<1e-4
        assert rgb.std()>1
        camera_results[c['name']]={'resolution':[320,180],'target_depth_m':measured}
    renderer.disable_depth_rendering();renderer.update_scene(dt,camera='overview');Image.fromarray(renderer.render()).save(ROOT/'previews/mujoco_preview.png')
buf=(ROOT/'glb/promise.glb').read_bytes();n=struct.unpack_from('<I',buf,12)[0];glb=json.loads(buf[20:20+n])
assert len(glb['cameras'])==4
node=next(x for x in glb['nodes'] if x.get('name')==config['frame'])
assert json.loads(node['extras']['profile_json'])['channels']==128
stage=Usd.Stage.Open(str(ROOT/'usd/promise_scene.usda'))
assert len([p for p in stage.Traverse() if p.IsA(UsdGeom.Camera)])==4
lidar=stage.GetPrimAtPath('/World/PROMISE/os1_link/'+config['frame']+'/sensor')
assert lidar.GetTypeName()=='OmniLidar'
assert lidar.GetAttribute('omni:sensor:Core:numberOfEmitters').Get()==128
assert len(lidar.GetAttribute('omni:sensor:Core:emitterState:s001:elevationDeg').Get())==128
report={'status':'PASS','turret_pose_cases':3,'max_position_error_m':max(position_errors),'max_rotation_matrix_error':max(rotation_errors),'mujoco':mujoco.__version__,'lidar':{'rays':scanner.count,'scan_wall_seconds':elapsed,'target_expected_m':expected,'target_measured_m':float(scanner.ranges[index]),'rate_hold_test':'PASS'},'cameras':camera_results,'glb_cameras':4,'usd':'128 emitter OmniLidar profile and 4 cameras structurally checked; Isaac runtime not run'}
(ROOT/'references/sensor_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
