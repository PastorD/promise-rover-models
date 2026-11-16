#!/usr/bin/env python3
"""Isaac Sim standalone launcher; run using Isaac Sim's ./python.sh."""
import argparse,pathlib,math,sys
p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--seconds',type=float,default=12);p.add_argument('--capture',type=pathlib.Path);a=p.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':a.headless})
from isaacsim.core.api import World
import omni.usd
from pxr import UsdPhysics
from isaacsim.sensors.experimental.rtx import LidarSensor,CameraSensor
import numpy as np,json
from kinematics import wheel_commands,WHEEL_ORDER,STEER_ORDER
root=pathlib.Path(__file__).resolve().parents[1]
omni.usd.get_context().open_stage(str(root/'usd/promise_scene.usda'))
for _ in range(10):app.update()
world=World(physics_dt=1/240,rendering_dt=1/60,stage_units_in_meters=1.0)
spec=json.loads((root/'data/robot_spec.json').read_text());lidar=LidarSensor('/World/PROMISE/os1_link/os1_lidar_frame/sensor',annotators=['generic-model-output'])
cameras={c['name']:CameraSensor('/World/PROMISE/'+c['parent']+'/'+c['name'],resolution=(c['height'],c['width']),annotators=['rgb','distance_to_camera']) for c in spec['cameras']}
world.reset();stage=omni.usd.get_context().get_stage();speeds,angles=wheel_commands(.10,0)
for i in range(int(a.seconds*240)):
    ramp=min(1,max(0,(i/240-2)/2))
    for name,v in zip(WHEEL_ORDER,speeds):
        prim=stage.GetPrimAtPath('/World/PROMISE/Joints/'+name+'_wheel_joint')
        UsdPhysics.DriveAPI(prim,'angular').GetTargetVelocityAttr().Set(math.degrees(v*ramp))
    world.step(render=True)
if a.capture:
    a.capture.mkdir(parents=True,exist_ok=True)
    def array(v):return v.numpy() if hasattr(v,'numpy') else np.asarray(v)
    for name,camera in cameras.items():
        rgb,_=camera.get_data('rgb');depth,_=camera.get_data('distance_to_camera')
        if rgb is None or depth is None:raise RuntimeError('No output from camera '+name)
        np.savez_compressed(a.capture/(name+'.npz'),rgb=array(rgb),range_m=array(depth))
    raw,_=lidar.get_data('generic-model-output')
    if raw is None:raise RuntimeError('No RTX lidar output; check RTX support and extensions.')
    np.save(a.capture/'os1_generic_model_output.npy',array(raw))
    (a.capture/'os1_profile.json').write_text(json.dumps(spec['lidar'],indent=2))
    (a.capture/'README.txt').write_text('Lidar is raw GenericModelOutput; decode using isaacsim.sensors.experimental.rtx.parse_generic_model_output_data. For Cartesian SENSOR-frame XYZ, apply scripts/sensor_rays.py:valid_sensor_points(profile, xyz) to enforce the head aperture and 0.5 m minimum range. Native RTX near-field occlusion has not been runtime-tested. Snapshot capture is not a lossless stream recorder.\n')
app.close()
