#!/usr/bin/env python3
"""Run the delivered MJCF in MuJoCo 4.0, interactively or headless."""
import argparse,pathlib,time,json
import numpy as np
import mujoco
from kinematics import wheel_commands,WHEEL_ORDER,STEER_ORDER
from mujoco_sensors import OS1Scanner

p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--seconds',type=float,default=12);p.add_argument('--speed',type=float,default=.10);p.add_argument('--yaw-rate',type=float,default=0);p.add_argument('--gravity',type=float,default=1.62);p.add_argument('--output',type=pathlib.Path);p.add_argument('--no-lidar',action='store_true');p.add_argument('--capture',type=pathlib.Path);a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1];m=mujoco.MjModel.from_xml_path(str(root/'mujoco/promise.xml'));m.opt.gravity[2]=-a.gravity;d=mujoco.MjData(m)
speeds,steers=wheel_commands(a.speed,a.yaw_rate)
scanner=None if a.no_lidar else OS1Scanner(m)
def command():
    ramp=min(1,max(0,(d.time-2)/2))
    for n,v in zip(WHEEL_ORDER,speeds):d.ctrl[m.actuator(n+'_wheel_drive').id]=v*ramp
    for n,v in zip(STEER_ORDER,steers):d.ctrl[m.actuator(n+'_steer_joint_servo').id]=v*ramp
def step():
    command();mujoco.mj_step(m,d)
    if scanner:scanner.update(d)
if a.headless:
    for i in range(round(a.seconds/m.opt.timestep)):step()
else:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(m,d) as viewer:
        viewer.cam.lookat[:]=[0,0,1];viewer.cam.distance=5;viewer.cam.azimuth=135;viewer.cam.elevation=-22
        while viewer.is_running() and d.time<a.seconds:
            t=time.monotonic();step();viewer.sync();time.sleep(max(0,m.opt.timestep-(time.monotonic()-t)))
q=lambda n:float(d.qpos[m.joint(n).qposadr[0]])
result={'mujoco':mujoco.__version__,'seconds':float(d.time),'position_m':d.qpos[:3].tolist(),'finite':bool(np.isfinite(d.qpos).all()),'differential_residual_rad':q('rocker_left_joint')+q('rocker_right_joint'),'contacts':int(d.ncon),'warning_counts':[int(x.number) for x in d.warning]}
print(json.dumps(result,indent=2))
if a.output:a.output.write_text(json.dumps(result,indent=2))
if a.capture:
    from PIL import Image
    a.capture.mkdir(parents=True,exist_ok=True)
    if scanner:scanner.update(d,force=True);scanner.save(a.capture/'os1.npz')
    spec=json.loads((root/'data/robot_spec.json').read_text());metadata={};mujoco.mj_forward(m,d)
    with mujoco.Renderer(m,height=720,width=1280) as renderer:
        for c in spec['cameras']:
            name=c['name'];renderer.update_scene(d,camera=name);Image.fromarray(renderer.render()).save(a.capture/(name+'.png'));metadata[name]={'calibration':c,'origin_world_m':d.cam_xpos[m.camera(name).id].tolist(),'rotation_world_from_blender_camera':d.cam_xmat[m.camera(name).id].reshape(3,3).tolist()}
    (a.capture/'cameras.json').write_text(json.dumps(metadata,indent=2))
