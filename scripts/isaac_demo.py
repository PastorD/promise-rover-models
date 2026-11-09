#!/usr/bin/env python3
"""Isaac Sim standalone launcher; run using Isaac Sim's ./python.sh."""
import argparse,pathlib,math,sys
p=argparse.ArgumentParser();p.add_argument('--headless',action='store_true');p.add_argument('--seconds',type=float,default=12);a=p.parse_args()
from isaacsim import SimulationApp
app=SimulationApp({'headless':a.headless})
from isaacsim.core.api import World
import omni.usd
from pxr import UsdPhysics
from kinematics import wheel_commands,WHEEL_ORDER,STEER_ORDER
root=pathlib.Path(__file__).resolve().parents[1]
omni.usd.get_context().open_stage(str(root/'usd/promise_scene.usda'))
for _ in range(10):app.update()
world=World(physics_dt=1/240,rendering_dt=1/60,stage_units_in_meters=1.0)
world.reset();stage=omni.usd.get_context().get_stage();speeds,angles=wheel_commands(.10,0)
for i in range(int(a.seconds*240)):
    ramp=min(1,max(0,(i/240-2)/2))
    for name,v in zip(WHEEL_ORDER,speeds):
        prim=stage.GetPrimAtPath('/World/PROMISE/Joints/'+name+'_wheel_joint')
        UsdPhysics.DriveAPI(prim,'angular').GetTargetVelocityAttr().Set(math.degrees(v*ramp))
    world.step(render=not a.headless)
app.close()
