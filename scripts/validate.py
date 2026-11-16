"""Structural and physical checks; produces a machine-readable validation report."""
import json,pathlib,struct,xml.etree.ElementTree as E,sys,math
import numpy as np
import mujoco,trimesh,yourdfpy,xacro
from pxr import Usd,UsdGeom,UsdPhysics,UsdUtils,Sdf
from kinematics import wheel_commands,WHEEL_ORDER,STEER_ORDER
ROOT=pathlib.Path(__file__).resolve().parents[1];P=ROOT/'ros2/promise_description'
spec=json.loads((ROOT/'data/robot_spec.json').read_text());links=spec['links'];report={}

# Read every referenced ROS mesh, rather than accepting XML syntax alone.
robot=yourdfpy.URDF.load(str(P/'urdf/promise.urdf'),filename_handler=lambda fname:str(P/fname.split('package://promise_description/')[-1]),load_meshes=True,build_scene_graph=True)
assert robot.base_link=='base_link'
assert len([j for j in robot.robot.joints if j.type=='continuous'])==6
assert not any('arm_' in j.name for j in robot.robot.joints)
for name,l in links.items():
    tf=robot.get_transform(name);expected=np.array(l['anchor'])-np.array(links['base_link']['anchor'])
    assert np.allclose(tf[:3,3],expected,atol=1e-7),(name,tf,expected)
    assert min(l['inertia'])>0
    assert np.max(np.abs(l['bounds']))<5
root=E.parse(P/'urdf/promise.urdf').getroot()
for mesh in root.findall('.//mesh'):assert (P/mesh.get('filename').split('package://promise_description/')[-1]).is_file()
report['urdf']={'status':'PASS','links':len(robot.robot.links),'movable_joints':16,'wheel_joints':6,'mesh_parts':sum(len(l['visuals']) for l in links.values()),'nominal_link_transform_check':'PASS','mass_kg_estimated':spec['total_mass_kg']}

# Resolve ament's package lookup for an offline xacro expansion check.
import xacro.substitution_args
xacro.substitution_args._eval_find=lambda package:str(P) if package=='promise_description' else (_ for _ in ()).throw(ValueError(package))
expanded=xacro.process_file(str(P/'urdf/promise.gazebo.urdf.xacro')).toxml();eroot=E.fromstring(expanded)
assert len(eroot.findall('.//sensor'))==5
assert len(eroot.findall('ros2_control/joint'))==16
report['gazebo_ros2']={'status':'STRUCTURAL_PASS_RUNTIME_NOT_RUN','xacro_expansion':'PASS','camera_sensors':4,'lidar_sensors':1,'ros2_control_joints':16,'runtime_reason':'Gazebo / ROS 2 are not installed in the build environment.'}

m=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco/promise.xml'))
assert mujoco.__version__.startswith('4.0');assert m.nu==12 and m.nv==22
report['mujoco']={'version':mujoco.__version__,'nq':m.nq,'nv':m.nv,'actuators':m.nu,'test_cases':[]}
def simulate(label,speed,yaw,seconds):
    d=mujoco.MjData(m);speeds,steers=wheel_commands(speed,yaw);max_diff=0.;wheel_contact_steps=0;settled=None
    wheel_bodies={m.body(n+'_wheel').id for n in WHEEL_ORDER}
    for i in range(round(seconds/m.opt.timestep)):
        ramp=min(1,max(0,(d.time-2)/2))
        for n,v in zip(WHEEL_ORDER,speeds):d.ctrl[m.actuator(n+'_wheel_drive').id]=v*ramp
        for n,v in zip(STEER_ORDER,steers):d.ctrl[m.actuator(n+'_steer_joint_servo').id]=v*ramp
        mujoco.mj_step(m,d)
        assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
        ql=d.qpos[m.joint('rocker_left_joint').qposadr[0]];qr=d.qpos[m.joint('rocker_right_joint').qposadr[0]];max_diff=max(max_diff,abs(ql+qr))
        if i==999:settled=d.qpos[:3].copy()
    displacement=d.qpos[:3]-settled
    contacted=set()
    for c in d.contact:
        if c.geom1==0:contacted.add(int(m.geom_bodyid[c.geom2]))
        if c.geom2==0:contacted.add(int(m.geom_bodyid[c.geom1]))
    warnings=[int(x.number) for x in d.warning]
    case={'name':label,'simulated_seconds':seconds,'final_position_m':d.qpos[:3].tolist(),'displacement_after_settle_m':displacement.tolist(),'max_rocker_constraint_error_rad':max_diff,'wheels_touching_ground_at_end':len(contacted&wheel_bodies),'warnings':warnings,'finite':True}
    assert not any(warnings),case
    assert .6<d.qpos[2]<1.2,case
    assert max_diff<.01,case
    if label=='forward':assert displacement[0]>.5 and abs(displacement[1])<.15,case
    if label=='stationary':assert np.linalg.norm(displacement[:2])<.05,case
    report['mujoco']['test_cases'].append(case)
simulate('stationary',0,0,6)
simulate('forward',.1,0,12)
simulate('turn',.1,.03,10)
report['mujoco']['status']='PASS'

# Check a genuine GLB and all six animation channels / self-contained images.
buf=(ROOT/'glb/promise.glb').read_bytes();magic,version,length=struct.unpack_from('<III',buf);assert magic==0x46546c67 and version==2 and length==len(buf)
jl=struct.unpack_from('<I',buf,12)[0];g=json.loads(buf[20:20+jl]);assert len(g['animations'])==1
animated={g['nodes'][c['target']['node']]['name'] for c in g['animations'][0]['channels'] if c['target']['path']=='rotation'}
assert {x+'_wheel' for x in WHEEL_ORDER}<=animated
assert all('bufferView' in i for i in g.get('images',[]))
report['glb']={'status':'PASS','bytes':len(buf),'animations':len(g['animations']),'animated_wheels':6,'embedded_images':len(g.get('images',[]))}

stage=Usd.Stage.Open(str(ROOT/'usd/promise_scene.usda'));assert stage
assert UsdGeom.GetStageMetersPerUnit(stage)==1 and UsdGeom.GetStageUpAxis(stage)=='Z'
joints=[p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
assert len(joints)==16
for j in joints:
    for rel in ['physics:body0','physics:body1']:
        targets=j.GetRelationship(rel).GetTargets();assert len(targets)==1 and stage.GetPrimAtPath(targets[0])
layers,assets,unresolved=UsdUtils.ComputeAllDependencies(str(ROOT/'usd/promise_scene.usda'));assert not unresolved,unresolved
assert len([p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)])==18
assert len([p for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint)])==1
report['usd']={'status':'STRUCTURAL_PASS_ISAAC_RUNTIME_NOT_RUN','openusd_version':Usd.GetVersion(),'revolute_joints':len(joints),'fixed_joints':1,'rigid_bodies':18,'unresolved_references':list(unresolved),'note':'PhysxMimicJointAPI authored; evaluation requires Isaac Sim PhysX.'}
(ROOT/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
