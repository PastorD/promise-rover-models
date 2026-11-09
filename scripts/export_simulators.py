"""Generate URDF / MJCF 4.0 / articulated OpenUSD from one common robot spec."""
import pathlib, json, math, shutil, xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf, Gf, Vt

ROOT=pathlib.Path(__file__).resolve().parents[1];P=ROOT/'ros2/promise_description'
spec=json.loads((ROOT/'data/robot_spec.json').read_text());L=spec['links'];MAT=spec['materials']
for d in [P/'urdf',P/'config',P/'launch',P/'worlds',ROOT/'mujoco',ROOT/'usd',ROOT/'usd/textures']:d.mkdir(parents=True,exist_ok=True)
def f(v):return ' '.join(f'{float(x):.9g}' for x in v)
def sub(p,tag,**kw):return E.SubElement(p,tag,{k:str(v) for k,v in kw.items()})
def write(e,path):E.indent(e);E.ElementTree(e).write(path,encoding='utf-8',xml_declaration=True)
def posrel(a,b):return (np.array(a)-np.array(b)).tolist()
def quat(rpy):
    x,y,z,w=Rotation.from_euler('xyz',rpy).as_quat();return (w,x,y,z)

# Collada has broad support in RViz and Gazebo. Keep textures relative to meshes.
def collada(ident,material):
    a=np.load(P/'meshes'/f'{ident}.npz');verts=a['vertices'];norms=a['normals'];uv=a['uv'];n=len(verts)
    ns='http://www.collada.org/2005/11/COLLADASchema';root=E.Element('COLLADA',xmlns=ns,version='1.4.1')
    asset=sub(root,'asset');sub(asset,'unit',name='meter',meter='1');sub(asset,'up_axis').text='Z_UP'
    m=MAT[material];texture=m.get('texture')
    if texture:
        imgs=sub(root,'library_images');im=sub(imgs,'image',id='texture');sub(im,'init_from').text='../textures/'+texture
    fx=sub(sub(root,'library_effects'),'effect',id='effect');prof=sub(fx,'profile_COMMON')
    if texture:
        par=sub(prof,'newparam',sid='surface');surface=sub(par,'surface',type='2D');sub(surface,'init_from').text='texture'
        sampler=sub(sub(prof,'newparam',sid='sampler'),'sampler2D');sub(sampler,'source').text='surface'
    tech=sub(prof,'technique',sid='common');phong=sub(tech,'phong');diff=sub(phong,'diffuse')
    if texture:sub(diff,'texture',texture='sampler',texcoord='UVMap')
    else:sub(diff,'color').text=f(m['rgba'])
    sub(sub(phong,'specular'),'color').text='0.2 0.2 0.2 1';sub(sub(phong,'shininess'),'float').text='32'
    mat=sub(sub(root,'library_materials'),'material',id='material',name=material);sub(mat,'instance_effect',url='#effect')
    geo=sub(sub(root,'library_geometries'),'geometry',id='geometry');mesh=sub(geo,'mesh')
    for key,data,components in [('positions',verts,['X','Y','Z']),('normals',norms,['X','Y','Z']),('uv',uv,['S','T'])]:
        src=sub(mesh,'source',id=key);arr=sub(src,'float_array',id=key+'-array',count=data.size);arr.text=f(data.ravel())
        acc=sub(sub(src,'technique_common'),'accessor',source='#'+key+'-array',count=n,stride=len(components))
        for x in components:sub(acc,'param',name=x,type='float')
    v=sub(mesh,'vertices',id='vertices');sub(v,'input',semantic='POSITION',source='#positions')
    tri=sub(mesh,'triangles',count=n//3,material='material');sub(tri,'input',semantic='VERTEX',source='#vertices',offset=0);sub(tri,'input',semantic='NORMAL',source='#normals',offset=1);sub(tri,'input',semantic='TEXCOORD',source='#uv',offset=2,set=0)
    sub(tri,'p').text=' '.join(f'{i} {i} {i}' for i in range(n))
    scene=sub(sub(root,'library_visual_scenes'),'visual_scene',id='Scene');node=sub(scene,'node',id='node');inst=sub(node,'instance_geometry',url='#geometry');binding=sub(sub(sub(inst,'bind_material'),'technique_common'),'instance_material',symbol='material',target='#material');sub(binding,'bind_vertex_input',semantic='UVMap',input_semantic='TEXCOORD',input_set=0)
    sub(sub(root,'scene'),'instance_visual_scene',url='#Scene');write(root,P/'meshes'/f'{ident}.dae')
for l in L.values():
    for v in l['visuals']:collada(v['mesh'],v['material'])

# Portable URDF: all nominal axes are parallel to the FLU body axes.
urdf=E.Element('robot',name='promise')
for name,l in L.items():
    e=sub(urdf,'link',name=name);inertial=sub(e,'inertial');sub(inertial,'origin',xyz=f(l['com']),rpy='0 0 0');sub(inertial,'mass',value=l['mass'])
    sub(inertial,'inertia',ixx=l['inertia'][0],iyy=l['inertia'][1],izz=l['inertia'][2],ixy=0,ixz=0,iyz=0)
    for v in l['visuals']:
        vis=sub(e,'visual',name=v['mesh']);sub(sub(vis,'geometry'),'mesh',filename='package://promise_description/meshes/'+v['mesh']+'.dae')
    for i,c in enumerate(l['collisions']):
        co=sub(e,'collision',name=name+'_collision_'+str(i));sub(co,'origin',xyz=f(c['pos']),rpy=f(c.get('rpy',[0,0,0])));g=sub(co,'geometry')
        if c['type']=='box':sub(g,'box',size=f(c['size']))
        else:sub(g,'cylinder',radius=c['radius'],length=c['length'])
    if l['parent']:
        j=sub(urdf,'joint',name=l['joint'],type=l['type']);sub(j,'parent',link=l['parent']);sub(j,'child',link=name);sub(j,'origin',xyz=f(posrel(l['anchor'],L[l['parent']]['anchor'])),rpy='0 0 0');sub(j,'axis',xyz=f(l['axis']))
        limit=dict(effort=80 if name.endswith('_wheel') else 150,velocity=4 if name.endswith('_wheel') else .6)
        if l['limits']:limit.update(lower=l['limits'][0],upper=l['limits'][1])
        sub(j,'limit',**limit);sub(j,'dynamics',damping=15 if name.startswith(('rocker','bogie')) else .1,friction=.05)
        if l['mimic']:sub(j,'mimic',joint=l['mimic']['joint'],multiplier=-1,offset=0)
def fixed_frame(name,parent,pos,rpy=[0,0,0]):
    sub(urdf,'link',name=name);j=sub(urdf,'joint',name=name+'_joint',type='fixed');sub(j,'parent',link=parent);sub(j,'child',link=name);sub(j,'origin',xyz=f(pos),rpy=f(rpy))
fixed_frame('base_footprint','base_link',[0,0,-.9])
fixed_frame('imu_link','base_link',[0,0,0])
for c in spec['cameras']:
    fixed_frame(c['name']+'_frame',c['parent'],posrel(c['position'],L[c['parent']]['anchor']))
    fixed_frame(c['name']+'_optical_frame',c['name']+'_frame',[0,0,0],[-math.pi/2,0,-math.pi/2])
write(urdf,P/'urdf/promise.urdf')

# ROS 2 Jazzy / Gazebo Harmonic wrapper, with passive suspension and differential.
x=E.Element('robot',{'xmlns:xacro':'http://www.ros.org/wiki/xacro','name':'promise'})
sub(x,'xacro:include',filename='$(find promise_description)/urdf/promise.urdf')
control=sub(x,'ros2_control',name='GazeboSimSystem',type='system');sub(sub(control,'hardware'),'plugin').text='gz_ros2_control/GazeboSimSystem'
for name,l in L.items():
    if not l['joint']:continue
    j=sub(control,'joint',name=l['joint'])
    if name.endswith('_wheel'):sub(j,'command_interface',name='velocity')
    elif name.endswith('_steer') or name in ['mast_yaw_link','head_pitch_link']:sub(j,'command_interface',name='position')
    for interface in ['position','velocity','effort']:sub(j,'state_interface',name=interface)
gaz=sub(x,'gazebo');plugin=sub(gaz,'plugin',filename='libgz_ros2_control-system.so',name='gz_ros2_control::GazeboSimROS2ControlPlugin');sub(plugin,'parameters').text='$(find promise_description)/config/controllers.yaml';sub(plugin,'hold_joints').text='false'
for name,l in L.items():
    if name.endswith('_wheel'):
        g=sub(x,'gazebo',reference=name)
        for k,val in [('mu1',.9),('mu2',.9),('kp',1000000),('kd',100),('selfCollide','false')]:sub(g,k).text=str(val)
for c in spec['cameras']:
    g=sub(x,'gazebo',reference=c['parent']);s=sub(g,'sensor',name=c['name'],type='camera');sub(s,'pose').text=f(posrel(c['position'],L[c['parent']]['anchor']))+' 0 0 0';sub(s,'topic').text='/promise/'+c['name'];sub(s,'always_on').text='true';sub(s,'update_rate').text=str(c['rate']);sub(s,'gz_frame_id').text=c['name']+'_optical_frame'
    cam=sub(s,'camera',name=c['name']);sub(cam,'horizontal_fov').text=str(c['hfov']);im=sub(cam,'image');sub(im,'width').text=str(c['width']);sub(im,'height').text=str(c['height']);sub(im,'format').text='R8G8B8';clip=sub(cam,'clip');sub(clip,'near').text='.05';sub(clip,'far').text='100'
    fx=c['width']/(2*math.tan(c['hfov']/2));cx=(c['width']-1)/2;cy=(c['height']-1)/2;tx=-fx*c['baseline'] if c['name'].endswith('right') else 0
    calibration={'image_width':c['width'],'image_height':c['height'],'camera_name':c['name'],'camera_matrix':{'rows':3,'cols':3,'data':[fx,0,cx,0,fx,cy,0,0,1]},'distortion_model':'plumb_bob','distortion_coefficients':{'rows':1,'cols':5,'data':[0]*5},'rectification_matrix':{'rows':3,'cols':3,'data':[1,0,0,0,1,0,0,0,1]},'projection_matrix':{'rows':3,'cols':4,'data':[fx,0,cx,tx,0,fx,cy,0,0,0,1,0]}}
    (P/'config'/f"{c['name']}_calibration.yaml").write_text(json.dumps(calibration,indent=2)) # JSON is valid YAML.
write(x,P/'urdf/promise.gazebo.urdf.xacro')

# Native MJCF 4.0: six torque-limited velocity actuators, position servos,
# passive rocker/bogie joints and an ideal differential equality constraint.
m=E.Element('mujoco',model='PROMISE')
sub(m,'compiler',angle='radian',autolimits='true',inertiafromgeom='false')
sub(m,'option',timestep='.002',gravity='0 0 -1.62',integrator='implicitfast',solver='Newton',iterations='80',cone='elliptic')
visual=sub(m,'visual');sub(visual,'global',offwidth=1280,offheight=960);sub(visual,'headlight',ambient='.3 .3 .3',diffuse='.6 .6 .6',specular='.2 .2 .2')
d=sub(m,'default');sub(d,'joint',damping='.1',armature='.03');sub(d,'geom',friction='.9 .01 .001',condim='4',solref='.02 1',solimp='.9 .95 .001')
a=sub(m,'asset');sub(a,'texture',name='ground_tex',type='2d',builtin='checker',rgb1='.22 .24 .27',rgb2='.30 .32 .35',width=512,height=512);sub(a,'material',name='ground_mat',texture='ground_tex',texrepeat='10 10',reflectance='.1')
for key,ma in MAT.items():
    at=dict(name=key,rgba=f(np.clip(ma['rgba'],0,1)),specular=.3,shininess=max(.01,1-ma['roughness']))
    if ma['texture']:
        sub(a,'texture',name=key+'_tex',type='2d',file='../ros2/promise_description/textures/'+ma['texture']);at['texture']=key+'_tex'
    sub(a,'material',**at)
for l in L.values():
    for v in l['visuals']:sub(a,'mesh',name=v['mesh'],file='../ros2/promise_description/meshes/'+v['mesh']+'.obj',inertia='shell')
w=sub(m,'worldbody');sub(w,'light',pos='0 -3 6',dir='0 0 -1',diffuse='.8 .8 .8',directional='true');sub(w,'geom',name='ground',type='plane',size='20 20 .1',material='ground_mat')
sub(w,'camera',name='overview',pos='5 -5 3.6',xyaxes='0.70710678 0.70710678 0 -0.2588 0.2588 0.9306')
bodies={}
for name,l in L.items():
    parent=bodies[l['parent']] if l['parent'] else w
    p=posrel(l['anchor'],L[l['parent']]['anchor']) if l['parent'] else [0,0,.92]
    b=sub(parent,'body',name=name,pos=f(p));bodies[name]=b
    if l['parent']:
        attrs=dict(name=l['joint'],axis=f(l['axis']),type='hinge')
        if l['limits']:attrs['range']=f(l['limits'])
        if name.startswith(('rocker','bogie')):attrs.update(damping=15,armature=.5)
        sub(b,'joint',**attrs)
    else:sub(b,'freejoint',name='floating_base')
    sub(b,'inertial',pos=f(l['com']),mass=l['mass'],diaginertia=f(l['inertia']))
    for v in l['visuals']:sub(b,'geom',name=v['mesh'],type='mesh',mesh=v['mesh'],material=v['material'],contype=0,conaffinity=0,group=2,density=0)
    for i,c in enumerate(l['collisions']):
        attrs=dict(name=name+'_collision_'+str(i),type=c['type'],pos=f(c['pos']),quat=f(quat(c.get('rpy',[0,0,0]))),rgba='0.5 0.5 0.5 0',group=3)
        attrs['size']=f(np.array(c['size'])/2) if c['type']=='box' else f([c['radius'],c['length']/2])
        sub(b,'geom',**attrs)
for c in spec['cameras']:sub(bodies[c['parent']],'camera',name=c['name'],pos=f(posrel(c['position'],L[c['parent']]['anchor'])),xyaxes='0 -1 0 0 0 1',fovy=math.degrees(2*math.atan(c['height']/c['width']*math.tan(c['hfov']/2))))
sub(bodies['base_link'],'site',name='imu',size='.01',rgba='0 0 0 0')
eq=sub(m,'equality');sub(eq,'joint',name='rocker_differential',joint1='rocker_right_joint',joint2='rocker_left_joint',polycoef='0 -1 0 0 0',solref='.01 1')
act=sub(m,'actuator')
for name,l in L.items():
    if name.endswith('_wheel'):sub(act,'velocity',name=name+'_drive',joint=l['joint'],kv=30,ctrlrange='-4 4',forcerange='-80 80')
    elif name.endswith('_steer') or name in ['mast_yaw_link','head_pitch_link']:sub(act,'position',name=l['joint']+'_servo',joint=l['joint'],kp=350,kv=45,ctrlrange=f(l['limits']),forcerange='-150 150')
sen=sub(m,'sensor');sub(sen,'gyro',name='imu_gyro',site='imu');sub(sen,'accelerometer',name='imu_accel',site='imu');sub(sen,'framequat',name='body_orientation',objtype='body',objname='base_link')
write(m,ROOT/'mujoco/promise.xml')

# USD uses the same exact vertex buffers and PBR base textures as the simulators.
for file in (P/'textures').glob('*.png'):shutil.copy2(file,ROOT/'usd/textures'/file.name)
stage=Usd.Stage.CreateNew(str(ROOT/'usd/promise.usdc'));UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1)
root=UsdGeom.Xform.Define(stage,'/PROMISE');stage.SetDefaultPrim(root.GetPrim());root.GetPrim().SetCustomData({'description':'NASA-derived PROMISE, manually articulated, estimated dynamics','sourceDate':'2026-11-08'})
usd_mats={}
for key,ma in MAT.items():
    material=UsdShade.Material.Define(stage,'/PROMISE/Looks/'+key);shader=UsdShade.Shader.Define(stage,material.GetPath().AppendChild('PreviewSurface'));shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*ma['rgba'][:3]));shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(ma['metallic']);shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(ma['roughness']);material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    if ma['texture']:
        uv=UsdShade.Shader.Define(stage,material.GetPath().AppendChild('stReader'));uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
        tex=UsdShade.Shader.Define(stage,material.GetPath().AppendChild('baseColor'));tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set('./textures/'+ma['texture']);tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');shader.GetInput('diffuseColor').ConnectToSource(tex.ConnectableAPI(),'rgb')
    usd_mats[key]=material
physmat=UsdShade.Material.Define(stage,'/PROMISE/Looks/WheelContact');pm=UsdPhysics.MaterialAPI.Apply(physmat.GetPrim());pm.CreateStaticFrictionAttr(.9);pm.CreateDynamicFrictionAttr(.8);pm.CreateRestitutionAttr(0)
for name,l in L.items():
    path='/PROMISE/'+name;xform=UsdGeom.Xform.Define(stage,path);xform.AddTranslateOp().Set(Gf.Vec3d(*l['anchor']));prim=xform.GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim)
    mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(l['mass']);mass.CreateCenterOfMassAttr(Gf.Vec3f(*l['com']));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*l['inertia']));mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
    if name=='base_link':UsdPhysics.ArticulationRootAPI.Apply(prim)
    for v in l['visuals']:
        a=np.load(P/'meshes'/f"{v['mesh']}.npz");verts=a['vertices'];mesh=UsdGeom.Mesh.Define(stage,path+'/visuals/'+v['mesh']);mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(verts));mesh.CreateFaceVertexCountsAttr([3]*(len(verts)//3));mesh.CreateFaceVertexIndicesAttr(list(range(len(verts))));mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(a['normals']));mesh.SetNormalsInterpolation('vertex');mesh.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(Vt.Vec3fArray.FromNumpy(verts)));st=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex);st.Set(Vt.Vec2fArray.FromNumpy(a['uv']));UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(usd_mats[v['material']])
    for i,c in enumerate(l['collisions']):
        cp=path+'/collisions/c'+str(i)
        if c['type']=='box':
            shape=UsdGeom.Cube.Define(stage,cp);shape.CreateSizeAttr(1)
        else:shape=UsdGeom.Cylinder.Define(stage,cp);shape.CreateRadiusAttr(c['radius']);shape.CreateHeightAttr(c['length']);shape.CreateAxisAttr('Z')
        xf=UsdGeom.Xformable(shape);xf.AddTranslateOp().Set(Gf.Vec3d(*c['pos']));q=quat(c.get('rpy',[0,0,0]));xf.AddOrientOp().Set(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
        if c['type']=='box':xf.AddScaleOp().Set(Gf.Vec3f(*c['size']))
        shape.CreatePurposeAttr('guide');UsdPhysics.CollisionAPI.Apply(shape.GetPrim());UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(physmat,materialPurpose='physics')
    if l['parent']:
        j=UsdPhysics.RevoluteJoint.Define(stage,'/PROMISE/Joints/'+l['joint']);j.CreateBody0Rel().SetTargets(['/PROMISE/'+l['parent']]);j.CreateBody1Rel().SetTargets([path]);j.CreateLocalPos0Attr(Gf.Vec3f(*posrel(l['anchor'],L[l['parent']]['anchor'])));j.CreateLocalPos1Attr(Gf.Vec3f(0));j.CreateAxisAttr('Y' if l['axis'][1] else 'Z');j.CreateCollisionEnabledAttr(False)
        if l['limits']:j.CreateLowerLimitAttr(math.degrees(l['limits'][0]));j.CreateUpperLimitAttr(math.degrees(l['limits'][1]))
        if name.endswith('_wheel') or name.endswith('_steer') or name in ['mast_yaw_link','head_pitch_link']:
            drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateMaxForceAttr(80 if name.endswith('_wheel') else 150);drive.CreateStiffnessAttr(0 if name.endswith('_wheel') else 350*math.pi/180);drive.CreateDampingAttr((30 if name.endswith('_wheel') else 45)*math.pi/180);drive.CreateTargetVelocityAttr(0);drive.CreateTargetPositionAttr(0)
        if l['mimic']:
            # PhysxSchema is provided by Isaac Sim, not the standalone usd-core wheel.
            # Author its documented schema and properties without falsely emulating it.
            prim=j.GetPrim();prim.AddAppliedSchema('PhysxMimicJointAPI:rotY')
            for prop,value in [('gearing',1.),('offset',0.),('naturalFrequency',20.),('dampingRatio',1.)]:prim.CreateAttribute('physxMimicJoint:rotY:'+prop,Sdf.ValueTypeNames.Float,custom=False).Set(value)
            prim.CreateAttribute('physxMimicJoint:rotY:referenceJointAxis',Sdf.ValueTypeNames.Token,custom=False).Set('rotY');prim.CreateRelationship('physxMimicJoint:rotY:referenceJoint',custom=False).SetTargets(['/PROMISE/Joints/rocker_left_joint'])
for c in spec['cameras']:
    cam=UsdGeom.Camera.Define(stage,'/PROMISE/'+c['parent']+'/'+c['name']);cam.AddTranslateOp().Set(Gf.Vec3d(*posrel(c['position'],L[c['parent']]['anchor'])));q=Rotation.from_matrix(np.array([[0,0,-1],[-1,0,0],[0,1,0]])).as_quat();cam.AddOrientOp().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])));cam.CreateFocalLengthAttr(12);cam.CreateHorizontalApertureAttr(24*math.tan(c['hfov']/2));cam.CreateVerticalApertureAttr(24*math.tan(c['hfov']/2)*c['height']/c['width']);cam.CreateClippingRangeAttr(Gf.Vec2f(.05,100))
stage.GetRootLayer().Save()
scene=Usd.Stage.CreateNew(str(ROOT/'usd/promise_scene.usda'));UsdGeom.SetStageUpAxis(scene,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(scene,1);world=UsdGeom.Xform.Define(scene,'/World');scene.SetDefaultPrim(world.GetPrim());r=UsdGeom.Xform.Define(scene,'/World/PROMISE');r.GetPrim().GetReferences().AddReference('./promise.usdc');r.AddTranslateOp().Set(Gf.Vec3d(0,0,.02));phys=UsdPhysics.Scene.Define(scene,'/World/PhysicsScene');phys.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));phys.CreateGravityMagnitudeAttr(1.62)
ground=UsdGeom.Cube.Define(scene,'/World/Ground');ground.CreateSizeAttr(1);ground.AddTranslateOp().Set(Gf.Vec3d(0,0,-.05));ground.AddScaleOp().Set(Gf.Vec3f(40,40,.1));ground.CreateDisplayColorAttr([Gf.Vec3f(.24,.25,.27)]);UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
light=UsdLux.DomeLight.Define(scene,'/World/Sky');light.CreateIntensityAttr(700);sun=UsdLux.DistantLight.Define(scene,'/World/Sun');sun.CreateIntensityAttr(1800);sun.AddRotateXYZOp().Set(Gf.Vec3f(30,-35,-25));scene.GetRootLayer().Save()
print('EXPORTED',len(L),'links',sum(bool(l['joint']) for l in L.values()),'joints; mass',spec['total_mass_kg'],'kg')
