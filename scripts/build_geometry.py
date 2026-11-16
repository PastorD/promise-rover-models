"""Assemble NASA donor meshes, manually resegment the robot, and export visuals.
Run with Blender 4.5's Python (bpy), see README. Units are meters, ROS FLU.
"""
import bpy, bmesh, math, json, pathlib, re, sys, numpy as np
from PIL import Image
from mathutils import Matrix, Vector

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
PKG=ROOT/'ros2/promise_description'
MESH=PKG/'meshes'; TEX=PKG/'textures'
for p in [MESH,TEX,ROOT/'blender',ROOT/'glb',ROOT/'previews',ROOT/'data']:p.mkdir(parents=True,exist_ok=True)
R=Matrix.Rotation(math.pi/2,4,'Z') # donor -Y forward -> ROS +X forward
ZOFF=.0227
def ros(p,dz=0):return list(R@Vector(p)+Vector((0,0,dz)))
def freeze(o,mat=R):
    w=o.matrix_world.copy();o.parent=None;o.animation_data_clear()
    o.data=o.data.copy();o.data.transform(mat@w);o.matrix_world=Matrix.Identity(4)
    o.hide_render=False;o.hide_set(False)
    for m in list(o.modifiers):o.modifiers.remove(m)
    return o
def append(path,names):
    with bpy.data.libraries.load(str(path),link=False) as (a,b):b.objects=list(names)
    objects=b.objects
    for o in objects:
        if o and not o.name in bpy.context.scene.objects:bpy.context.collection.objects.link(o)
    bpy.context.view_layer.update()
    return objects
def subset(o,pred,name):
    ob=o.copy();ob.data=o.data.copy();ob.name=name;bpy.context.collection.objects.link(ob)
    bm=bmesh.new();bm.from_mesh(ob.data)
    bad=[f for f in bm.faces if not pred(f.calc_center_median())]
    bmesh.ops.delete(bm,geom=bad,context='FACES')
    loose=[v for v in bm.verts if not v.link_faces]
    bmesh.ops.delete(bm,geom=loose,context='VERTS')
    bm.to_mesh(ob.data);bm.free();ob.data.update()
    return ob
def simplify(o,target=5000):
    # Merge the vertex duplication introduced by glTF's material boundaries.
    if len(o.data.polygons)<target*1.2:return
    bm=bmesh.new();bm.from_mesh(o.data)
    bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=0.00001)
    bmesh.ops.dissolve_degenerate(bm,edges=list(bm.edges),dist=0.000001)
    bm.to_mesh(o.data);bm.free();o.data.update()
    backup=o.data.copy();before=np.array([v.co[:] for v in backup.vertices]);lo=before.min(0);hi=before.max(0)
    bpy.context.view_layer.objects.active=o;o.select_set(True)
    m=o.modifiers.new('Simulation visual LOD','DECIMATE');m.ratio=min(1,target/len(o.data.polygons))
    bpy.ops.object.modifier_apply(modifier=m.name);o.select_set(False)
    after=np.array([v.co[:] for v in o.data.vertices])
    if np.any(after.min(0)<lo-.002) or np.any(after.max(0)>hi+.002):
        print('LOD rejected due to displaced vertices:',o.name,flush=True);o.data=backup
def newmat(name,rgba,metal=0,rough=.45,emission=0):
    m=bpy.data.materials.new(name);m.use_nodes=True;m.diffuse_color=rgba
    p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=rgba;p.inputs['Metallic'].default_value=metal;p.inputs['Roughness'].default_value=rough
    if emission:p.inputs['Emission Color'].default_value=rgba;p.inputs['Emission Strength'].default_value=emission
    return m
def box(name,pos,size,mat,bevel=.01):
    bpy.ops.mesh.primitive_cube_add(size=1,location=pos);o=bpy.context.object;o.name=name;o.dimensions=size
    bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
    o.data.materials.append(mat)
    if bevel:
        m=o.modifiers.new('Machined edges','BEVEL');m.width=bevel;m.segments=2;bpy.ops.object.modifier_apply(modifier=m.name)
    return o

cur=ROOT/'source_assets/curiosity_clean/Curiosity Rover (MSL) (Clean).blend'
per=ROOT/'source_assets/perseverance_blend/Mars 2020 Perseverance Rover.blend'
pro=ROOT/'source_assets/promise_reference.blend'
bpy.ops.wm.open_mainfile(filepath=str(cur))
# Explicit donor selection: omit the entire arm hierarchy, old mast and Mars labs.
base_names=['body','Chassis','radiators','RTG','antenna_LG',
 'suspension_axel_L','suspension_axel_L2','suspension_axel_R','suspension_axel_R2',
 'suspension_xmember','suspension_rod_L','suspension_rod_R']
susp_names=['suspension_arm_'+p+'_'+s for s in ['L','R'] for p in ['F','B','B2']]+['suspension_steer_'+p+'_'+s for s in ['L','R'] for p in ['F','B']]
keep=set(base_names+susp_names)
original={o.name:o for o in bpy.data.objects if o.name in keep}
transforms={n:o.matrix_world.copy() for n,o in original.items()}
for n,o in original.items():o.parent=None;o.matrix_world=transforms[n]
for o in list(bpy.data.objects):
    if o.name not in keep:bpy.data.objects.remove(o,do_unlink=True)
for o in original.values():freeze(o,Matrix.Translation((0,0,ZOFF))@R)

links={}; groups={}; provenance=[]
def link(name,anchor,parent=None,joint=None,axis=None,kind='fixed',mass=1,limits=None,mimic=None):
    links[name]=dict(name=name,anchor=list(anchor),parent=parent,joint=joint,axis=axis,type=kind,mass=mass,limits=limits,mimic=mimic,collisions=[])
    groups[name]=[]
link('base_link',[0,0,.9],mass=760)
groups['base_link']=[original[n] for n in base_names]
provenance.append(dict(source='NASA Curiosity Rover (MSL) (Clean), July 7 2026 listing',objects=base_names+susp_names,operation='Remove arm, original mast, science instruments; preserve body and rocker-bogie geometry; rotate FLU and raise 22.7 mm for larger wheels.'))
# Curiosity's front and aft rocker tubes are one rigid rocker, not two free joints.
for side,tag,sgn in [('left','L',1),('right','R',-1)]:
    rocker=ros([sgn*.797,-.213,.884],ZOFF)
    bogie=ros([sgn*.797,.540,.643],ZOFF)
    link('rocker_'+side,rocker,'base_link','rocker_'+side+'_joint',[0,1,0],'revolute',20,[-.65,.65],{'joint':'rocker_left_joint','multiplier':-1} if side=='right' else None)
    groups['rocker_'+side]=[original['suspension_arm_'+p+'_'+tag] for p in ['F','B']]
    link('bogie_'+side,bogie,'rocker_'+side,'bogie_'+side+'_joint',[0,1,0],'revolute',15,[-.9,.9])
    groups['bogie_'+side]=[original['suspension_arm_B2_'+tag]]
    for which,letter,oldy in [('front','F',-1.094),('rear','B',1.164)]:
        anchor=ros([sgn*1.061,oldy,.642],ZOFF)
        name=f'{which}_{side}_steer'
        link(name,anchor,'rocker_'+side if which=='front' else 'bogie_'+side,name+'_joint',[0,0,1],'revolute',6,[-1.4,1.4])
        groups[name]=[original['suspension_steer_'+letter+'_'+tag]]

# The current Perseverance Blender file groups all six wheels into one mesh.
# Split by physical wheel location, preserve material / UV data, and re-center
# each wheel's axle on the manually defined kinematic frame.
w=append(per,['Wheels_objs'])[0];freeze(w)
for side,sgn in [('left',1),('right',-1)]:
    for which,xnom,ynom in [('front',1.094,sgn*1.061),('middle',-.089,sgn*1.192),('rear',-1.164,sgn*1.061)]:
        def pred(p,which=which,sgn=sgn):return p.y*sgn>0 and ((p.x>.5) if which=='front' else (p.x<-.5) if which=='rear' else abs(p.x)<.5)
        o=subset(w,pred,f'Perseverance_{which}_{side}')
        a=np.array([v.co[:] for v in o.data.vertices]);lo=a.min(0);hi=a.max(0)
        center=(lo+hi)/2
        radius=max(hi[0]-lo[0],hi[2]-lo[2])/2
        # Axle coordinate along Y does not affect spin. Align tire centers to heritage chassis.
        target=np.array([xnom,ynom,.2625]);factor=.2625/radius
        for v in o.data.vertices:v.co=Vector((np.array(v.co)-center)*factor+target)
        name=f'{which}_{side}_wheel'
        parent='bogie_'+side if which=='middle' else f'{which}_{side}_steer'
        link(name,target,parent,name+'_joint',[0,1,0],'continuous',6)
        groups[name]=[o]
        links[name]['wheel_radius']=.2625;links[name]['wheel_width']=float((hi[1]-lo[1])*factor)
bpy.data.objects.remove(w,do_unlink=True)
provenance.append(dict(source='NASA Mars 2020 Perseverance Rover.blend, October 16 2026 listing',objects=['Wheels_objs'],operation='Split all six wheels; normalize tread radius to 0.2625 m; remount on Curiosity suspension; preserve NASA treads, spokes and hub materials.'))

# Latest PROMISE-only apparatus: integrate actual NASA visualization geometry
# and manually assign rigid groups, yaw/pitch axes and sensor frames.
names=['Mesh_'+str(x) for x in [0,1,2,3,17,18,19,20,21,22,23,24,25]]+['Mesh_24.001']
addons=append(pro,names)
for o,source_name in zip(addons,names):
    freeze(o);o.name='PROMISE_'+source_name
    if source_name=='Mesh_20':
        mast=subset(o,lambda p:p.z<1.875,'PROMISE_compact_mast')
        head=subset(o,lambda p:p.z>=1.875,'PROMISE_compact_head')
        simplify(mast,5000);simplify(head,7000)
        link('mast_yaw_link',[.679,-.164,1.17],'base_link','mast_yaw_joint',[0,0,1],'revolute',6,[-math.pi,math.pi])
        link('head_pitch_link',[.679,-.164,1.89],'mast_yaw_link','head_pitch_joint',[0,1,0],'revolute',4,[-.65,.65])
        groups['mast_yaw_link']=[mast];groups['head_pitch_link']=[head]
        bpy.data.objects.remove(o,do_unlink=True)
    else:
        simplify(o,6000 if source_name in ['Mesh_1','Mesh_3','Mesh_25'] else 3500)
        groups['base_link'].append(o)
provenance.append(dict(source='NASA PROMISE interactive GLB, November 7 2026 release',objects=names,operation='Use current head, stowed TRIDENT and payload accessory meshes; manually split mast/head and create yaw/pitch joints. Exclude its chassis, wheels and suspension, which come from the requested donor models.'))

# New simplified surface appendages replace Curiosity-only equipment.
white=newmat('PROMISE_thermal_white',(.82,.84,.86,1),.18,.43)
dark=newmat('PROMISE_black_anodized',(.045,.055,.065,1),.7,.35)
silver=newmat('PROMISE_silver',(.52,.55,.58,1),.8,.3)
lamp=newmat('PROMISE_illuminator',(.7,.91,1,1),.1,.2,2)
# PROMISE's mobility tubes are dark-coated in the released rendering.
for n in susp_names:
    ob=original[n];ob.data.materials.clear();ob.data.materials.append(dark)
    for face in ob.data.polygons:face.material_index=0
for name in ['aluminum_black','gunmetal_dark','gunmetal']:
    ma=bpy.data.materials.get(name)
    if ma and ma.use_nodes:
        bsdf=ma.node_tree.nodes.get('Principled BSDF')
        if bsdf:
            bsdf.inputs['Roughness'].default_value=.48
            bsdf.inputs['Metallic'].default_value=.6
def basebox(*args,**kwargs):
    o=box(*args,**kwargs);groups['base_link'].append(o);return o
basebox('Front arm deletion cover',[.955,0,.94],[.045,1.34,.41],white)
basebox('NSS enclosure',[.71,-.786,.963],[.22,.14,.25],white)
basebox('HPSC enclosure',[-.08,-.785,.79],[.50,.16,.26],white)
basebox('GPU thermal cover',[-.40,-.786,.80],[.20,.16,.23],white)
basebox('UHF pedestal',[-1.083,-.505,1.47],[.075,.075,.23],white)
for y in [-.59,.59]:
    basebox('Front illuminator housing',[.985,y,.82],[.06,.115,.12],dark,.006)
    for row in range(3):
        for col in range(3):basebox('Front LED',[1.017,y+(col-1)*.030,.82+(row-1)*.032],[.003,.023,.024],lamp,.001)
for y in [-.78,.78]:
    basebox('Side illuminator housing',[-.75,y,1.01],[.12,.06,.13],dark,.006)
    for row in range(3):
        for col in range(3):basebox('Side LED',[-.75+(col-1)*.031,y+math.copysign(.032,y),1.01+(row-1)*.034],[.024,.003,.026],lamp,.001)
provenance.append(dict(source='Manually authored approximation against NASA PROMISE render',objects=['arm deletion cover','NSS enclosure','HPSC / GPU exterior','UHF pedestal','illuminator arrays'],operation='Create simplified attachment geometry; placements are illustrative, not flight CAD.'))

# User-requested recessed OS1 mount: a hollow enclosure with a front aperture.
from mast_enclosure import rebuild_enclosure
shell,head_collision_world,enclosure=rebuild_enclosure(groups['head_pitch_link'][0],white,box)
groups['head_pitch_link'].extend(shell)
os1_anchor=[.692,-.164,1.9522]
os1_rpy=[0,math.radians(22.5),0]
os1_rotation=Matrix.Rotation(os1_rpy[1],3,'Y')
link('os1_link',os1_anchor,'head_pitch_link','os1_mount_joint',kind='fixed',mass=.5)
links['os1_link']['rpy']=os1_rpy
os1_materials=[silver,newmat('OS1_optical_window',(.018,.027,.024,1),.15,.18),dark]
triangles=np.load(ROOT/'source_assets/os1_rev8_geometry.npz')['triangles']
vertices=triangles.reshape((-1,3))@np.array(os1_rotation).T+np.array(os1_anchor)
mesh=bpy.data.meshes.new('Ouster_OS1_Rev8_CAD');mesh.from_pydata(vertices.tolist(),[],np.arange(len(vertices)).reshape((-1,3)).tolist());mesh.update()
ob=bpy.data.objects.new('Ouster_OS1_Rev8_physical_case',mesh);bpy.context.collection.objects.link(ob)
for ma in os1_materials:mesh.materials.append(ma)
for face,triangle in zip(mesh.polygons,triangles):
    z=float(triangle[:,2].mean());face.material_index=1 if .021<z<.050 else 2 if z<.012 else 0
simplify(ob,8000);groups['os1_link']=[ob]
# Split the simplified faces at the optical-window material boundaries.
# Classifying a long CAD triangle only by its centroid produced jagged bands.
bm=bmesh.new();bm.from_mesh(ob.data)
for z in (.012,.021,.050):
    bmesh.ops.bisect_plane(bm,geom=list(bm.verts)+list(bm.edges)+list(bm.faces),dist=1e-7,plane_co=Vector(os1_anchor)+os1_rotation@Vector((0,0,z)),plane_no=os1_rotation@Vector((0,0,1)),clear_inner=False,clear_outer=False)
for face in bm.faces:
    z=(os1_rotation.inverted()@(face.calc_center_median()-Vector(os1_anchor))).z
    face.material_index=1 if .021<z<.050 else 2 if z<.012 else 0
bm.to_mesh(ob.data);bm.free();ob.data.update()
plate=box('OS1_internal_tilt_plate',[0,0,-.0025],[.070,.070,.004],dark,.001)
plate.data.transform(Matrix.Translation(Vector(os1_anchor))@os1_rotation.to_4x4())
groups['head_pitch_link'].append(plate)
groups['head_pitch_link'].append(box('OS1_internal_rear_support',[.665,-.164,1.945],[.008,.060,.024],dark,.001))
links['head_pitch_link']['mass']+=.05
provenance.append(dict(source='Ouster OS1 Rev8 XX STEP, 2026-04-30',objects=['OS1 physical case'],operation='Recess into the original white enclosure, near its front face; rotate +22.5 degrees around ROS +Y so forward rays point down. Hollow the box and cut a 150 x 92 mm front aperture; replace roof bracket with an internal tilted cradle. User-requested modification, not NASA flight CAD.'))

# Build a consistent named link hierarchy. Mesh coordinates become link-local.
empties={}
for name,spec in links.items():
    e=bpy.data.objects.new(name,None);bpy.context.collection.objects.link(e);e.empty_display_size=.08
    e.location=spec['anchor'];e.rotation_euler=spec.get('rpy',[0,0,0]);e['mass_kg_estimated']=spec['mass'];e['source_note']='Public NASA visualization, manually articulated; not flight CAD.'
    empties[name]=e
    for o in groups[name]:
        orientation=Matrix.Rotation(spec.get('rpy',[0,0,0])[1],4,'Y')
        o.data.transform(orientation.inverted()@Matrix.Translation(-Vector(spec['anchor'])))
        o.parent=e;o.matrix_parent_inverse=Matrix.Identity(4);o.matrix_basis=Matrix.Identity(4)
    if spec['parent']:
        e.parent=empties[spec['parent']];e.location=Vector(spec['anchor'])-Vector(links[spec['parent']]['anchor'])
    else:
        e['wheel_speed_rad_s']=2*math.pi/8

def collision(name,typ,pos,size=None,radius=None,length=None,rpy=None):
    d=dict(type=typ,pos=(Vector(pos)-Vector(links[name]['anchor']))[:])
    if size:d['size']=size
    if radius:d['radius']=radius
    if length:d['length']=length
    if rpy:d['rpy']=rpy
    links[name]['collisions'].append(d)
collision('base_link','box',[-.02,0,.92],[1.91,1.48,.46])
collision('base_link','box',[-1.24,0,1.20],[.72,.64,.67],rpy=[0,-.27,0])
links['os1_link']['collisions'].append(dict(type='cylinder',pos=[0,0,.029175],radius=.0435,length=.05835))
for name,spec in links.items():
    a=spec['anchor']
    if name.endswith('_wheel'):collision(name,'cylinder',a,radius=.2625,length=.30,rpy=[math.pi/2,0,0])
    elif name.endswith('_steer'):collision(name,'box',[a[0],a[1]-.09*(1 if 'left' in name else -1),a[2]-.18],[.12,.15,.35])
    elif name.startswith('rocker_') or name.startswith('bogie_'):
        # Collision beams approximate the tubing; visual meshes are never contact hulls.
        side=1 if 'left' in name else -1
        if name.startswith('rocker_'):ends=[([a[0],a[1],a[2]],[1.094,side*1.061,.665]),([a[0],a[1],a[2]],[-.540,side*.797,.666])]
        else:ends=[([a[0],a[1],a[2]],[-.089,side*1.192,.2625]),([a[0],a[1],a[2]],[-1.164,side*1.061,.665])]
        for u,v in ends:
            u,v=Vector(u),Vector(v);q=(v-u).to_track_quat('Z','Y');collision(name,'cylinder',(u+v)/2,radius=.043,length=(v-u).length,rpy=list(q.to_euler()))
    elif name=='mast_yaw_link':collision(name,'cylinder',[.679,-.164,1.51],radius=.05,length=.65)
    elif name=='head_pitch_link':
        for c in head_collision_world:collision(name,c['type'],c['pos'],c['size'],rpy=c['rpy'])

# Positive-definite box/cylinder estimates; total mass is a configurable estimate.
for name,spec in links.items():
    verts=np.array([v.co[:] for o in groups[name] for v in o.data.vertices]);lo=verts.min(0);hi=verts.max(0)
    assert np.all(np.abs(verts)<5), f'Invalid visual bounds: {name}, {lo}, {hi}'
    size=np.maximum(hi-lo,.08);com=(lo+hi)/2;m=spec['mass']
    if name.endswith('_wheel'):
        com=np.zeros(3);r=.2625;w=.30;I=[m*(3*r*r+w*w)/12,m*r*r/2,m*(3*r*r+w*w)/12]
    else:I=[m*(size[1]**2+size[2]**2)/12,m*(size[0]**2+size[2]**2)/12,m*(size[0]**2+size[1]**2)/12]
    if name=='base_link':com=np.array([-.10,0,.02]);I=[220.,350.,400.]
    spec.update(com=com.tolist(),inertia=I,bounds=[lo.tolist(),hi.tolist()],visuals=[])

# Approximate optical centers in the compact head; +Z optical forward, +X right.
cameras=[]
for pair,baseline,height in [('stereo',.40,1.951),('dashcam',.31,2.003)]:
    for side,sgn in [('left',1),('right',-1)]:
        cameras.append(dict(name=f'{pair}_{side}',parent='head_pitch_link',position=[.835,-.164+sgn*baseline/2,height],baseline=baseline,width=1280,height=720,hfov=1.221730476,rate=10))
lidar=dict(name='os1',model='Ouster OS1 Rev8',parent='os1_link',frame='os1_lidar_frame',optical_offset=[0,0,.0402],channels=128,columns=1024,rate_hz=10,vertical_fov_deg=44,vertical_min_deg=-22,vertical_max_deg=22,horizontal_fov_deg=360,min_range_m=.5,max_range_m=170,housing_diameter_m=.087,housing_height_m=.05835,mass_kg=.5,calibration='Uniform elevation, zero azimuth offsets, approximate optical origin; not device-calibrated beam intrinsics.',sampling='Instantaneous full frame, single geometric return; no packet transport, reflectivity, native RGB, dual returns or calibrated noise.')
lidar.update(mount_rpy_rad=os1_rpy,mount_pitch_down_deg=22.5,mount_anchor_world_m=os1_anchor,enclosure=enclosure,installation='Recessed inside white mast head, behind a front aperture. Enclosure blocks most side/rear directions; do not interpret the native 360-degree scan grid as unobstructed coverage.')
from head_visibility import calculate_visibility
bpy.context.view_layer.update()
optical_world=Vector(os1_anchor)+os1_rotation@Vector(lidar['optical_offset'])
mask,visibility=calculate_visibility(groups['head_pitch_link'],np.array(optical_world),np.array(os1_rotation),lidar)
lidar.update(head_visible_mask_encoding='zlib+packbits-little-base64',head_visible_mask_b64=mask,head_visibility=visibility)
sensor_objects=[];optical_to_flu=Matrix(((0,0,-1),(-1,0,0),(0,1,0)))
for c in cameras:
    data=bpy.data.cameras.new(c['name']);data.type='PERSP';data.sensor_fit='HORIZONTAL';data.sensor_width=36;data.lens=36/(2*math.tan(c['hfov']/2));data.clip_start=.01;data.clip_end=170
    ob=bpy.data.objects.new(c['name'],data);bpy.context.collection.objects.link(ob);ob.parent=empties[c['parent']];ob.location=Vector(c['position'])-Vector(links[c['parent']]['anchor']);ob.rotation_euler=optical_to_flu.to_euler();ob['sensor_type']='camera';ob['calibration_json']=json.dumps(c);sensor_objects.append(ob)
ob=bpy.data.objects.new(lidar['frame'],None);bpy.context.collection.objects.link(ob);ob.parent=empties['os1_link'];ob.location=lidar['optical_offset'];ob.empty_display_type='ARROWS';ob.empty_display_size=.1;ob['sensor_type']='lidar';ob['profile_json']=json.dumps(lidar);sensor_objects.append(ob)
bpy.data.texts.new('os1_rev8_profile.json').write(json.dumps(lidar,indent=2))

# Save single-material geometry for MJCF/USD and Collada for ROS renderers.
materials={};image_cache={};mesh_count=0
def slug(s):return re.sub('[^a-zA-Z0-9_]+','_',s).strip('_')
def imagefile(im):
    if im.name in image_cache:return image_cache[im.name]
    name=slug(im.name)+'.png';path=TEX/name
    im.filepath_raw=str(path);im.file_format='PNG';im.save()
    # Packed JPEG images can be copied verbatim by Blender's save(), despite
    # their requested extension. Normalize bytes and rebind the verified PNG.
    with Image.open(path) as pil:pil.convert('RGBA').save(path,format='PNG')
    replacement=bpy.data.images.load(str(path),check_existing=False)
    replacement.pixels[0] # force loading before a later save/pack changes paths
    image_cache[replacement.name]=name
    for ma in bpy.data.materials:
        if ma.use_nodes:
            for node in ma.node_tree.nodes:
                if node.type=='TEX_IMAGE' and node.image==im:node.image=replacement
    image_cache[im.name]=name;return name
def material(m):
    if m is None:return 'default'
    key=slug(m.name)
    if key in materials:return key
    color=list(m.diffuse_color);metal=0.;rough=.5;texture=None
    p=next((x for x in m.node_tree.nodes if x.type=='BSDF_PRINCIPLED'),None) if m.use_nodes else None
    if p:
        color=list(p.inputs['Base Color'].default_value);metal=float(p.inputs['Metallic'].default_value);rough=float(p.inputs['Roughness'].default_value)
        sock=p.inputs['Base Color']
        if sock.is_linked:
            n=sock.links[0].from_node
            if n.type=='TEX_IMAGE' and n.image:texture=imagefile(n.image)
    materials[key]=dict(rgba=color,metallic=metal,roughness=rough,texture=texture)
    return key
materials['default']=dict(rgba=[.7,.7,.7,1],metallic=0,roughness=.5,texture=None)
for name in links:
    for o in groups[name]:
        o.data.calc_loop_triangles();mesh=o.data
        uv=mesh.uv_layers.active.data if mesh.uv_layers.active else None
        for mi in sorted(set(t.material_index for t in mesh.loop_triangles)):
            ts=[t for t in mesh.loop_triangles if t.material_index==mi]
            key=material(mesh.materials[mi] if mi<len(mesh.materials) else None)
            ident=f'{name}_{mesh_count:03d}';mesh_count+=1
            coords=[];uvs=[];normals=[]
            for t in ts:
                for vi,li in zip(t.vertices,t.loops):
                    coords.append(mesh.vertices[vi].co[:]);normals.append(mesh.corner_normals[li].vector[:]);uvs.append(uv[li].uv[:] if uv else (0,0))
            a=np.array(coords,dtype=np.float32);u=np.array(uvs,dtype=np.float32);n=np.array(normals,dtype=np.float32)
            np.savez_compressed(MESH/(ident+'.npz'),vertices=a,uv=u,normals=n)
            with open(MESH/(ident+'.obj'),'w') as f:
                f.write('# NASA-derived PROMISE simulation visual, meters, FLU, link-local\n')
                for v in a:f.write('v %.7g %.7g %.7g\n'%tuple(v))
                for v in u:f.write('vt %.7g %.7g\n'%tuple(v))
                for v in n:f.write('vn %.7g %.7g %.7g\n'%tuple(v))
                for k in range(1,len(a)+1,3):f.write('f '+' '.join(f'{i}/{i}/{i}' for i in range(k,k+3))+'\n')
            links[name]['visuals'].append(dict(mesh=ident,material=key,triangles=len(ts)))

spec=dict(name='promise',revision='0.3.0',date='2026-11-14',units='m',axes='X forward, Y left, Z up',base_height=.9,total_mass_kg=sum(l['mass'] for l in links.values()),links=links,materials=materials,cameras=cameras,provenance=provenance,notes=['Masses, inertias, joint limits, actuator gains, friction and optical calibration are estimates, not NASA flight parameters.','Latest public PROMISE visualization used for mast/payload geometry. TRIDENT is fixed in its stowed driving pose.','Rocker differential uses q_left + q_right = 0.','Tread geometry is visual; wheel contact is a smooth cylinder. No deformable regolith model.'])
(ROOT/'data/robot_spec.json').write_text(json.dumps(spec,indent=2))
spec['lidar']=lidar;spec['notes'].append('OS1 Rev8 is a user-requested turret sensor substitution, not a NASA flight hardware claim.')
(ROOT/'data/robot_spec.json').write_text(json.dumps(spec,indent=2))
(ROOT/'data/os1_rev8_profile.json').write_text(json.dumps(lidar,indent=2))

# All six wheels have a genuine looping animation around their own local Y axes.
scene=bpy.context.scene;scene.frame_start=1;scene.frame_end=240;scene.render.fps=30
for name,e in empties.items():
    if not name.endswith('_wheel'):continue
    e.rotation_mode='XYZ';e.rotation_euler=(0,0,0);e.keyframe_insert(data_path='rotation_euler',index=1,frame=1)
    e.rotation_euler.y=2*math.pi;e.keyframe_insert(data_path='rotation_euler',index=1,frame=241)
    for fc in e.animation_data.action.fcurves:
        for k in fc.keyframe_points:k.interpolation='LINEAR'
        fc.modifiers.new('CYCLES')
scene.frame_set(1)
for o in bpy.data.objects:o.select_set(o in empties.values() or o in sensor_objects or any(o in g for g in groups.values()))
bpy.context.view_layer.objects.active=empties['base_link']
scene.render.resolution_x=1280;scene.render.resolution_y=720;scene.render.resolution_percentage=100
bpy.ops.export_scene.gltf(filepath=str(ROOT/'glb/promise.glb'),export_format='GLB',use_selection=True,export_animations=True,export_animation_mode='ACTIVE_ACTIONS',export_frame_range=True,export_force_sampling=True,export_yup=True,export_extras=True,export_cameras=True)

# Studio context is excluded from the general-purpose GLB and simulator geometry.
scene.world=bpy.data.worlds.new('PROMISE studio');scene.world.use_nodes=True
bg=scene.world.node_tree.nodes['Background'];bg.inputs[0].default_value=(.19,.23,.30,1);bg.inputs[1].default_value=.45
for pos,power,size in [((4,-4,6),1400,5),((1,5,4),1700,4),((-4,-2,4),1700,4)]:
    bpy.ops.object.light_add(type='AREA',location=pos);o=bpy.context.object;o.name='Studio_Area';o.data.energy=power;o.data.shape='DISK';o.data.size=size;o.rotation_euler=(Vector((0,0,1))-o.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(5,-5,3.6));cam=bpy.context.object;cam.name='Presentation_Camera';cam.rotation_euler=(Vector((0,0,1))-cam.location).to_track_quat('-Z','Y').to_euler();cam.data.type='ORTHO';cam.data.ortho_scale=4.7;scene.camera=cam
scene.render.engine='CYCLES';scene.cycles.samples=24;scene.cycles.use_denoising=True
scene.view_settings.view_transform='AgX';scene.view_settings.look='AgX - Medium High Contrast';scene.view_settings.exposure=-.4
scene.render.resolution_x=1600;scene.render.resolution_y=1200;scene.render.resolution_percentage=100
scene.render.film_transparent=True;scene.render.image_settings.file_format='PNG'
scene.render.filepath=str(ROOT/'previews/promise_rover.png')
scene['README']='PROMISE reconstruction. Press Space to play six independently pivoted wheels. 1–240: eight-second loop. All physics parameters are estimates. See ../README.md.'
for im in list(bpy.data.images):
    if not im.has_data and not im.packed_file:bpy.data.images.remove(im)
bpy.ops.file.pack_all()
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'blender/promise_wheel_animation.blend'),compress=True)
bpy.ops.render.render(write_still=True)
print('GEOMETRY_COMPLETE',len(links),mesh_count,sum(v['triangles'] for l in links.values() for v in l['visuals']),flush=True)
