"""Articulate NASA's public PROMISE model, add the recessed OS1 lidar, and export visuals.
Run with Blender 4.5's Python (bpy), see README. Units are meters, ROS FLU.
"""
import bpy, bmesh, math, json, pathlib, re, sys, numpy as np
from PIL import Image
from mathutils import Matrix, Vector
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

ROOT=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
PKG=ROOT/'ros2/promise_description'
MESH=PKG/'meshes'; TEX=PKG/'textures'
for p in [MESH,ROOT/'blender',ROOT/'glb',ROOT/'previews',ROOT/'data']:p.mkdir(parents=True,exist_ok=True)
# Part numbering follows each build; never leave a previous build's visuals behind.
for p in MESH.iterdir():
    if p.suffix in ('.obj','.npz','.dae'):p.unlink()
R=Matrix.Rotation(math.pi/2,4,'Z') # NASA -Y forward -> ROS +X forward
# NASA's corner normals ride along as a mesh attribute. Blender's encoded custom
# normals drift when a mesh is edited, so they are only set again before export.
NORMAL='nasa_normal'
def newmat(name,rgba,metal=0,rough=.45,emission=0):
    m=bpy.data.materials.new(name);m.use_nodes=True;m.diffuse_color=rgba
    p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=rgba;p.inputs['Metallic'].default_value=metal;p.inputs['Roughness'].default_value=rough
    if emission:p.inputs['Emission Color'].default_value=rgba;p.inputs['Emission Strength'].default_value=emission
    return m
def simplify(o,target=5000):
    # Merge duplicated vertices, then decimate unless that moves the bounds.
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
def box(name,pos,size,mat,bevel=.01):
    bpy.ops.mesh.primitive_cube_add(size=1,location=pos);o=bpy.context.object;o.name=name;o.dimensions=size
    bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
    o.data.materials.append(mat)
    if bevel:
        m=o.modifiers.new('Machined edges','BEVEL');m.width=bevel;m.segments=2;bpy.ops.object.modifier_apply(modifier=m.name)
    return o

def nasa_triangles(o):
    """Triangle corners, unit corner normals and material names in ROS FLU."""
    me=o.data;me.calc_loop_triangles();n=len(me.loop_triangles)
    co=np.empty(len(me.vertices)*3);me.vertices.foreach_get('co',co)
    tv=np.empty(n*3,dtype=np.int64);me.loop_triangles.foreach_get('vertices',tv)
    tl=np.empty(n*3,dtype=np.int64);me.loop_triangles.foreach_get('loops',tl)
    mi=np.empty(n,dtype=np.int64);me.loop_triangles.foreach_get('material_index',mi)
    cn=np.empty(len(me.loops)*3);me.corner_normals.foreach_get('vector',cn)
    M=np.array(R@o.matrix_world);L=M[:3,:3];assert np.linalg.det(L)>0
    p=(co.reshape(-1,3)[tv]@L.T+M[:3,3]).reshape(-1,3,3)
    nr=(cn.reshape(-1,3)[tl]@np.linalg.inv(L)).reshape(-1,3,3)
    # A few NASA corner normals are NaN or zero; use the face normal there.
    face=np.cross(p[:,1]-p[:,0],p[:,2]-p[:,0]);face/=np.maximum(np.linalg.norm(face,axis=1,keepdims=True),1e-30)
    length=np.linalg.norm(nr,axis=2,keepdims=True);bad=~(length>1e-6)
    nr=np.where(bad,face[:,None],nr/np.where(bad,1,length))
    return dict(p=p,n=nr,m=np.array([me.materials[i].name for i in mi]))
def take(t,mask):return {k:v[mask] for k,v in t.items()}
def weld(p):
    """Index triangle corners by position on a 10 µm grid."""
    key=np.round(p.reshape(-1,3)/1e-5).astype(np.int64)
    _,first,index=np.unique(key,axis=0,return_index=True,return_inverse=True)
    return p.reshape(-1,3)[first],index.reshape(-1,3)
def islands(index):
    n=int(index.max())+1;a=np.r_[index[:,0],index[:,1]];b=np.r_[index[:,1],index[:,2]]
    return connected_components(coo_matrix((np.ones(len(a)),(a,b)),shape=(n,n)),directed=False)[1][index[:,0]]
def make_object(name,t):
    """Blender mesh with welded vertices, NASA's materials and NASA's corner normals."""
    verts,index=weld(t['p'])
    keep=(index[:,0]!=index[:,1])&(index[:,1]!=index[:,2])&(index[:,0]!=index[:,2]) # zero-area slivers
    index=index[keep];normals=t['n'][keep];names=t['m'][keep]
    # Blender rejects repeated faces; give exact duplicates their own vertices.
    _,unique=np.unique(np.sort(index,axis=1),axis=0,return_index=True);repeat=np.setdiff1d(np.arange(len(index)),unique)
    if len(repeat):
        extra=verts[index[repeat]].reshape(-1,3);index[repeat]=len(verts)+np.arange(len(extra)).reshape(-1,3);verts=np.vstack([verts,extra])
    me=bpy.data.meshes.new(name);me.vertices.add(len(verts));me.vertices.foreach_set('co',verts.ravel())
    me.loops.add(index.size);me.loops.foreach_set('vertex_index',index.ravel().astype(np.int32))
    me.polygons.add(len(index));me.polygons.foreach_set('loop_start',np.arange(0,index.size,3,dtype=np.int32))
    order=sorted(set(names.tolist()))
    for n in order:me.materials.append(bpy.data.materials[n])
    me.polygons.foreach_set('material_index',np.searchsorted(order,names).astype(np.int32))
    me.update(calc_edges=True);assert not me.validate(),name
    me.polygons.foreach_set('use_smooth',np.ones(len(index),dtype=bool))
    me.attributes.new(NORMAL,'FLOAT_VECTOR','CORNER').data.foreach_set('vector',normals.astype(np.float32).ravel())
    o=bpy.data.objects.new(name,me);bpy.context.collection.objects.link(o);return o
def transform(o,M):
    """Apply M to the mesh, rotating the stored NASA normals with it."""
    o.data.transform(M);a=o.data.attributes.get(NORMAL)
    if a and M.to_3x3()!=Matrix.Identity(3):
        n=np.empty(len(a.data)*3,dtype=np.float32);a.data.foreach_get('vector',n)
        n=n.reshape(-1,3)@np.linalg.inv(np.array(M.to_3x3()));n/=np.linalg.norm(n,axis=1,keepdims=True);a.data.foreach_set('vector',n.ravel())
def corner_normals(mesh):
    a=mesh.attributes.get(NORMAL);n=np.empty(len(mesh.loops)*3,dtype=np.float32)
    (a.data if a else mesh.corner_normals).foreach_get('vector',n);return n.reshape(-1,3)
def restore_normals(o):
    """Set NASA's corner normals as Blender custom normals and drop the helper attribute."""
    if o.type!='MESH' or NORMAL not in o.data.attributes:return
    n=corner_normals(o.data);o.data.attributes.remove(o.data.attributes[NORMAL]);o.data.normals_split_custom_set(n)

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(ROOT/'source_assets/promise_nasa.glb'));bpy.context.view_layer.update()
source={o.name:nasa_triangles(o) for o in bpy.data.objects if o.type=='MESH'}
for o in list(bpy.data.objects):bpy.data.objects.remove(o,do_unlink=True)
for m in list(bpy.data.meshes):bpy.data.meshes.remove(m)
assert len(source)==27,sorted(source)
nasa_names=sorted(source)

links={}; groups={}; provenance=[]
def link(name,anchor,parent=None,joint=None,axis=None,kind='fixed',mass=1,limits=None,mimic=None):
    links[name]=dict(name=name,anchor=list(anchor),parent=parent,joint=joint,axis=axis,type=kind,mass=mass,limits=limits,mimic=mimic,collisions=[])
    groups[name]=[]
link('base_link',[0,0,.9],mass=760)
# Joint axes fitted to concentric bearing rings in NASA's hub geometry (<1 mm residual).
ROCKER_PIVOT=(.214,.894);BOGIE_PIVOT=(-.540,.664) # x,z; both turn about ROS +Y
STEER_AXES={'front_left':(1.0969,1.0648),'rear_left':(-1.1649,1.0625),'front_right':(1.0951,-1.0618),'rear_right':(-1.1667,-1.0618)}
STEER_Z=.64
for side,sgn in [('left',1),('right',-1)]:
    link('rocker_'+side,[ROCKER_PIVOT[0],sgn*.80,ROCKER_PIVOT[1]],'base_link','rocker_'+side+'_joint',[0,1,0],'revolute',20,[-.65,.65],{'joint':'rocker_left_joint','multiplier':-1} if side=='right' else None)
    link('bogie_'+side,[BOGIE_PIVOT[0],sgn*.80,BOGIE_PIVOT[1]],'rocker_'+side,'bogie_'+side+'_joint',[0,1,0],'revolute',15,[-.9,.9])
    for which in ['front','rear']:
        name=f'{which}_{side}_steer';x,y=STEER_AXES[f'{which}_{side}']
        link(name,[x,y,STEER_Z],'rocker_'+side if which=='front' else 'bogie_'+side,name+'_joint',[0,0,1],'revolute',6,[-1.4,1.4])

# NASA models each side's rocker, bogie, cabling and differential hub as one mesh.
# Faces follow the nearer tube centerline in side view; loose parts move whole with
# their majority, and the pivot hub between body and rocker stays on the body.
def segment_distance(p,a,b):
    ab=b-a;t=np.clip(((p-a)@ab)/(ab@ab),0,1);return np.linalg.norm(p-(a+t[:,None]*ab),axis=1)
def split_suspension(t):
    c=t['p'].mean(1);xz=c[:,[0,2]];rp=np.array(ROCKER_PIVOT);bp=np.array(BOGIE_PIVOT)
    front=np.array([STEER_AXES['front_left'][0],.73]);rear=np.array([STEER_AXES['rear_left'][0],.73])
    rocker=np.min([segment_distance(xz,rp,front),segment_distance(xz,rp,bp),segment_distance(xz,rp,rp+[0,.3])],axis=0)
    bogie=np.min([segment_distance(xz,bp,rear),segment_distance(xz,bp,np.array([-.090,.264]))],axis=0)
    label=np.where(rocker<=bogie,1,2);final=label.copy()
    area=.5*np.linalg.norm(np.cross(t['p'][:,1]-t['p'][:,0],t['p'][:,2]-t['p'][:,0]),axis=1)
    part=islands(weld(t['p'])[1]);order=np.argsort(part,kind='stable');bounds=np.flatnonzero(np.diff(part[order]))+1
    for f in np.split(order,bounds):
        if np.linalg.norm(xz[f].mean(0)-rp)<.10 and np.abs(c[f,1]).max()<.72:final[f]=0;continue
        w=np.bincount(label[f],weights=area[f],minlength=3);top=int(w.argmax())
        if len(f)<400 or w[top]>=.85*w.sum():final[f]=top
    return final
for source_name in ['Mesh_11','Mesh_12']:
    t=source.pop(source_name);side='left' if t['p'][:,:,1].mean()>0 else 'right';label=split_suspension(t)
    for k,target in enumerate(['base_link','rocker_'+side,'bogie_'+side]):
        groups[target].append(make_object(f'PROMISE_{source_name}_{target}',take(t,label==k)))
for source_name in ['Mesh_13','Mesh_14','Mesh_15','Mesh_16']:
    t=source.pop(source_name);c=t['p'].reshape(-1,3).mean(0)
    corner=min(STEER_AXES,key=lambda k:math.hypot(STEER_AXES[k][0]-c[0],STEER_AXES[k][1]-c[1]))
    assert not groups[corner+'_steer'];groups[corner+'_steer'].append(make_object('PROMISE_'+source_name,t))

def fit_wheel(t):
    """Spin centre, grouser-tip radius and width; turns the wheel so its axle is exactly
    ROS +Y. NASA's wheel axles lean 0.5-0.7 degrees, which would wobble about a +Y joint."""
    p=t['p'].reshape(-1,3);mean=p.mean(0)
    axle=np.linalg.eigh(np.cov((p-mean).T))[1][:,0];axle*=np.sign(axle[1])
    rot=np.array(Vector(axle).rotation_difference(Vector((0,1,0))).to_matrix());q=(p-mean)@rot.T
    r=np.hypot(q[:,0],q[:,2]);tread=q[r>np.percentile(r,90)]
    cx,cz,_=np.linalg.lstsq(np.c_[2*tread[:,0],2*tread[:,2],np.ones(len(tread))],tread[:,0]**2+tread[:,2]**2,rcond=None)[0]
    hub=np.array([cx,(q[:,1].min()+q[:,1].max())/2,cz]);centre=mean+rot.T@hub
    t['p']=(q-hub+centre).reshape(-1,3,3);t['n']=t['n']@rot.T
    return centre,float(np.hypot(q[:,0]-cx,q[:,2]-cz).max()),float(np.ptp(q[:,1]))
fits={}
for source_name in ['Mesh_5','Mesh_6','Mesh_7','Mesh_8','Mesh_9','Mesh_10']:
    t=source.pop(source_name);centre,radius,width=fit_wheel(t)
    key=('front' if centre[0]>.5 else 'rear' if centre[0]<-.5 else 'middle')+'_'+('left' if centre[1]>0 else 'right')
    assert key not in fits;fits[key]=(source_name,t,centre,radius,width)
for side in ['left','right']:
    for which in ['front','middle','rear']:
        source_name,t,centre,radius,width=fits[f'{which}_{side}'];name=f'{which}_{side}_wheel'
        parent='bogie_'+side if which=='middle' else f'{which}_{side}_steer'
        link(name,centre.tolist(),parent,name+'_joint',[0,1,0],'continuous',6)
        groups[name]=[make_object('PROMISE_'+source_name,t)]
        links[name]['wheel_radius']=radius;links[name]['wheel_width']=width

t=source.pop('Mesh_20');z=t['p'].mean(1)[:,2]
mast=make_object('PROMISE_compact_mast',take(t,z<1.875));head=make_object('PROMISE_compact_head',take(t,z>=1.875))
link('mast_yaw_link',[.679,-.164,1.17],'base_link','mast_yaw_joint',[0,0,1],'revolute',6,[-math.pi,math.pi])
link('head_pitch_link',[.679,-.164,1.89],'mast_yaw_link','head_pitch_joint',[0,1,0],'revolute',4,[-.65,.65])
groups['mast_yaw_link']=[mast];groups['head_pitch_link']=[head]
for source_name,t in sorted(source.items()):groups['base_link'].append(make_object('PROMISE_'+source_name,t))
provenance.append(dict(source='NASA PROMISE interactive GLB, November 7 2026 release',objects=nasa_names,operation='All visible geometry, at full resolution. Rotate to ROS FLU and keep NASA materials and corner normals. Split the one-piece rocker-bogie meshes into body, rocker and bogie parts and the mast at z=1.875 m into yaw and pitch parts. Joint axes fitted to NASA hub bearing rings; wheels turned 0.5-0.7 degrees so their axles match the +Y spin joints.'))

dark=newmat('PROMISE_black_anodized',(.045,.055,.065,1),.7,.35)
silver=newmat('PROMISE_silver',(.52,.55,.58,1),.8,.3)

# User-requested recessed OS1 mount: a hollow enclosure with a front aperture,
# finished like NASA's white head box (Material_3).
from mast_enclosure import rebuild_enclosure
shell,head_collision_world,enclosure=rebuild_enclosure(groups['head_pitch_link'][0],bpy.data.materials['Material_3'],box)
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
        transform(o,orientation.inverted()@Matrix.Translation(-Vector(spec['anchor'])))
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
# Chassis and rear power unit, sized from NASA's body mesh.
collision('base_link','box',[-.11,0,.94],[2.03,1.20,.53])
collision('base_link','box',[-1.43,0,1.24],[.46,.84,.58])
links['os1_link']['collisions'].append(dict(type='cylinder',pos=[0,0,.029175],radius=.0435,length=.05835))
for name,spec in links.items():
    a=spec['anchor']
    if name.endswith('_wheel'):collision(name,'cylinder',a,radius=spec['wheel_radius'],length=spec['wheel_width'],rpy=[math.pi/2,0,0])
    elif name.endswith('_steer'):collision(name,'box',[a[0],a[1]-.09*(1 if 'left' in name else -1),a[2]-.18],[.12,.15,.35])
    elif name.startswith('rocker_') or name.startswith('bogie_'):
        # Collision beams approximate the tubing; visual meshes are never contact hulls.
        side=name.split('_')[1];steer=lambda which:[*STEER_AXES[which+'_'+side],.665]
        if name.startswith('rocker_'):ends=[(a,steer('front')),(a,links['bogie_'+side]['anchor'])]
        else:ends=[(a,links['middle_'+side+'_wheel']['anchor']),(a,steer('rear'))]
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
        com=np.zeros(3);r=spec['wheel_radius'];w=spec['wheel_width'];I=[m*(3*r*r+w*w)/12,m*r*r/2,m*(3*r*r+w*w)/12]
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

# Save indexed single-material geometry for MJCF/USD/Collada exports.
materials={};image_cache={};mesh_count=0
def slug(s):return re.sub('[^a-zA-Z0-9_]+','_',s).strip('_')
def imagefile(im):
    if im.name in image_cache:return image_cache[im.name]
    TEX.mkdir(exist_ok=True);name=slug(im.name)+'.png';path=TEX/name
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
    color=list(m.diffuse_color);metal=0.;rough=.5;emission=0.;texture=None
    p=next((x for x in m.node_tree.nodes if x.type=='BSDF_PRINCIPLED'),None) if m.use_nodes else None
    if p:
        color=list(p.inputs['Base Color'].default_value);metal=float(p.inputs['Metallic'].default_value);rough=float(p.inputs['Roughness'].default_value)
        if p.inputs['Emission Strength'].default_value>0 and max(p.inputs['Emission Color'].default_value[:3])>0:emission=1.
        sock=p.inputs['Base Color']
        if sock.is_linked:
            n=sock.links[0].from_node
            if n.type=='TEX_IMAGE' and n.image:texture=imagefile(n.image)
    materials[key]=dict(rgba=color,metallic=metal,roughness=rough,emission=emission,texture=texture)
    return key
def write_obj(path,part):
    lines=['# NASA-derived PROMISE simulation visual, meters, FLU, link-local']
    lines+=['v %.7g %.7g %.7g'%tuple(v) for v in part['positions'].tolist()]
    if 'uv' in part:lines+=['vt %.7g %.7g'%tuple(v) for v in part['uv'].tolist()]
    lines+=['vn %.4g %.4g %.4g'%tuple(v) for v in part['normals'].tolist()]
    p=part['position_index']+1;n=part['normal_index']+1
    if 'uv' in part:
        t=part['uv_index']+1;lines+=['f %d/%d/%d %d/%d/%d %d/%d/%d'%tuple(c) for c in np.stack([p,t,n],2).reshape(-1,9).tolist()]
    else:lines+=['f %d//%d %d//%d %d//%d'%tuple(c) for c in np.stack([p,n],2).reshape(-1,6).tolist()]
    path.write_text('\n'.join(lines)+'\n')
materials['default']=dict(rgba=[.7,.7,.7,1],metallic=0,roughness=.5,emission=0,texture=None)
for name in links:
    for o in groups[name]:
        mesh=o.data;mesh.calc_loop_triangles();n=len(mesh.loop_triangles)
        co=np.empty(len(mesh.vertices)*3,dtype=np.float32);mesh.vertices.foreach_get('co',co);co=co.reshape(-1,3)
        tv=np.empty(n*3,dtype=np.int64);mesh.loop_triangles.foreach_get('vertices',tv);tv=tv.reshape(-1,3)
        tl=np.empty(n*3,dtype=np.int64);mesh.loop_triangles.foreach_get('loops',tl);tl=tl.reshape(-1,3)
        mi=np.empty(n,dtype=np.int64);mesh.loop_triangles.foreach_get('material_index',mi)
        cn=corner_normals(mesh);uv=None
        if mesh.uv_layers.active:uv=np.empty(len(mesh.loops)*2,dtype=np.float32);mesh.uv_layers.active.data.foreach_get('uv',uv);uv=uv.reshape(-1,2)
        for k in np.unique(mi):
            sel=mi==k;key=material(mesh.materials[k] if k<len(mesh.materials) else None)
            ident=f'{name}_{mesh_count:03d}';mesh_count+=1
            used,pi=np.unique(tv[sel],return_inverse=True);normals,ni=np.unique(cn[tl[sel]].reshape(-1,3),axis=0,return_inverse=True)
            part=dict(positions=co[used],position_index=pi.reshape(-1,3).astype(np.int32),normals=normals,normal_index=ni.reshape(-1,3).astype(np.int32))
            if materials[key]['texture'] and uv is not None:
                uvs,ui=np.unique(uv[tl[sel]].reshape(-1,2),axis=0,return_inverse=True);part.update(uv=uvs,uv_index=ui.reshape(-1,3).astype(np.int32))
            np.savez_compressed(MESH/(ident+'.npz'),**part);write_obj(MESH/(ident+'.obj'),part)
            links[name]['visuals'].append(dict(mesh=ident,material=key,triangles=int(sel.sum())))

spec=dict(name='promise',revision='0.4.0',date='2026-11-16',units='m',axes='X forward, Y left, Z up',base_height=.9,total_mass_kg=sum(l['mass'] for l in links.values()),links=links,materials=materials,cameras=cameras,provenance=provenance,notes=['Masses, inertias, joint limits, actuator gains, friction and optical calibration are estimates, not NASA flight parameters.','All visible rover geometry is NASA\'s public PROMISE visualization. TRIDENT is fixed in its stowed driving pose.','Rocker differential uses q_left + q_right = 0.','Tread geometry is visual; wheel contact is a smooth cylinder. No deformable regolith model.'])
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
for g in groups.values():
    for o in g:restore_normals(o)
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
scene.render.filepath='//../previews/promise_rover.png' # relative, so the .blend holds no build paths
scene.render.use_stamp_filename=False # keep the local .blend path out of the PNG metadata
scene['README']='PROMISE reconstruction. Press Space to play six independently pivoted wheels. 1–240: eight-second loop. All physics parameters are estimates. See ../README.md.'
for im in list(bpy.data.images):
    if not im.has_data and not im.packed_file:bpy.data.images.remove(im)
bpy.ops.file.pack_all()
# Workspace file browsers default to the user's home folder; keep it out of the file.
for screen in bpy.data.screens:
    for area in screen.areas:
        for space in area.spaces:
            if space.type=='FILE_BROWSER' and space.params:space.params.directory=b'//'
bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'blender/promise_wheel_animation.blend'),compress=True,relative_remap=True)
bpy.ops.render.render(write_still=True)
print('GEOMETRY_COMPLETE',len(links),mesh_count,sum(v['triangles'] for l in links.values() for v in l['visuals']),flush=True)
