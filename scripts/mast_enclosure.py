"""Replace only the donor's solid white box with a thin-walled, open-front shell.
The original six-sided footprint and all camera geometry are retained.
"""
import math
import bpy, bmesh, numpy as np


def rebuild_enclosure(head, material, make_box):
    adjacency=[set() for _ in head.data.vertices]
    for edge in head.data.edges:
        a,b=edge.vertices;adjacency[a].add(b);adjacency[b].add(a)
    remaining=set(range(len(adjacency)));candidates=[]
    while remaining:
        first=remaining.pop();found={first};stack=[first]
        while stack:
            for v in adjacency[stack.pop()]:
                if v in remaining:remaining.remove(v);found.add(v);stack.append(v)
        p=np.array([head.data.vertices[v].co[:] for v in found])
        if np.ptp(p[:,1])>.4 and .09<np.ptp(p[:,2])<.12:
            candidates.append((found,p))
    assert len(candidates)==1,'Expected exactly one white enclosure island'
    indices,points=candidates[0];lo=points.min(0);hi=points.max(0)
    footprint=np.unique(np.round(points[:,:2],6),axis=0)
    center=footprint.mean(0);footprint=footprint[np.argsort(np.arctan2(footprint[:,1]-center[1],footprint[:,0]-center[0]))]
    bm=bmesh.new();bm.from_mesh(head.data);bm.verts.ensure_lookup_table()
    bmesh.ops.delete(bm,geom=[bm.verts[i] for i in indices],context='VERTS')
    bm.to_mesh(head.data);bm.free();head.data.update()
    wall=.003;front=float(hi[0]);ycenter=-.164
    opening=[front,ycenter,1.979];width=.150;bottom=1.933;top=2.025
    objects=[];collisions=[]

    def prism(name,polygon,zlo,zhi):
        n=len(polygon);v=[(float(x),float(y),float(z)) for z in (zlo,zhi) for x,y in polygon]
        faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
        mesh=bpy.data.meshes.new(name);mesh.from_pydata(v,[],faces);mesh.update()
        bm=bmesh.new();bm.from_mesh(mesh);bmesh.ops.recalc_face_normals(bm,faces=bm.faces);bm.to_mesh(mesh);bm.free()
        o=bpy.data.objects.new(name,mesh);bpy.context.collection.objects.link(o);mesh.materials.append(material);objects.append(o)
        return o

    def add_box(name,pos,size,rpy=None):
        o=make_box(name,pos,size,material,bevel=.0003);objects.append(o)
        collisions.append(dict(type='box',pos=list(pos),size=list(size),rpy=rpy or [0,0,0]))
        return o

    prism('Mast_Enclosure_floor',footprint,lo[2],lo[2]+wall)
    prism('Mast_Enclosure_roof',footprint,hi[2]-wall,hi[2])
    for name,z in [('floor',lo[2]+wall/2),('roof',hi[2]-wall/2)]:
        collisions.append(dict(type='box',pos=[float((lo[0]+hi[0])/2),float((lo[1]+hi[1])/2),float(z)],size=[float(hi[0]-lo[0]),float(hi[1]-lo[1]),wall],rpy=[0,0,0]))
    for i,a in enumerate(footprint):
        b=footprint[(i+1)%len(footprint)];delta=b-a;length=np.linalg.norm(delta)
        if abs(a[0]-front)<1e-5 and abs(b[0]-front)<1e-5:continue
        normal=np.array([-delta[1],delta[0]])/length
        polygon=[a,b,b+normal*wall,a+normal*wall]
        prism('Mast_Enclosure_wall_'+str(i),polygon,lo[2],hi[2])
        mid=(a+b+normal*wall)/2
        collisions.append(dict(type='box',pos=[float(mid[0]),float(mid[1]),float((lo[2]+hi[2])/2)],size=[float(length),wall,float(hi[2]-lo[2])],rpy=[0,0,float(math.atan2(delta[1],delta[0]))]))
    yl=ycenter-width/2;yr=ycenter+width/2;xm=front-wall/2
    for name,a,b in [('left',lo[1],yl),('right',yr,hi[1])]:
        add_box('Mast_Enclosure_front_'+name,[xm,(a+b)/2,(lo[2]+hi[2])/2],[wall,b-a,hi[2]-lo[2]])
    for name,a,b in [('bottom',lo[2],bottom),('top',top,hi[2])]:
        add_box('Mast_Enclosure_front_'+name,[xm,ycenter,(a+b)/2],[wall,width,b-a])
    info={'outer_bounds_m':[lo.tolist(),hi.tolist()],'wall_thickness_m':wall,'front_plane_x_m':front,'opening_width_m':width,'opening_bottom_z_m':bottom,'opening_top_z_m':top,'opening_center_y_m':ycenter,'note':'True open aperture and hollow cavity; no glass cover. Original enclosure footprint and four camera meshes retained.'}
    return objects,collisions,info
