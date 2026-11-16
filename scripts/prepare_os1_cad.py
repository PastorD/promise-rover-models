"""Tessellate the physical case of Ouster's Rev8 XX STEP, excluding its FOV solid."""
from pathlib import Path
import json,numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE,TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
ROOT=Path(__file__).resolve().parents[1]
source=next((ROOT/'source_assets/os1_rev8_cad').rglob('*OS1 XX*.STEP'))
r=STEPControl_Reader();r.ReadFile(str(source));r.TransferRoots()
e=TopExp_Explorer(r.OneShape(),TopAbs_SOLID);triangles=[];records=[]
while e.More():
    solid=e.Current();box=Bnd_Box();BRepBndLib.Add_s(solid,box);p,q=box.CornerMin(),box.CornerMax()
    bounds=np.array([[p.X(),p.Y(),p.Z()],[q.X(),q.Y(),q.Z()]]);size=bounds[1]-bounds[0]
    physical=max(size[0],size[2])<95
    records.append({'bounds_mm':bounds.tolist(),'included':bool(physical)})
    if physical:
        BRepMesh_IncrementalMesh(solid,.15,False,.15,True);faces=TopExp_Explorer(solid,TopAbs_FACE)
        while faces.More():
            face=TopoDS.Face(faces.Current());loc=TopLoc_Location();mesh=BRep_Tool.Triangulation_s(face,loc)
            if mesh:
                for i in range(1,mesh.NbTriangles()+1):
                    triangle=mesh.Triangle(i);indices=[triangle.Value(j) for j in (1,2,3)]
                    if face.Orientation()==TopAbs_REVERSED:indices[1],indices[2]=indices[2],indices[1]
                    points=[mesh.Node(j).Transformed(loc.Transformation()) for j in indices]
                    triangles.append([[v.Z()/1000,v.X()/1000,v.Y()/1000] for v in points])
            faces.Next()
    e.Next()
vertices=np.asarray(triangles,dtype=np.float32);assert vertices.size and vertices[:,:,2].max()<.059
np.savez_compressed(ROOT/'source_assets/os1_rev8_geometry.npz',triangles=vertices)
(ROOT/'references/os1_cad_conversion.json').write_text(json.dumps({'source':str(source.relative_to(ROOT)),'solids':records,'triangles':len(vertices),'bounds_m':[vertices.min((0,1)).tolist(),vertices.max((0,1)).tolist()],'note':'Physical Ouster CAD; illustrative materials and turret mounting bracket.'},indent=2))
print('OS1 CAD triangles',len(vertices),flush=True)
