import bpy, json, pathlib, sys
from mathutils import Vector

ROOT=pathlib.Path(__file__).resolve().parents[1]
for label,path in [
    ('curiosity', ROOT/'source_assets/curiosity_clean/Curiosity Rover (MSL) (Clean).blend'),
    ('perseverance', ROOT/'source_assets/perseverance_blend/Mars 2020 Perseverance Rover.blend'),
    ('promise', ROOT/'source_assets/promise_nasa.glb')]:
    if path.suffix=='.blend': bpy.ops.wm.open_mainfile(filepath=str(path))
    else:
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.ops.import_scene.gltf(filepath=str(path))
    bpy.context.view_layer.update()
    records=[]
    for o in bpy.data.objects:
        if o.type!='MESH':continue
        pts=[o.matrix_world @ Vector(x) for x in o.bound_box]
        mn=[min(v[i] for v in pts) for i in range(3)]
        mx=[max(v[i] for v in pts) for i in range(3)]
        rec=dict(name=o.name,parent=o.parent.name if o.parent else None,
                 vertices=len(o.data.vertices),polygons=len(o.data.polygons),
                 origin=list(o.matrix_world.translation),bounds=[mn,mx],
                 materials=[x.name if x else None for x in o.data.materials])
        records.append(rec)
        print(label,o.name,rec['vertices'],[[round(y,3) for y in x] for x in [mn,mx]],flush=True)
    (ROOT/'references'/f'{label}_objects.json').write_text(json.dumps(records,indent=2))
    if label=='promise':bpy.ops.wm.save_as_mainfile(filepath=str(ROOT/'source_assets/promise_reference.blend'))
