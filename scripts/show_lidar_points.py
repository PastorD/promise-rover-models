"""Display a PROMISE OS1 scan in Blender 4.3+ without VisionSIM.

Open the sensor-enabled rover .blend, open this file in Blender's Text Editor,
and click Run Script. Requires the companion blender_sensors.py/sensor_rays.py
for a fresh scan; saved NPZ captures need only this script and Blender's NumPy.
Creates a static snapshot; rerun after moving the rover, turret, or environment.
Hide OS1_Points from render or remove it before generating sensor datasets with
other capture scripts; it is display geometry. This viewer excludes its own
display geometry when rescanning.
"""
from pathlib import Path
import argparse
import sys
import bpy
import numpy as np

# Edit these settings in Blender, then Run Script again.
NPZ_PATH = ""               # Empty: scan current scene. Or "/path/to/000001.npz".
ADD_DEMO_WORLD = False     # True: add a ground plane and two test objects.
POINT_RADIUS = 0.015       # Display sphere radius in meters; not sensor accuracy.
DISPLAY_STRIDE = 4         # Show every 4th valid return; 1 displays every return.
MAX_DISPLAY_RANGE = 30.0   # Display cutoff in meters; 0 displays the whole scan.

TAG = "promise_os1_point_display"


def scanner_module():
    candidates = []
    if "__file__" in globals():
        candidates.append(Path(__file__).resolve().parent)
    if bpy.data.filepath:
        folder = Path(bpy.data.filepath).resolve().parent
        candidates.extend([folder.parent / "scripts", folder / "scripts"])
    for folder in candidates:
        if (folder / "blender_sensors.py").is_file():
            sys.path.insert(0, str(folder))
            try:
                import blender_sensors
                return blender_sensors
            finally:
                sys.path.pop(0)
    raise RuntimeError(
        "Extract promise_blender_sensors.zip and open its blender/"
        "promise_wheel_animation.blend, keeping the scripts folder beside blender. "
        "Alternatively set NPZ_PATH to an existing lidar capture."
    )


def show_lidar_points(npz_path=NPZ_PATH, demo=ADD_DEMO_WORLD,
                      radius=POINT_RADIUS, stride=DISPLAY_STRIDE,
                      max_range=MAX_DISPLAY_RANGE):
    if radius <= 0 or stride < 1 or max_range < 0:
        raise ValueError("Use a positive radius/stride and a nonnegative range.")
    previous = [o for o in bpy.data.objects if o.get(TAG, False)]
    visibility = [(o, o.hide_render) for o in previous]
    # The display must not become an obstacle in the next ray cast.
    for o in previous:
        o.hide_render = True
    try:
        if npz_path:
            path = Path(bpy.path.abspath(str(npz_path))).expanduser()
            with np.load(path, allow_pickle=False) as scan:
                points = scan['points_sensor_m'].reshape(-1, 3)
                valid = scan['valid'].reshape(-1).astype(bool)
                ranges = scan['ranges_m'].reshape(-1)
                origin = scan['origin_world_m'].copy()
                rotation = scan['rotation_world_from_sensor'].copy()
                valid &= np.isfinite(points).all(axis=1) & np.isfinite(ranges)
                if max_range:
                    valid &= ranges <= max_range
                world = points[valid] @ rotation.T + origin
            source = str(path)
        else:
            module = scanner_module()
            if demo:
                module.demo_world()
            config = module.profile()
            ranges, origin, rotation = module.scan_frame(config)
            from sensor_rays import ray_directions
            valid = np.isfinite(ranges) & (ranges >= config['min_range_m'])
            if max_range:
                valid &= ranges <= max_range
            world = (ray_directions(config)[valid] * ranges[valid, None]) @ rotation.T + origin
            source = "Current scene, frame " + str(bpy.context.scene.frame_current)
    finally:
        for o, hidden in visibility:
            o.hide_render = hidden
    points = np.asarray(world[::stride], dtype=np.float32)
    if not len(points):
        raise RuntimeError("No visible returns. Add ground/obstacles, enable ADD_DEMO_WORLD, or increase MAX_DISPLAY_RANGE.")

    # Replace only objects owned by this script; keep arbitrary user objects.
    for o in previous:
        mesh = o.data
        groups = [m.node_group for m in o.modifiers if m.type == 'NODES']
        bpy.data.objects.remove(o, do_unlink=True)
        if mesh and mesh.users == 0:
            bpy.data.meshes.remove(mesh)
        for group in groups:
            if group and group.users == 0:
                bpy.data.node_groups.remove(group)
    mesh = bpy.data.meshes.new("OS1_Point_Positions")
    mesh.from_pydata(points.tolist(), [], [])
    mesh.update()
    obj = bpy.data.objects.new("OS1_Points", mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj[TAG] = True
    obj['scan_source'] = source
    obj['displayed_points'] = len(points)
    obj['display_radius_m'] = radius
    obj.color = (0.02, 0.85, 1.0, 1.0)

    material = bpy.data.materials.get("OS1_Point_Display_Material")
    if material is None:
        material = bpy.data.materials.new("OS1_Point_Display_Material")
    material.diffuse_color = obj.color
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    emission = nodes.new('ShaderNodeEmission')
    emission.inputs['Color'].default_value = obj.color
    emission.inputs['Strength'].default_value = 1.5
    output = nodes.new('ShaderNodeOutputMaterial')
    material.node_tree.links.new(emission.outputs['Emission'], output.inputs['Surface'])
    mesh.materials.append(material)

    group = bpy.data.node_groups.new("OS1_Point_Display_Nodes", 'GeometryNodeTree')
    group.interface.new_socket(name='Geometry', in_out='INPUT', socket_type='NodeSocketGeometry')
    group.interface.new_socket(name='Geometry', in_out='OUTPUT', socket_type='NodeSocketGeometry')
    nodes = group.nodes
    source_node = nodes.new('NodeGroupInput'); source_node.location = (-450, 150)
    sphere = nodes.new('GeometryNodeMeshIcoSphere'); sphere.location = (-450, -100)
    sphere.inputs['Radius'].default_value = radius
    sphere.inputs['Subdivisions'].default_value = 1
    set_material = nodes.new('GeometryNodeSetMaterial'); set_material.location = (-200, -100)
    set_material.inputs['Material'].default_value = material
    instances = nodes.new('GeometryNodeInstanceOnPoints'); instances.location = (50, 150)
    output_node = nodes.new('NodeGroupOutput'); output_node.location = (300, 150)
    group.links.new(source_node.outputs['Geometry'], instances.inputs['Points'])
    group.links.new(sphere.outputs['Mesh'], set_material.inputs['Geometry'])
    group.links.new(set_material.outputs['Geometry'], instances.inputs['Instance'])
    group.links.new(instances.outputs['Instances'], output_node.inputs['Geometry'])
    modifier = obj.modifiers.new("Visible lidar returns", 'NODES')
    modifier.node_group = group
    for selected in bpy.context.selected_objects:
        selected.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.context.view_layer.update()
    print(f"OS1: displaying {len(points):,} points from {source}", flush=True)
    return obj


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--npz', default=NPZ_PATH)
    parser.add_argument('--demo-world', action='store_true', default=ADD_DEMO_WORLD)
    parser.add_argument('--radius', type=float, default=POINT_RADIUS)
    parser.add_argument('--stride', type=int, default=DISPLAY_STRIDE)
    parser.add_argument('--max-range', type=float, default=MAX_DISPLAY_RANGE)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])
    show_lidar_points(args.npz, args.demo_world, args.radius, args.stride, args.max_range)
