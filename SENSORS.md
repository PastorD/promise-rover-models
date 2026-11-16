# PROMISE turret sensors — revision 0.3

All five formats now include **four turret cameras (two stereo pairs)** and a physically modeled **Ouster OS1 Rev 8 lidar**. The OS1 is now fully recessed inside the white mast-head box, near its front face, with a fixed **22.5° downward pitch** relative to the head. A real **150 × 92 mm front opening** exposes the optical window. The camera frames and lidar rotate with both mast yaw and head pitch. The OS1 is the user-requested substitution; its inclusion is not a claim that NASA selected this sensor for flight.

The Curiosity body and suspension, six Perseverance wheels, compact PROMISE head and removed arm are retained. The stowed TRIDENT drill is unchanged. NASA's November 7, 2026 release and public PROMISE model are the design references; see README.md for donor provenance.

## What each format provides

| Format | Camera implementation | Lidar implementation |
|---|---|---|
| URDF / Gazebo | Four camera housings, camera frames and ROS optical frames; xacro adds four image sensors and camera-info configuration | Physical CAD housing, mass, collision and fixed mount; xacro adds `gpu_lidar`; bridge supplies `/promise/os1/points_raw`; the launched aperture filter publishes `/promise/os1/points` as `sensor_msgs/msg/PointCloud2` |
| MuJoCo 4.0 | Four native MJCF cameras; included capture runner saves RGB | Housing, optical site and `os1_ranges` user sensor; included Python scanner fills it using native `mj_multiRay` |
| Blender 4.5 | Four native camera objects; VisionSIM 0.2.0 captures RGB and depth | Physical CAD housing and sensor frame; companion Blender BVH ray caster exports range and point arrays |
| GLB | Four native glTF perspective cameras, named hierarchy and camera calibration in extras | Physical housing, named sensor frame and profile JSON in extras; GLB does not execute a lidar simulator |
| Isaac Sim USD | Four camera prims; launcher creates RTX camera sensors | Physical rigid body and fixed joint, `OmniLidar` prim and an emitter profile; launcher creates an RTX lidar sensor |

The bare URDF contains geometry and frames. Use the Gazebo xacro for sensor execution. Opening the MJCF XML alone does **not** populate the user lidar sensor: run `mujoco_demo.py` or call `OS1Scanner.update(data)` from your own simulation loop. GLB is a portable scene description, not a running sensor engine.

## Geometry and scan configuration

The lidar housing is tessellated from Ouster's Rev 8 **OS1 XX** STEP model (without the optional halo). Its illustrative field-of-view solid is excluded. The CAD is converted from millimeters/Y-up to meters/ROS FLU. The former rooftop bracket is removed. A tilted internal cradle supports the lidar inside a 3 mm wall shell with the donor enclosure's original six-sided footprint. The front face has a true opening and the inside is hollow; the camera meshes and camera extrinsics are retained. The simplified head collisions are separate wall pieces, leaving the opening and cavity clear.

| Setting | Value |
|---|---|
| Housing diameter / height | 87 mm / 58.35 mm |
| Housing mass | 0.50 kg; mounting bracket adds an estimated 0.05 kg |
| Mount pitch | +22.5° around ROS +Y; the forward +X ray points down 22.5° |
| Mount origin in nominal rover coordinates | (0.692, −0.164, 1.9522) m |
| Optical center in nominal rover coordinates | Approximately (0.707384, −0.164, 1.989340) m |
| Forward aperture | 150 mm wide × 92 mm high; no glass cover |
| Front edge of housing | Approximately 0.55 mm behind the outer front face, inside the opening |
| Lidar frame | `os1_lidar_frame`, fixed to `os1_link` |
| Parent chain | `mast_yaw_link → head_pitch_link → os1_link → os1_lidar_frame` |
| Selected mode | 128 channels × 1024 azimuth samples at 10 Hz |
| Horizontal / vertical field of view | 360° / 44° |
| Elevation angles | Uniform from −22° to +22°, inclusive |
| Azimuth angles | 0 to 360° with endpoint excluded; +X forward, +Y left, +Z up |
| Simulated valid range | 0.5–170 m |
| Optical center | 40.2 mm along lidar-local +Z from its mounting origin; estimated |
| Sampling | Instantaneous full scan, one geometric return per ray |

The native 360° ray grid is retained, but the enclosing box blocks most side and rear directions. Mesh ray tests found 38,314 directions clear of the rigid head and 92,758 blocked. The complete 44° vertical fan is clear for sensor-local azimuths approximately −44.65° to +44.65°. At azimuth zero, the nominal head-level fan spans **44.5° down to 0.5° down**. Additional obstruction by terrain or other rover components is evaluated at runtime.

A compressed visibility mask, derived from the actual head mesh and including the four camera housings, is embedded in the profile. It excludes the OS1 case itself. Blender and MuJoCo apply it in addition to geometric ray casting. The ROS launch applies it to the raw Gazebo point cloud because a renderer's near clipping can otherwise hide the close enclosure. GLB and USD carry the same mask as metadata. The Isaac sample saves raw GenericModelOutput and the profile: after decoding to Cartesian SENSOR-frame XYZ, use `sensor_rays.valid_sensor_points(profile, xyz)` to enforce the mask. The raw RTX buffer is not post-filtered automatically.

The manufacturer datasheet supports the selected channel count, mode and FOV. **170 m is the chosen simulation cutoff**, not a guarantee at every reflectivity. The documented range depends on target reflectivity and operating conditions. Beam elevations, azimuth offsets, optical origin and mounting extrinsics are illustrative, not serial-number calibration. No motion distortion, packet timing, dual returns, reflectivity response, native RGB, calibrated noise or dropout is implemented. Isaac's ideal-ray profile should be reviewed in the installed RTX runtime before quantitative use.

## Cameras

| Pair | Names | Baseline | Estimated nominal optical centers in common model coordinates |
|---|---|---|---|
| Stereo | `stereo_left`, `stereo_right` | 0.40 m | X = 0.835 m, Y = −0.164 ± 0.200 m, Z = 1.951 m |
| Dashcam | `dashcam_left`, `dashcam_right` | 0.31 m | X = 0.835 m, Y = −0.164 ± 0.155 m, Z = 2.003 m |

All cameras use a pinhole projection, 1280 × 720, 70° horizontal FOV and a requested 10 Hz capture rate. Sensor parameters and baselines are estimates, not NASA calibration. The optical centers are slightly ahead of the modeled front housing to avoid self-occlusion. ROS optical frames use +Z forward, +X right, +Y down; Blender/MuJoCo/USD native camera axes use −Z forward, +Y up. Exports perform the conversion.

## MuJoCo sensor capture

From the `promise_rover` directory:

```bash
MUJOCO_GL=egl uv run --python 3.12 --with mujoco==4.0.0 --with pillow \
  scripts/mujoco_demo.py --headless --seconds 1 --capture captures/mujoco
```

The normal runner activates the lidar by default. `--no-lidar` disables ray casting when testing dynamics alone. Captures contain four RGB PNGs, camera transforms/calibration and `os1.npz`. For custom loops:

```python
from mujoco_sensors import OS1Scanner
scanner = OS1Scanner(model)
# After each physics step:
scanner.update(data)  # runs at 10 Hz simulation time, holds the last scan between ticks
```

`scripts` must be on the Python import path. The scanner uses detailed visual meshes and excludes simplified collision proxies and its own lidar body. It still accounts for the enclosing mast box, front aperture, cameras and cradle. Each full scan contains 131,072 rays; execution is not guaranteed to keep up with real time on the host CPU.

## Blender with VisionSIM

The `.blend` includes packed textures, the four sensor cameras, lidar frame and embedded profile, plus the original six-wheel animation. Use the **Blender sensor ZIP** or complete ZIP for the companion scripts. VisionSIM is an external dependency, not bundled into the `.blend`.

The tested integration runs `visionsim.simulate.blender.BlenderService` inside a local `bpy` Python interpreter. It reuses the model's camera positions and intrinsics. It does not start an RPC server. VisionSIM generates RGB, depth EXR and metadata; the OS1 geometric scanner is a separate component because this VisionSIM workflow does not implement spinning-lidar emulation.

```bash
uv venv .vision --python 3.11
uv pip install --python .vision/bin/python torch==2.15.0+cpu \
  --index-url https://download.pytorch.org/whl/cpu
uv pip install --python .vision/bin/python bpy==4.5.3 visionsim==0.2.0 \
  numpy==1.26.4 pillow OpenEXR
.vision/bin/python scripts/visionsim_capture.py \
  --output captures/visionsim --start 1 --end 1 --lidar --demo-world
```

The CPU Torch wheel avoids installing CUDA solely for this capture helper. Rendering can be configured separately for your Blender GPU. For an animation sequence use `--start 1 --end 240 --step 3`; at 30 fps, step 3 samples at 10 Hz. The existing wheel animation spins the wheels with the rover stationary. Add a vehicle trajectory and environment for navigation datasets.

`--scale 0.25 --samples 4` is a quick 320 × 180 smoke test. Remove `--demo-world` when using your own scene. The flag adds a flat ground plane, a blue target box at X=5 m and a farther column. VisionSIM files are under each camera's `frames`, `depths` and metadata directories. Lidar captures are under `os1`.

To run only lidar inside a normal Blender process:

```bash
blender -b blender/promise_wheel_animation.blend \
  --python scripts/blender_sensors.py -- --output captures/os1 --demo-world
```

The NPZ files use `ranges_m[128,1024]`, `points_sensor_m[128,1024,3]`, `valid`, `origin_world_m`, `rotation_world_from_sensor`, `time_s` and `frame`. Invalid returns are NaN. World points are `points_sensor_m @ rotation_world_from_sensor.T + origin_world_m`.

## Display scan points in Blender

The Blender and complete bundles include `scripts/show_lidar_points.py`. Open the rover .blend, open that script in Blender's Scripting workspace and run it. Set `ADD_DEMO_WORLD = True` for a ground/obstacle demonstration, or use your scene. It reads the embedded tilted sensor frame, so points follow the new mount automatically. Rerun after changing a pose; the display is a static snapshot. For a saved capture, set `NPZ_PATH` to its path.

## Isaac Sim

Run the included launcher in the Isaac Sim Python environment with an RTX-capable renderer:

```bash
./python.sh /absolute/path/promise_rover/scripts/isaac_demo.py \
  --headless --seconds 2 --capture /absolute/path/captures/isaac
```

It targets the Isaac Sim 6.x `isaacsim.sensors.experimental.rtx` API family. The lidar prim is `/World/PROMISE/os1_link/os1_lidar_frame/sensor`. Keep `os1_rev8_lidar.usda`, `promise.usdc` and textures alongside the scene. The helper renders frames even in headless mode, which the sensors require. It saves camera RGB/range NPZs, the raw lidar GenericModelOutput buffer and the OS1 profile. Native RTX near-field self-occlusion remains unverified; apply the included profile mask to decoded sensor-frame points as described above. This is a snapshot example, not a lossless stream recorder or ROS bridge.

## Verification and limits

`validation.json`, `references/sensor_validation.json` and `references/blender_validation.json` and `references/recessed_mount_validation.json` record the actual checks. The recessed-mount check also verifies physical fit, downward pitch, all 128 forward-center rays and rear-wall blockage. The sensor checks render all four MuJoCo and VisionSIM cameras, ray-cast a target with a known position, compare URDF/MuJoCo sensor transforms at three turret poses, check Blender turret parenting and six-wheel animation, inspect GLB camera/profile metadata and compose USD references.

Blender 4.5.3, VisionSIM 0.2.0 and MuJoCo 4.0.0 were exercised. ROS/Gazebo and Isaac Sim were **not runtime-tested** in this environment; their exports received structural checks. Mounting, calibration, mass distribution and contact dynamics remain engineering estimates. These are simulation assets, not flight-qualified hardware models.

## Sources

- [NASA PROMISE model](https://www.nasa.gov/moonbase/promise-model/)
- [Ouster downloads and sensor CAD](https://ouster.com/downloads)
- [Ouster OS1 Rev 8 datasheet](https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os1.pdf)
- [VisionSIM documentation](https://visionsim.readthedocs.io/en/latest/)
- [MuJoCo documentation](https://mujoco.readthedocs.io/en/stable/)
- [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/latest/)

The complete package includes the source STEP archive and datasheet with SHA-256 hashes. NASA and Ouster source assets retain their respective attribution and terms; see NOTICE.md.
