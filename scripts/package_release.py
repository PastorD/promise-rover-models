"""Build portable per-platform downloads and the complete reproducible package."""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / "deliverables"
OUT.mkdir(exist_ok=True)

common = [ROOT / p for p in (
    "README.md", "NOTICE.md", "LICENSE", "validation.json",
    "data/robot_spec.json", "references/source_checksums.json",
    "references/blender_validation.json", "previews/promise_rover.png",
)]

def files_in(directory):
    return [p for p in (ROOT / directory).rglob("*")
            if p.is_file() and "__pycache__" not in p.parts
            and not p.name.endswith(".blend1")]

ros = files_in("ros2")
usd = files_in("usd")
mesh_prefix = ROOT / "ros2/promise_description/meshes"
texture_files = files_in("ros2/promise_description/textures")

def archive(name, files):
    path = OUT / name
    temporary = OUT / (name + ".partial")
    selected = sorted(set(common + files))
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in selected:
            z.write(p, Path("promise_rover") / p.relative_to(ROOT))
    with zipfile.ZipFile(temporary) as z:
        assert z.testzip() is None
    temporary.replace(path)
    print(f"{name}: {len(selected)} files, {path.stat().st_size:,} bytes", flush=True)
    return path

archive("promise_ros2_gazebo.zip", [p for p in ros if p.suffix not in {".obj", ".npz"}])
archive("promise_mujoco_4.zip", files_in("mujoco") + texture_files
        + list(mesh_prefix.glob("*.obj"))
        + [ROOT / "scripts/mujoco_demo.py", ROOT / "scripts/kinematics.py"])
archive("promise_isaac_usd.zip", usd
        + [ROOT / "scripts/isaac_demo.py", ROOT / "scripts/kinematics.py"])

source_files = [ROOT / "source_assets" / name for name in
                ("curiosity_clean.zip", "perseverance_blend.zip", "perseverance.glb", "promise_nasa.glb")]
archive("promise_rover_complete.zip", ros + usd + files_in("mujoco")
        + files_in("blender") + files_in("glb") + files_in("scripts")
        + files_in("previews") + source_files
        + [ROOT / "references/promise-7.jpg", ROOT / "references/promise_reference_render.png"])

for source, name in (
    (ROOT / "glb/promise.glb", "promise.glb"),
    (ROOT / "blender/promise_wheel_animation.blend", "promise_wheel_animation.blend"),
    (ROOT / "previews/promise_rover.png", "promise_rover.png"),
    (ROOT / "README.md", "PROMISE_README.md"),
):
    shutil.copy2(source, OUT / name)

manifest = [{"file": p.name, "bytes": p.stat().st_size,
             "sha256": hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in sorted(OUT.iterdir()) if p.is_file() and p.name != "release_manifest.json"]
(OUT / "release_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
