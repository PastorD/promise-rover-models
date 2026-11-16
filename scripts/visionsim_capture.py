"""VisionSIM 0.2.0 turret RGB/depth with existing camera extrinsics.
Run inside Blender after visionsim post-install. Uses a local BlenderService,
with no network/RPC server. Lidar uses the companion Blender ray caster.
"""
import argparse,sys,json
from pathlib import Path
import bpy
sys.path.insert(0,str(Path(__file__).resolve().parent))
from blender_sensors import demo_world,scan_frame,profile
from sensor_rays import save_scan
def main():
    p=argparse.ArgumentParser();p.add_argument('--blend',type=Path,default=Path(__file__).resolve().parents[1]/'blender/promise_wheel_animation.blend');p.add_argument('--output',type=Path,default=Path('captures/visionsim'));p.add_argument('--start',type=int,default=1);p.add_argument('--end',type=int,default=1);p.add_argument('--step',type=int,default=3);p.add_argument('--scale',type=float,default=1);p.add_argument('--samples',type=int,default=16);p.add_argument('--demo-world',action='store_true');p.add_argument('--lidar',action='store_true')
    a=p.parse_args(sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else sys.argv[1:]);a.blend=a.blend.resolve();a.output=a.output.resolve()
    if a.step<1 or not 0<a.scale<=1:raise ValueError('Invalid step or scale.')
    from visionsim.simulate.blender import BlenderService
    frames=list(range(a.start,a.end+1,a.step));summary={}
    for name in ['stereo_left','stereo_right','dashcam_left','dashcam_right']:
        service=BlenderService();service.exposed_initialize(a.blend,a.output/name);camera=bpy.data.objects[name];bpy.context.scene.camera=camera;c=json.loads(camera['calibration_json']);scene=bpy.context.scene;scene.render.resolution_x=round(c['width']*a.scale);scene.render.resolution_y=round(c['height']*a.scale);scene.render.resolution_percentage=100;scene.cycles.samples=a.samples;scene.render.film_transparent=False
        if a.demo_world:demo_world()
        service.exposed_include_frames();service.exposed_include_depths(preview=False,exr_codec='ZIP');service.exposed_render_frames(frames,allow_skips=False)
        summary[name]={'frames':frames,'resolution':[scene.render.resolution_x,scene.render.resolution_y],'calibration':c,'outputs':['RGB PNG','depth EXR','VisionSIM metadata']}
    if a.lidar:
        c=profile();out=a.output/'os1';out.mkdir(parents=True,exist_ok=True)
        for frame in frames:
            bpy.context.scene.frame_set(frame);ranges,origin,rotation=scan_frame(c);save_scan(out/f'{frame:06}.npz',c,ranges,origin,rotation,(frame-1)/bpy.context.scene.render.fps)
        summary['os1']={'profile':c,'frames':frames,'implementation':'Blender geometric ray casting, separate from VisionSIM camera emulation'}
    a.output.mkdir(parents=True,exist_ok=True);(a.output/'capture.json').write_text(json.dumps(summary,indent=2));print('VISION_SIM_CAPTURE_COMPLETE',flush=True)
if __name__=='__main__':main()
