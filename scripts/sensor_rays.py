"""Shared OS1 ray grid: elevation rows, azimuth columns in [0,2π)."""
import numpy as np
import base64,zlib
def head_visibility(c):
    count=c['channels']*c['columns']
    if 'head_visible_mask_b64' not in c:return np.ones(count,dtype=bool)
    raw=zlib.decompress(base64.b64decode(c['head_visible_mask_b64']))
    bits=np.unpackbits(np.frombuffer(raw,dtype=np.uint8),bitorder='little')
    if len(bits)!=count:raise ValueError('Invalid head visibility mask size')
    return bits.astype(bool)
def ray_directions(c):
    az=2*np.pi*np.arange(c['columns'])/c['columns'];el=np.deg2rad(np.linspace(c['vertical_min_deg'],c['vertical_max_deg'],c['channels']))
    return np.stack(np.broadcast_arrays(np.cos(el[:,None])*np.cos(az),np.cos(el[:,None])*np.sin(az),np.sin(el[:,None])),axis=-1).reshape((-1,3))
def valid_sensor_points(c,points):
    """Range and head-aperture validity for XYZ expressed in the lidar frame."""
    points=np.asarray(points);shape=points.shape[:-1];p=points.reshape(-1,3)
    r=np.linalg.norm(p,axis=1);valid=np.isfinite(p).all(1)&(r>=c['min_range_m'])&(r<=c['max_range_m'])
    safe=np.where(np.isfinite(p),p,0.)
    az=np.mod(np.arctan2(safe[:,1],safe[:,0]),2*np.pi)
    el=np.degrees(np.arctan2(safe[:,2],np.hypot(safe[:,0],safe[:,1])))
    row=(el-c['vertical_min_deg'])/(c['vertical_max_deg']-c['vertical_min_deg'])*(c['channels']-1)
    valid&=(row>=-.5)&(row<=c['channels']-.5)
    row=np.clip(np.rint(row).astype(int),0,c['channels']-1)
    column=np.rint(az*c['columns']/(2*np.pi)).astype(int)%c['columns']
    valid&=head_visibility(c)[row*c['columns']+column]
    return valid.reshape(shape)
def save_scan(path,c,ranges,origin,rotation,time_s):
    rays=ray_directions(c);valid=np.isfinite(ranges)&(ranges>=c['min_range_m'])&(ranges<=c['max_range_m']);local=np.full_like(rays,np.nan);local[valid]=rays[valid]*ranges[valid,None]
    np.savez_compressed(path,ranges_m=ranges.reshape(c['channels'],c['columns']),points_sensor_m=local.reshape(c['channels'],c['columns'],3),valid=valid.reshape(c['channels'],c['columns']),origin_world_m=origin,rotation_world_from_sensor=rotation,time_s=float(time_s),frame=c['frame'])
