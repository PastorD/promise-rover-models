"""Call update() after stepping to populate the MJCF os1_ranges user sensor.
The XML alone does not run Python. This helper uses native MuJoCo 4 mj_multiRay
at the configured sensor rate, holding the last complete scan between ticks.
"""
import json
from pathlib import Path
import mujoco,numpy as np
from sensor_rays import ray_directions,save_scan,head_visibility
class OS1Scanner:
    def __init__(self,model,config=None):
        self.model=model;self.config=config or json.loads((Path(__file__).resolve().parents[1]/'data/os1_rev8_profile.json').read_text());self.site=model.site(self.config['frame']).id;self.sensor=model.sensor('os1_ranges').id;self.address=int(model.sensor_adr[self.sensor]);self.count=int(model.sensor_dim[self.sensor]);self.rays=ray_directions(self.config);assert len(self.rays)==self.count
        self.visible=head_visibility(self.config);self.ranges=np.full(self.count,np.nan);self.ids=np.full(self.count,-1,dtype=np.int32);self.next_time=0.;self.last_time=None;self.groups=np.array([1,1,1,0,0,0],dtype=np.uint8)
    def update(self,data,force=False):
        due=force or data.time+1e-9>=self.next_time
        if due:
            mujoco.mj_fwdPosition(self.model,data);self.origin=data.site_xpos[self.site].copy();self.rotation=data.site_xmat[self.site].reshape(3,3).copy();directions=np.ascontiguousarray(self.rays@self.rotation.T).ravel();dist=np.empty(self.count)
            mujoco.mj_multiRay(self.model,data,self.origin,directions,self.groups,True,int(self.model.site_bodyid[self.site]),self.ids,dist,None,self.count,self.config['max_range_m'])
            dist[(dist<self.config['min_range_m'])|(dist>self.config['max_range_m'])|(~self.visible)]=np.nan;self.ranges=dist;self.last_time=float(data.time);self.next_time=float(data.time)+1/self.config['rate_hz']
        data.sensordata[self.address:self.address+self.count]=self.ranges
        return due
    def save(self,path):
        if self.last_time is None:raise RuntimeError('Call update() before save().')
        save_scan(path,self.config,self.ranges,self.origin,self.rotation,self.last_time)
