"""Isaac Sim 6.x OmniLidar profile; ideal rays, estimated beam calibration."""
import numpy as np
from pxr import Usd,UsdGeom,Sdf,Vt
def write_os1_profile(path,c):
    stage=Usd.Stage.CreateNew(str(path));UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,'Z')
    p=stage.DefinePrim('/OS1_Rev8','OmniLidar');stage.SetDefaultPrim(p)
    for api in ['OmniSensorAPI','OmniSensorGenericLidarCoreAPI','OmniSensorGenericLidarCoreEmitterStateAPI:s001']:p.AddAppliedSchema(api)
    def a(name,kind,value):p.CreateAttribute(name,kind,custom=False).Set(value)
    for n,v in [('modelName','OS1_Rev8_128_1024_10_estimated'),('modelVersion','Rev8'),('modelVendor','Ouster'),('marketName','OS1 Rev8')]:a('omni:sensor:'+n,Sdf.ValueTypeNames.String,v)
    a('omni:sensor:tickRate',Sdf.ValueTypeNames.Float,float(c['rate_hz']));prefix='omni:sensor:Core:'
    for n,v in [('scanType','ROTARY'),('rotationDirection','CCW'),('rayType','IDEALIZED'),('elementsCoordsType','CARTESIAN'),('outputFrameOfReference','SENSOR'),('outputMotionCompensationState','NONCOMPENSATED')]:a(prefix+n,Sdf.ValueTypeNames.Token,v)
    for n,v in [('scanRateBaseHz',c['rate_hz']),('patternFiringRateHz',c['rate_hz']*c['columns']),('numberOfEmitters',c['channels']),('numberOfChannels',c['channels']),('maxReturns',1),('emitterStateCount',1),('stateResolutionStep',1)]:a(prefix+n,Sdf.ValueTypeNames.UInt,v)
    for n,v in [('nearRangeM',c['min_range_m']),('farRangeM',c['max_range_m']),('validStartAzimuthDeg',0),('validEndAzimuthDeg',360),('startAzimuthOffsetDeg',0),('rangeResolutionM',.001),('rangeAccuracyM',0),('azimuthErrorStd',0),('elevationErrorStd',0),('waveLengthNm',865),('minReflectance',.1),('minReflectionRangeM',90)]:a(prefix+n,Sdf.ValueTypeNames.Float,float(v))
    for n,v in [('instantLidar',True),('accumulateOutputs',True),('skipDroppingInvalidPoints',True)]:a(prefix+n,Sdf.ValueTypeNames.Bool,v)
    n=c['channels'];state=prefix+'emitterState:s001:'
    for k,v in [('azimuthDeg',[0.]*n),('elevationDeg',np.linspace(c['vertical_min_deg'],c['vertical_max_deg'],n).tolist())]:a(state+k,Sdf.ValueTypeNames.FloatArray,Vt.FloatArray(v))
    for k,v in [('fireTimeNs',[0]*n),('channelId',list(range(n)))]:a(state+k,Sdf.ValueTypeNames.UIntArray,Vt.UIntArray(v))
    p.SetCustomData({'calibration':c['calibration'],'sampling':c['sampling'],'axes':'X forward, Y left, Z up','mountPitchDownDeg':float(c.get('mount_pitch_down_deg',0)),'headVisibleMaskEncoding':c.get('head_visible_mask_encoding',''),'headVisibleMaskBase64':c.get('head_visible_mask_b64',''),'installation':c.get('installation','')});stage.GetRootLayer().Save()
