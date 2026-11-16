"""Four-corner steering and six-wheel speeds for a planar body twist (m/s, rad/s)."""
import math

WHEEL_ORDER=['front_left','middle_left','rear_left','front_right','middle_right','rear_right']
STEER_ORDER=['front_left','rear_left','front_right','rear_right']
# Steering axes and middle-wheel centres (m, base_link), from data/robot_spec.json.
CENTERS={'front_left':(1.097,1.065),'middle_left':(-.090,1.185),'rear_left':(-1.165,1.063),'front_right':(1.095,-1.062),'middle_right':(-.090,-1.202),'rear_right':(-1.167,-1.062)}

def wheel_commands(v,w,radius=.2634):
    """Return wheel rad/s and corner radians in the declared orders.

    Rear wheels counter-steer. Middle wheels are fixed and use the longitudinal
    component; their slight fore-aft offset causes small unavoidable scrub.
    Commands whose corner angle exceeds 1.4 rad are rejected rather than silently
    clamping steering into inconsistent wheel velocities.
    """
    speeds={};angles={}
    for name,(x,y) in CENTERS.items():
        vx=v-w*y;vy=w*x
        if name.startswith('middle'):
            speeds[name]=vx/radius;continue
        angle=math.atan2(vy,vx) if abs(vx)+abs(vy)>1e-10 else 0.
        speed=math.hypot(vx,vy)
        if angle>math.pi/2:angle-=math.pi;speed=-speed
        elif angle<-math.pi/2:angle+=math.pi;speed=-speed
        if abs(angle)>1.4:raise ValueError('Requested curvature exceeds corner steering limits; reduce yaw rate or change speed.')
        angles[name]=angle;speeds[name]=speed/radius
    if max(abs(x) for x in speeds.values())>4:raise ValueError('Wheel speed exceeds 4 rad/s model limit.')
    return [speeds[n] for n in WHEEL_ORDER],[angles[n] for n in STEER_ORDER]
