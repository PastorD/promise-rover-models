"""Four-corner steering and six-wheel speeds for a planar body twist (m/s, rad/s)."""
import math

WHEEL_ORDER=['front_left','middle_left','rear_left','front_right','middle_right','rear_right']
STEER_ORDER=['front_left','rear_left','front_right','rear_right']
CENTERS={name:(x,y) for side,sign in [('left',1),('right',-1)] for name,x,y in [(f'front_{side}',1.094,sign*1.061),(f'middle_{side}',-.089,sign*1.192),(f'rear_{side}',-1.164,sign*1.061)]}

def wheel_commands(v,w,radius=.2625):
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
