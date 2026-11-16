#!/usr/bin/env bash
# Headless Gazebo Harmonic + ROS 2 Jazzy runtime test of promise_description.
# Build the package in a colcon workspace first (see README), then:
#   bash scripts/gazebo_runtime_test.sh <workspace> <out_dir>
# Drives 0.10 m/s for 15 s, commands the mast, and checks the OS1 cloud and cameras.
# Results: <out_dir>/probe.json, controllers.txt, pose.txt, launch.log
set +u
WS=$1; OUT=$2; rm -rf "$OUT"; mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
if pgrep -x ruby -a 2>/dev/null | grep -q "moon.sdf" || pgrep -f "^gz sim .*moon.sdf" >/dev/null; then echo "stale gz sim running; aborting"; exit 2; fi
export ROS_DOMAIN_ID=73 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=promise_test_$$ QT_QPA_PLATFORM=offscreen
SHARE=$(ros2 pkg prefix promise_description)/share/promise_description
cat > "$OUT/headless.launch.py" <<PY
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.actions, launch_ros.actions
src = open('$SHARE/launch/gazebo.launch.py').read().replace("'-r '+", "'-r -s --headless-rendering '+")
ns = {}; exec(compile(src, 'gazebo.launch.py', 'exec'), ns)
def generate_launch_description(): return ns['generate_launch_description']()
PY
setsid ros2 launch "$OUT/headless.launch.py" > "$OUT/launch.log" 2>&1 &
LAUNCH=$!
cleanup() { kill -INT -- -$LAUNCH 2>/dev/null; sleep 5; kill -9 -- -$LAUNCH 2>/dev/null; wait $LAUNCH 2>/dev/null; ros2 daemon stop >/dev/null 2>&1; }
trap cleanup EXIT
for i in $(seq 1 120); do
  ros2 control list_controllers 2>/dev/null | grep -c active > "$OUT/active_count" ; [ "$(cat "$OUT/active_count")" -ge 4 ] && break; sleep 1
done
echo "controllers after ${i}s:"; ros2 control list_controllers 2>&1 | tee "$OUT/controllers.txt"
timeout 25 ros2 topic echo --once /promise/os1/points --field header > "$OUT/os1_header.txt" 2>&1
timeout 25 ros2 topic echo --once /promise/os1/points --field width > "$OUT/os1_width.txt" 2>&1
timeout 20 ros2 topic echo --once /promise/stereo_left/image --field encoding > "$OUT/img.txt" 2>&1
timeout 20 ros2 topic echo --once /promise/stereo_left/camera_info --field p > "$OUT/caminfo.txt" 2>&1
python3 - "$OUT" <<'PY' &
import sys, json, time, math, rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState, PointCloud2
from tf2_msgs.msg import TFMessage
out = sys.argv[1]; rclpy.init(); n = Node('promise_runtime_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
state = {'js': None, 'cloud': None, 'pose': []}
n.create_subscription(JointState, '/joint_states', lambda m: state.__setitem__('js', m), 10)
n.create_subscription(PointCloud2, '/promise/os1/points', lambda m: state.__setitem__('cloud', m), qos_profile_sensor_data)
pub = n.create_publisher(Twist, '/cmd_vel', 10); mast = n.create_publisher(Float64MultiArray, '/mast_controller/commands', 10)
def spin(sec, cmd=None):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        if cmd: pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=.05)
spin(3); js0 = state['js']
t = Twist(); t.linear.x = .10; spin(15, t); js1 = state['js']
t = Twist(); spin(2, t)
end = time.monotonic() + 6
while time.monotonic() < end: mast.publish(Float64MultiArray(data=[.4, -.2])); rclpy.spin_once(n, timeout_sec=.05)
js2 = state['js']
res = {}
if js0 and js1:
    p0 = dict(zip(js0.name, js0.position)); p1 = dict(zip(js1.name, js1.position)); v1 = dict(zip(js1.name, js1.velocity))
    res['wheel_rotation_rad'] = {k: round(p1[k] - p0[k], 3) for k in p1 if k.endswith('wheel_joint')}
    res['wheel_velocity'] = {k: round(v1[k], 3) for k in v1 if k.endswith('wheel_joint')}
    res['rocker_sum_rad'] = round(p1.get('rocker_left_joint', 0) + p1.get('rocker_right_joint', 0), 4)
    res['all_finite'] = all(math.isfinite(x) for x in js1.position)
    p2 = dict(zip(js2.name, js2.position)); res['mast_after_command_[0.4,-0.2]'] = [round(p2['mast_yaw_joint'], 3), round(p2['head_pitch_joint'], 3)]
if state['cloud'] is not None:
    c = state['cloud']; dt = np.dtype({'names': ['x','y','z'], 'formats': ['<f4']*3, 'offsets': [f.offset for f in c.fields if f.name in 'xyz'][:3], 'itemsize': c.point_step})
    a = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(c.height, c.row_step)[:, :c.width*c.point_step].copy().view(dt).reshape(c.height, c.width)
    xyz = np.stack([a['x'], a['y'], a['z']], -1).reshape(-1, 3); fin = np.isfinite(xyz).all(1)
    res['cloud'] = dict(frame=c.header.frame_id, height=c.height, width=c.width, finite_points=int(fin.sum()), min_range=float(np.linalg.norm(xyz[fin], axis=1).min()) if fin.any() else None, median_range=float(np.median(np.linalg.norm(xyz[fin], axis=1))) if fin.any() else None)
json.dump(res, open(out + '/probe.json', 'w'), indent=1); n.destroy_node(); rclpy.shutdown()
PY
PROBE=$!; wait $PROBE
gz model --list > "$OUT/models.txt" 2>&1; gz model -m promise -p > "$OUT/pose.txt" 2>&1
echo "== probe:"; cat "$OUT/probe.json"; echo; echo "== os1 header:"; head -8 "$OUT/os1_header.txt"; echo "== image: $(head -1 "$OUT/img.txt")"; echo "== camera_info p: $(head -3 "$OUT/caminfo.txt" | tr '\n' ' ')"
echo "== pose:"; cat "$OUT/pose.txt" | head -12
echo "== log errors:"; grep -i -E 'error|exception|traceback|fatal|segmentation|failed' "$OUT/launch.log" | grep -v -i 'ruby' | sort | uniq -c | sort -rn | head -15
