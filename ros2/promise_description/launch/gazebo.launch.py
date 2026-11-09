from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    share=Path(get_package_share_directory('promise_description'))
    gz_share=Path(get_package_share_directory('ros_gz_sim'))
    description=xacro.process_file(str(share/'urdf/promise.gazebo.urdf.xacro')).toxml()
    gazebo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':'-r '+str(share/'worlds/moon.sdf')}.items())
    rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':description,'use_sim_time':True}])
    spawn=Node(package='ros_gz_sim',executable='create',arguments=['-topic','robot_description','-name','promise','-z','0.92'],output='screen')
    controllers=[Node(package='controller_manager',executable='spawner',arguments=[n,'--controller-manager-timeout','120']) for n in ['joint_state_broadcaster','wheel_controller','steering_controller','mast_controller']]
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':str(share/'config/bridge.yaml'),'use_sim_time':True}])
    drive=Node(package='promise_description',executable='drive.py',parameters=[{'use_sim_time':True}])
    info=Node(package='promise_description',executable='stereo_info.py',parameters=[{'use_sim_time':True}])
    return LaunchDescription([gazebo,rsp,bridge,spawn,RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=controllers)),drive,info])
