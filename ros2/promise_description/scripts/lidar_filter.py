#!/usr/bin/env python3
"""Apply the physical head aperture to the raw Gazebo OS1 point cloud.
Some renderers clip nearby housing geometry at the lidar minimum range. The
precomputed mask retains that housing's shadow without rendering the OS1 case.
"""
import copy,json
from pathlib import Path
import numpy as np
from sensor_rays import valid_sensor_points


def filter_cloud(message,profile):
    fields={f.name:f for f in message.fields};formats=[];offsets=[]
    for name in ('x','y','z'):
        field=fields[name]
        if field.datatype not in (7,8) or field.count!=1:raise ValueError('Expected scalar float XYZ fields')
        formats.append(('>' if message.is_bigendian else '<')+('f4' if field.datatype==7 else 'f8'));offsets.append(field.offset)
    dtype=np.dtype({'names':['x','y','z'],'formats':formats,'offsets':offsets,'itemsize':message.point_step})
    data=bytearray(message.data)
    values=np.ndarray((message.height,message.width),dtype=dtype,buffer=data,strides=(message.row_step,message.point_step))
    valid=valid_sensor_points(profile,np.stack([values[n] for n in ('x','y','z')],axis=-1))
    for name in ('x','y','z'):values[name][~valid]=np.nan
    output=copy.copy(message);output.data=bytes(data);output.is_dense=False
    return output


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    class Filter(Node):
        def __init__(self):
            super().__init__('os1_aperture_filter')
            self.declare_parameter('profile','')
            self.profile=json.loads(Path(self.get_parameter('profile').value).read_text())
            self.publisher=self.create_publisher(PointCloud2,'/promise/os1/points',qos_profile_sensor_data)
            self.subscription=self.create_subscription(PointCloud2,'/promise/os1/points_raw',self.receive,qos_profile_sensor_data)
        def receive(self,message):
            if message.header.frame_id.lstrip('/')!=self.profile['frame']:
                self.get_logger().error('OS1 aperture filter requires XYZ in '+self.profile['frame']);return
            self.publisher.publish(filter_cloud(message,self.profile))
    from rclpy.executors import ExternalShutdownException
    rclpy.init();node=Filter()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except Exception:
        if rclpy.ok():raise # otherwise Ctrl-C raced spin(); exit cleanly
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
