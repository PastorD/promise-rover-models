#!/usr/bin/env python3
"""Publish estimated rectified calibration with the right camera's -fx*baseline.
Separate Gazebo camera sensors otherwise publish a monocular projection matrix.
"""
import json,pathlib,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo,Image
from ament_index_python.packages import get_package_share_directory

class StereoInfo(Node):
    def __init__(self):
        super().__init__('promise_stereo_info')
        root=pathlib.Path(get_package_share_directory('promise_description'))/'config'
        self.pubs={};self.cal={};self.subs=[]
        for name in ['stereo_left','stereo_right','dashcam_left','dashcam_right']:
            self.cal[name]=json.loads((root/(name+'_calibration.yaml')).read_text())
            self.pubs[name]=self.create_publisher(CameraInfo,'/promise/'+name+'/camera_info',qos_profile_sensor_data)
            self.subs.append(self.create_subscription(Image,'/promise/'+name+'/image',lambda image,name=name:self.publish(name,image),qos_profile_sensor_data))
    def publish(self,name,image):
        c=self.cal[name];m=CameraInfo();m.header=image.header;m.header.frame_id=name+'_optical_frame'
        m.width=c['image_width'];m.height=c['image_height'];m.distortion_model=c['distortion_model']
        m.k=[float(x) for x in c['camera_matrix']['data']];m.d=[float(x) for x in c['distortion_coefficients']['data']];m.r=[float(x) for x in c['rectification_matrix']['data']];m.p=[float(x) for x in c['projection_matrix']['data']]
        self.pubs[name].publish(m)

def main():
    rclpy.init();n=StereoInfo()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
