#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from kinematics import wheel_commands

class Drive(Node):
    def __init__(self):
        super().__init__('promise_drive');self.v=0.;self.w=0.;self.last=self.get_clock().now()
        self.wh=self.create_publisher(Float64MultiArray,'/wheel_controller/commands',10)
        self.st=self.create_publisher(Float64MultiArray,'/steering_controller/commands',10)
        self.create_subscription(Twist,'/cmd_vel',self.receive,10);self.create_timer(.02,self.update)
    def receive(self,msg):
        self.v=msg.linear.x;self.w=msg.angular.z;self.last=self.get_clock().now()
    def update(self):
        v,w=(self.v,self.w) if (self.get_clock().now()-self.last).nanoseconds<500_000_000 else (0.,0.)
        try:s,a=wheel_commands(v,w)
        except ValueError as error:
            self.get_logger().warning(str(error),throttle_duration_sec=2);s,a=wheel_commands(0,0)
        self.wh.publish(Float64MultiArray(data=s));self.st.publish(Float64MultiArray(data=a))

def main():
    rclpy.init();n=Drive()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except Exception:
        if rclpy.ok():raise # otherwise Ctrl-C raced spin(); exit cleanly
    finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
