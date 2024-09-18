#!/usr/bin/python3

from turtle_bringup.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import String 

from geometry_msgs.msg import Twist,Point,TransformStamped,PoseStamped

from turtlesim.msg import Pose

import numpy as np

from turtlesim_plus_interfaces.srv import GivePosition

from std_srvs.srv import Empty 

# from nav_msgs.msg import Odometry

# from tf2_ros import  TransformBroadcaster

from tf_transformations import quaternion_from_euler

import math

from controller_interfaces.srv import SetTarget,SetParam

from sun_interfaces.srv import PizzaPose

from std_msgs.msg import Int32

class sum_controller(Node):
    
    def __init__(self):
        super().__init__('sum_controller')
        cl = "/cp"
        self.create_subscription(Int32,"Foxy/isfinish",self.Foxy_callback,10) 
        self.create_subscription(Int32,"Noetic/isfinish",self.Noetic_callback,10) 
        self.create_subscription(Int32,"Humble/isfinish",self.Humble_callback,10) 
        self.create_subscription(Int32,"Iron/isfinish",self.Iron_callback,10)

        self.create_timer(0.1,self.timmer_callback)
        self.f = 0
        self.n = 0
        self.h = 0
        self.i = 0
        self.pub_sum = self.create_publisher(Int32,'/sum_callback',10)

    def Foxy_callback(self,x):
        # self.get_logger().info(f"{x.data}")
        self.f = x.data
    def Noetic_callback(self,x):
        # self.get_logger().info(f"{x.data}")
        self.n = x.data
    def Humble_callback(self,x):
        # self.get_logger().info(f"{x.data}")
        self.h = x.data
    def Iron_callback(self,x):
        # self.get_logger().info(f"{x.data}")
        self.i = x.data


    def timmer_callback(self):
        msg = Int32()
        if self.f and self.n and self.i and self.h:
            msg.data = 1
            # self.get_logger().info(f"{msg.data}")
            self.pub_sum.publish(msg)
        else:
            msg.data = 0
            # self.get_logger().info(f"{msg.data}")
            self.pub_sum.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = sum_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()