#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import threading
from std_msgs.msg import String, Int32
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from controller_interfaces.srv import SetParam
from sun_interfaces.srv import PizzaPose
import math


class cp_controller(Node):
    def __init__(self):
        super().__init__('cp_controller')

        # Declare parameters
        self.declare_parameter('kp', 6.0)
        self.kp = self.get_parameter('kp').get_parameter_value().double_value

        self.declare_parameter('ki', 10.0)
        self.kp_a = self.get_parameter('ki').get_parameter_value().double_value

        self.declare_parameter('name', "default")
        self.name = self.get_parameter('name').get_parameter_value().string_value

        self.declare_parameter('num', 0)
        self.num = self.get_parameter('num').get_parameter_value().integer_value


        
        # Publishers and services
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.name}/cmd_vel', 10)
        self.spawnable_pose = self.create_service(PizzaPose, f'{self.name}/spawnable_pizza', self.spawnable_pizza)
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
        self.check_isfinish = self.create_publisher(Int32, f'{self.name}/isfinish', 10)

        # Subscriptions
        self.create_subscription(Pose, f'{self.name}/pose', self.pose_callback, 10)
        self.create_subscription(Int32, '/sum_callback', self.sum_callback, 10)

        # Control state variables
        self.x = []
        self.y = []
        self.all_p = []
        self.goal_pose = [[0, 0]]
        self.state = 0
        self.last_pose = [0]
        self.R_pose = [0, 0, 0]
        self.do = 0
        self.count = 0
        self.i = 0
        self.sum = 0
        self.stop = 0

        msg = Int32()
        msg.data = 0
        self.check_isfinish.publish(msg)
        self.create_timer(0.1, self.timmer_callback)

    def set_Param(self, request: SetParam.Request, response: SetParam.Response):
        self.kp = request.kp_linear.data
        self.kp_a = request._kp_angular.data
        return response

    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        
        # Call the service to spawn the pizza asynchronously
        future = self.spawn_pizza_client.call_async(position_request)
        future.add_done_callback(self.pizza_spawn_response_callback)

    def pizza_spawn_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Pizza spawn successful!")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn pizza: {str(e)}")

    def pose_callback(self, msg):
        self.R_pose[0] = msg.x
        self.R_pose[1] = msg.y
        self.R_pose[2] = msg.theta

    def spawnable_pizza(self, request: PizzaPose.Request, response: PizzaPose.Response):
        # Receive goal positions and assign them to the controller
        self.x = request.x
        self.y = request.y
        self.num = request.number
        response.isfinish = 1
        
        self.get_logger().info(f"Received x = {len(self.x)} positions.")
        self.get_logger().info(f"Received y = {len(self.y)} positions.")
        for i in range(len(self.x)):
            self.all_p.append([self.x[i], self.y[i]])

        self.get_logger().info(f"{self.x},{self.y}")
        self.get_logger().info(f"Received {len(self.all_p)} positions.")
        self.goal_pose = self.all_p
        self.get_logger().info(f"{self.goal_pose}")
        self.state = 1  # Start the movement towards the first goal
        self.get_logger().info(f"{self.state}")
        return response

    def sum_callback(self, msg):
        self.sum = msg.data
        # self.get_logger().info(f"sum is {self.sum}")
            

    def timmer_callback(self):
        # Timer for movement control
        # self.get_logger().info(f"check2")
        if self.state == 1:
            if self.count < len(self.goal_pose):
                self.last_pose = self.goal_pose[self.i]
                self.get_logger().info(f"x: {self.last_pose[0]}")
                self.get_logger().info(f"y: {self.last_pose[1]}")
                
            d_x = self.last_pose[0] - self.R_pose[0]
            d_y = self.last_pose[1] - self.R_pose[1]
            d = math.sqrt((d_x * d_x) + (d_y * d_y))
            # self.get_logger().info(f"d: {d}")
            turtle_angle = math.atan2(d_y, d_x)
            angular_error = turtle_angle - self.R_pose[2]
            e = math.atan2(math.sin(angular_error), math.cos(angular_error))

            # If close to the goal, spawn a pizza and proceed to the next goal
            if d >= 0.05 and abs(angular_error) > 0.1:
                # Move towards the goal if distance is significant
                # Wrap around
                if e > math.pi:
                    e -= 2*math.pi
                elif e < -math.pi:
                    e += 2*math.pi
                vx = self.kp * d
                w = self.kp_a * e
            else:
                # Stop moving and "spawn pizza" if the robot is close enough
                vx = 0.0
                w = 0.0

                if  self.count == len(self.goal_pose):
                    # Signal completion when all goals are reached
                    msg = Int32()
                    msg.data = 1
                    self.check_isfinish.publish(msg)
                    # Wait until all turtles have finished
                    if self.sum == 0:
                        self.get_logger().info(f"Waiting for all turtles to finish. Current sum: {self.sum}")
                        # rclpy.spin_once(self)
                        return
                    self.get_logger().info(f"Waiting for all turtles to finish. Current sum: {self.sum}")
                    self.goal_pose.append([9.9, 9.9])
                    self.stop = 1
                elif self.stop == 1:
                    self.get_logger().info(f"Finish turtle sim")
                    self.state = 0
                else:
                    self.get_logger().info("Next goal")
                    self.spawn_pizza(float(self.last_pose[0]), float(self.last_pose[1]))
                    self.count += 1
                    self.i += 1

            self.cmdvel(vx, w)

    def cmdvel(self, v, w):
        # Publish velocity commands
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = cp_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
