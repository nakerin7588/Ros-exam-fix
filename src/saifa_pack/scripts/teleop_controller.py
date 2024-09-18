#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import threading
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from controller_interfaces.srv import SetTarget, SetParam
from sun_interfaces.srv import PizzaPose
from std_srvs.srv import Trigger

class teleop_controller(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        # Declare PID control parameters
        self.declare_parameter('kp', 6.0)
        self.kp = self.get_parameter('kp').get_parameter_value().double_value

        self.declare_parameter('ki', 10.0)
        self.kp_a = self.get_parameter('ki').get_parameter_value().double_value

        # Publisher and service creation
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.unsave_p_pose = self.create_service(PizzaPose, "/eatable_pizza", self.unsave_p)
        self.eat_pizza_client = self.create_client(Empty, 'eat')
        self.finish_auto = self.create_client(Trigger, '/finish_auto')
        
        # Timer for callback execution
        self.create_timer(0.1, self.timmer_callback)

        # Pose subscription
        self.create_subscription(Pose, 'pose', self.pose_callback, 10)

        # Event to synchronize threads and task management
        self.task_completed_event = threading.Event()
        self.timer_task_active = False  # Flag to indicate if the timer task is running

        # State management and goal tracking
        self.x = []
        self.y = []
        self.all_p = []
        self.goal_pose = []
        self.state = 0
        self.R_pose = [0, 0, 0]  # Robot's current pose (x, y, theta)

    def start_auto_callback(self):
        pass
    
    def set_Param(self, request: SetParam.Request, response: SetParam.Response):
        self.kp = request.kp_linear.data
        self.kp_a = request._kp_angular.data
        return response

    def pose_callback(self, msg):
        # Update robot's current pose
        self.R_pose[0] = msg.x
        self.R_pose[1] = msg.y
        self.R_pose[2] = msg.theta

    def unsave_p(self, request: PizzaPose.Request, response: PizzaPose.Response):
        self.get_logger().info("IN1")
        self.task_completed_event.clear()  # Clear the event to reset synchronization
        self.get_logger().info("IN2")

        # Reset goal_pose and all_p for new request
        self.all_p = []
        for i in range(len(request.x)):
            self.all_p.append([request.x[i], request.y[i]])

        self.get_logger().info("IN3")
        self.goal_pose = self.all_p
        self.state = 1  # Set the state to start navigation
        self.get_logger().info("IN4")

        # Signal that the timer task should start processing
        self.timer_task_active = True
        self.state = 1

        return response

    def cmdvel(self, v, w):
        # Publish velocity commands
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def eat_pizza(self):
        # Call the "eat pizza" service
        eat_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_request)

    def timmer_callback(self):
        # Timer task runs every 0.1 seconds
        if self.timer_task_active == True:  # If state is 1, it means the robot is navigating
            self.get_logger().info("IN6 - Moving towards goal")

            if len(self.goal_pose) == 0:
                # No goals remaining, stop the robot
                d_x = 0
                d_y = 0
            else:
                # Calculate distance to the next goal
                d_x = self.goal_pose[0][0] - self.R_pose[0]
                d_y = self.goal_pose[0][1] - self.R_pose[1]
                self.get_logger().info("IN7 - New goal")

            # Calculate Euclidean distance and angle error
            d = math.sqrt((d_x * d_x) + (d_y * d_y))
            turtle_angle = math.atan2(d_y, d_x)
            angular_error = turtle_angle - self.R_pose[2]
            e = math.atan2(math.sin(angular_error), math.cos(angular_error))

            if d >= 0.05:
                # Move towards the goal if distance is significant
                # Wrap around
                if e > math.pi:
                    e -= 2*math.pi
                elif e < -math.pi:
                    e += 2*math.pi
                    
                vx = self.kp * d
                w = self.kp_a * e
            else:
                # Stop moving and "eat pizza" if the robot is close enough
                vx = 0.0
                w = 0.0
                if len(self.goal_pose) > 0:
                    self.get_logger().info(f"Eating pizza at goal {len(self.goal_pose)}")
                    self.eat_pizza()
                    self.goal_pose.pop(0)  # Remove the current goal

                if len(self.goal_pose) == 0:
                    self.timer_task_active = False

            # Publish velocity commands
            self.cmdvel(vx, w)
            
        if self.timer_task_active == False and self.state == 1:
            # Once the task is done, reset and stop movement
            self.get_logger().info("Timer task is done. Stopping movement.")
            self.cmdvel(0.0, 0.0)  # Stop the robot
            self.state = 0  # Reset state
            request = Trigger.Request()
            self.finish_auto.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = teleop_controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()