#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import sys, select, termios, tty
settings = termios.tcgetattr(sys.stdin)

from turtlesim_plus_interfaces.srv import GivePosition
from turtlesim.msg import Pose
from sun_interfaces.srv import PizzaPose


# getKey function (based on teleop_twist_keyboard)
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

class TeleopTurtle(Node):
    def __init__(self):
        super().__init__('teleop_turtle')
        self.get_logger().info('TeleopTurtleNode has been started')
        
        self.name = 't_name'
        # tele = "Teleop"
        # ros2 run saifa_pack teleop_turtle.py --ros-args -p name:="t_name"
        self.publisher = self.create_publisher(Twist, '/' +self.name +'/cmd_vel', 10)
        self.create_subscription(Pose,'/' +self.name + '/pose',self.turtle_pos, 10)
        
        self.clear_pizza_client = self.create_client(Trigger, 'clear_pizza')
        self.ready_to_spawn_pizza = self.create_client(Trigger, 'ready_to_spawn_pizza')
        self.save_pizza_client = self.create_client(Trigger, 'save_pizza')
        self.eatable_pizza_client = self.create_client(PizzaPose, 'eatable_pizza')
        self.finish_auto_server = self.create_service(Trigger, 'finish_auto', self.finish_auto_callback)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.velocity = Twist()
        self.exit_flag = False
        self.turtle_position = [0,0]
        self.save_times = 0
        self.count_pizza = 0
        self.is_auto = 0
        
    def turtle_pos(self,msg):
        self.turtle_position[0] = msg.x
        self.turtle_position[1] = msg.y

    def timer_callback(self):
        key = getKey()
        if key == 'q':
            self.exit_flag = True
            return
        elif key == '\x03':  # Ctrl+C to exit
            self.exit_flag = True
            return
        
        if self.is_auto == 0:
            if key == 'w':
                self.velocity.linear.x = 2.0
                self.velocity.angular.z = 0.0
            elif key == 's':
                self.velocity.linear.x = -2.0
                self.velocity.angular.z = 0.0
            elif key == 'a':
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 2.0
            elif key == 'd':
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = -2.0
            elif key == 'r':
                position_request = Trigger.Request()
                self.ready_to_spawn_pizza.call_async(position_request)
                    
            elif key == 'u':
                # write to yaml
                self.save_pizza_request()
                    
            elif key == 'c':
                # eat all of the keep pose -> send keep pose to teleop controller 
                request = Trigger.Request()
                self.clear_pizza_client.call_async(request)
                self.get_logger().info("Clear pizza")
                self.is_auto = 1
                
            elif key == 'e':
                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.0
                
            # Publish the velocity
            self.publisher.publish(self.velocity)
        else :
            self.get_logger().warning('In auto mode can not press any key')
              
    def eatable_pizza_request(self, x, y):
        # Wait for the service to be available
        if not self.eatable_pizza_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Eatable pizza service not available.')
            return
        
        # Prepare the request message
        request = PizzaPose.Request()
        request.x = x
        request.y = y
        request.number = 0
        
        # Call the service and wait for a response
        future = self.eatable_pizza_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result().isfinish}')
        else:
            self.get_logger().error('Service call failed')
    
    def save_pizza_request(self):
        # Wait for the service to be available
        if not self.save_pizza_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Save pizza service not available.')
            return
        
        # Prepare the request message
        request = Trigger.Request()
        
        # Call the service
        self.save_pizza_client.call_async(request)
    
    def finish_auto_callback(self, request, response):
        self.is_auto = 0
        return response

def main(args=None):
  
    if args is None:
        args = sys.argv
        
    rclpy.init(args=args)
    teleop_turtle = TeleopTurtle()

    try:
        while rclpy.ok() and not teleop_turtle.exit_flag:
            rclpy.spin_once(teleop_turtle)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_turtle.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()