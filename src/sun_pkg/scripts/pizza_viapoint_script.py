#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import yaml
import os
import time

# Additional interfaces
from sun_interfaces.srv import PizzaPose
from turtlesim_plus_interfaces.srv import GivePosition
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
from turtlesim.msg import Pose

class PizzaViapointNode(Node):
    def __init__(self):
        super().__init__('pizza_viapoint_node')
        
        # Declare parameters
        self.declare_parameter('max_pizza', 20)
        self.declare_parameter('name', 't_name')
        self.name = self.get_parameter('name').get_parameter_value().string_value
        self.max_pizza = self.get_parameter('max_pizza').get_parameter_value().integer_value
        
        # Create the subscriber
        self.create_subscription(Pose,'/' +self.name + '/pose',self.turtle_pos, 10)
        
        # Create the service server
        self.clear_pizza_server = self.create_service(Trigger, 'clear_pizza', self.clear_pizza_callback)
        self.ready_to_spawn_pizza = self.create_service(Trigger, 'ready_to_spawn_pizza', self.spawn_pizza_request)
        self.save_pizza_server = self.create_service(Trigger, 'save_pizza', self.save_pizza_callback)
        self.get_logger().info('Service server ready to receive pizza position.')

        # Create the service client
        self.clear_pizza_to_controller = self.create_client(PizzaPose, 'eatable_pizza')
        self.spawn_pizza_client = self.create_client(GivePosition, 'spawn_pizza')
        self.read_pizza_pose_client = self.create_client(Trigger, '/read_pizza_pose')
        self.finish_auto_client = self.create_client(Trigger, 'finish_auto')
        
        # Define the output YAML file path
        output_dir = os.path.expanduser('~/ros2_yaml_files')  # Creates file in the home directory
        os.makedirs(output_dir, exist_ok=True)  # Make sure the directory exists
        self.yaml_file_path = os.path.join(output_dir, 'pizza_pose.yaml')

        # Check if the directory exists, create it if necessary
        directory = os.path.dirname(self.yaml_file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
            self.get_logger().info(f"Directory created: {directory}")
            
        # Additional variables
        self.count_pizza = 0
        self.keep_pose = []
        self.update_number = 0
        self.turtle_position = [0, 0]
           
        # Initial function that need to run
        self.clear_yaml(self.yaml_file_path)
        
        # Add a callback to dynamically update parameters
        self.add_on_set_parameters_callback(self.on_parameter_update)
        
    def turtle_pos(self,msg):
        self.turtle_position[0] = msg.x
        self.turtle_position[1] = msg.y
        
    def clear_pizza_callback(self, request, response):
        
        if self.count_pizza > 0 :
            temp_x = []
            temp_y = []
            
            for i in range (len(self.keep_pose)):
                temp_x.append(self.keep_pose[i][0])
                temp_y.append(self.keep_pose[i][1])
                
            clear_to_controller = PizzaPose.Request()
            clear_to_controller.x = temp_x
            clear_to_controller.y = temp_y
            clear_to_controller.number = 0
            
            self.clear_pizza_to_controller.call_async(clear_to_controller)
            self.get_logger().info("Start clear pizza")
            self.keep_pose = []
            self.count_pizza = 0
            
            return response
        else :
            self.get_logger().info("Don't have pizza on the floor")
            temp_req = Trigger.Request()
            self.finish_auto_client.call_async(temp_req)
            return response
    
    def spawn_pizza_request(self, request, response):
        
        if self.count_pizza < self.max_pizza:
            self.count_pizza += 1
            position_request = GivePosition.Request()
            position_request.x = self.turtle_position[0]
            position_request.y = self.turtle_position[1]
            self.keep_pose.append([position_request.x, position_request.y])
            self.spawn_pizza_client.call_async(position_request)
            self.get_logger().info(f'Pizza spawn: {self.count_pizza} pcs')
            return response
        else :
            self.get_logger().warning(f'Spawn pizza for 1 turtle is limit at {self.max_pizza} ')
            return response
    
    def save_pizza_callback(self, request, response):
        
        if self.update_number < 4:
            self.update_number += 1
            self.get_logger().info(f"Update pizza position {self.update_number}")
            
            # Handle the incoming 1D array of float64
            self.get_logger().info(f"Received pizza pose to save to pizza_pose.yaml")
            
            print(self.keep_pose, self.update_number)
            
            # Call the function to append data to YAML
            self.append_yaml(self.yaml_file_path, self.keep_pose, self.update_number)
            
            # Save data to .yaml
            self.get_logger().info(f"Completed to save data to .yaml file")
            self.count_pizza = 0
            self.keep_pose = []
            
            if self.update_number == 4:
                request = Trigger.Request()
                self.get_logger().info("Sending data to copy/turtlesim_plus")
                self.update_number += 1
                self.read_pizza_pose_client.call_async(request)
                # rclpy.spin_until_future_complete(self, future)
                # if future.result() is not None:
                #     self.get_logger().info(f"Successfully sent data to copy/turtlesim_plus")
                # else:
                #     self.get_logger().error(f"Failed to send data to copy/turtlesim_plus")
                
                return response
            return response
        else:
            self.get_logger().warning(f"Can't update pizza position after {self.update_number - 1} times")  
            return response
    
    def clear_yaml(self, file_path):
        self.write_yaml(file_path, {})  # Clears the YAML file by overwriting with an empty dictionary
        time.sleep(0.05)
        self.get_logger().info(f"All data successfully cleared in {file_path}")
        
    def append_yaml(self, yaml_file_path, data, number):
        """
        Appends a 2D array to the 'pizza_pose' key in the YAML file.
        Arguments:
        - data: The 2D array to append (a list of lists)
        """
        try:
            # Load existing content if the file exists
            if os.path.exists(yaml_file_path):
                with open(yaml_file_path, 'r') as yaml_file:
                    existing_data = yaml.safe_load(yaml_file) or {}
            else:
                existing_data = {}

            # If 'pizza_pose' key exists, append to the existing 2D array
            if 'pizza_pose' in existing_data:
                if isinstance(existing_data['pizza_pose'], list):
                    existing_data['pizza_pose'].extend(data)
                else:
                    self.get_logger().error("'pizza_pose' is not a list, unable to append.")
            else:
                # If 'pizza_pose' doesn't exist, create it as a 2D array
                existing_data['pizza_pose'] = data

            # Write back the updated content to the YAML file
            with open(yaml_file_path, 'w') as yaml_file:
                yaml.dump(existing_data, yaml_file, default_flow_style=False)

            self.get_logger().info(f'2D array appended to {yaml_file_path}')
            
        except Exception as e:
            self.get_logger().error(f"Failed to append to YAML file: {str(e)}")


    def write_yaml(self, file_path, data):
        """Write data to YAML file."""
        try:
            with open(file_path, 'w') as yaml_file:
                yaml.dump(data, yaml_file, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"Failed to write to {file_path}: {str(e)}")

    def on_parameter_update(self, params):
        # Callback to handle parameter updates
        for param in params:
            if param.name == 'max_pizza':
                self.max_pizza = param.value
                self.get_logger().info(f'Update max pizza changed to: {self.max_pizza} pcs')

        return SetParametersResult(successful=True)
    
def main(args=None):
    rclpy.init(args=args)
    node = PizzaViapointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()