#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import yaml
import os

from std_srvs.srv import Trigger
from sun_interfaces.srv import PizzaPose

class StartCopyNode(Node):
    def __init__(self):
        super().__init__('start_copy_node')
        
        # Define parameter for node name
        self.declare_parameter('name',"default")
        self.name = self.get_parameter('name').get_parameter_value().string_value
        
        # Create the service server
        self.read_pizza_pose_server = self.create_service(Trigger, '/read_pizza_pose', self.read_pizza_pose_callback)
        
        # Create the service clients for the four turtles
        self.spawnable_pizza_clients = {
            "Foxy": self.create_client(PizzaPose, 'Foxy/spawnable_pizza'),
            "Noetic": self.create_client(PizzaPose, 'Noetic/spawnable_pizza'),
            "Humble": self.create_client(PizzaPose, 'Humble/spawnable_pizza'),
            "Iron": self.create_client(PizzaPose, 'Iron/spawnable_pizza')
        }
        
        # Define the YAML file path where the 3D array was written
        self.yaml_file_path = os.path.expanduser('~/ros2_yaml_files/pizza_pose.yaml')
        
    def read_pizza_pose_callback(self, request, response):
        # Read the 3D array from the YAML file
        spawnable_pose = self.read_yaml(self.yaml_file_path)
        
        # Split the array into 4 sections for the 4 turtles
        sections = self.split_into_sections(spawnable_pose, 4)
        
        # Send each section to the corresponding turtle
        turtles = ["Foxy", "Noetic", "Humble", "Iron"]
        
        for i, turtle_name in enumerate(turtles):
            self.send_pizza_pose_to_turtle(turtle_name, sections[i])

        return response
    
    def read_yaml(self, yaml_file_path):
        """Reads the 3D array from a YAML file."""
        try:
            with open(yaml_file_path, 'r') as yaml_file:
                yaml_data = yaml.safe_load(yaml_file)

            # Extract the 3D array from YAML data
            my_3d_array = yaml_data['pizza_pose']
            self.get_logger().info(f"3D Array from YAML: {my_3d_array}")

            return my_3d_array
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found at {yaml_file_path}!")
        except KeyError:
            self.get_logger().error("Key 'pizza_pose' not found in the YAML file!")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {str(e)}")
    
    def split_into_sections(self, data, num_sections):
        """Splits the 3D array into a number of sections."""
        section_size = len(data) // num_sections
        return [data[i*section_size:(i+1)*section_size] for i in range(num_sections)]
    
    def send_pizza_pose_to_turtle(self, turtle_name, section):
        """Sends a section of the pizza pose array to a specific turtle."""
        request = PizzaPose.Request()
        temp_x, temp_y = [], []

        # Prepare the x and y data from the section
        for row in section:
            temp_x.append(row[0])
            temp_y.append(row[1])

        self.get_logger().info(f"Prepared data: x={temp_x}, y={temp_y}")
        request.x = temp_x
        request.y = temp_y

        # Make the asynchronous service call
        future = self.spawnable_pizza_clients[turtle_name].call_async(request)
        
        # Add the future to a list so we can check its status later
        future.add_done_callback(lambda future: self.pizza_pose_response_callback(turtle_name, future))

    def pizza_pose_response_callback(self, turtle_name, future):
        """Handles the response from the service call."""
        try:
            response = future.result()
            if response:
                self.get_logger().info(f"Successfully sent data to {turtle_name}")
            else:
                self.get_logger().error(f"Failed to send data to {turtle_name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = StartCopyNode()
    try:
        # Spin the node to handle callbacks and incoming service responses
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
