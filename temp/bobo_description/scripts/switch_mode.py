#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch import LaunchService
from launch_ros.actions import Node as LaunchNode
from launch.actions import Shutdown

import time
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

class SwitchMode(Node):

    description_pkg = get_package_share_directory('bobo_description')

    def __init__(self):
        super().__init__('switch_mode')
        self.launch_service = LaunchService()
        self.current_launch = None

        description_pkg = get_package_share_directory('bobo_description')

        # Declare file paths for the launch files
        self.navigation_launch_file = os.path.join(description_pkg,'launch','nav2.launch.py')
        self.cartographer_launch_file = os.path.join(description_pkg,'launch','mapping.launch.py')
        self.remapping_launch_file = os.path.join(description_pkg,'launch','remapping.launch.py')

        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'output_topic',
            self.mode_callback,
            10
        )

    def mode_callback(self, msg):
        
        global mode
        global running_mode

        mode = msg.data
        running_mode = None

        # if mode != running_mode:
        if self.launch_service:
            self.get_logger().info(f'Shutting down current launch: {self.current_launch}')
            self.launch_service.shutdown()
            time.sleep(3)  # Wait for shutdown

            if mode == 'navi':
                self.start_launch(self.navigation_launch_file, 'Navigation')
            if mode == 'mapping':
                self.start_launch(self.cartographer_launch_file, 'Cartographer')
            if mode == 'remapping':
                self.start_launch(self.remapping_launch_file, 'Remapping')
            
        running_mode = mode
        self.get_logger().info(f"running node : {running_mode}")
        

    def start_launch(self, launch_file, launch_name):
        self.get_logger().info(f'Starting {launch_name} launch...')
        launch_description = self.get_launch_description(launch_file)
        self.launch_service.include_launch_description(launch_description)
        self.current_launch = self.launch_service.run()
        

    def get_launch_description(self, launch_file):
        """Returns the launch description from the provided file."""
        # return LaunchNode(
        #     package= "bobo_description",
        #     executable=launch_file,
        #     output='screen'
        launch_service = LaunchService()

    # Create a LaunchDescription using the path to the launch file
        launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file)
        )

    # Add the launch description to the launch service
        launch_service.include_launch_description(launch_description)

    # Run the launch service to execute the launch file
        print(f"Launching {launch_file}")
        return launch_service.run()

def main(args=None):
    rclpy.init(args=args)
    node = SwitchMode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
