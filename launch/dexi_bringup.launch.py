from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dexi_bringup = get_package_share_directory('dexi_bringup')
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Declare the launch arguments
    apriltags = LaunchConfiguration('apriltags', default='false')
    servos = LaunchConfiguration('servos', default='false')
    gpio = LaunchConfiguration('gpio', default='false')
    
    # Include rosbridge server launch file
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        ])
    )
    ld.add_action(rosbridge_launch)
    
    # Include LED service launch file
    led_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_led'), 'launch', 'led_service.launch.py')
        ])
    )
    ld.add_action(led_launch)
    
    # AprilTag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        condition=IfCondition(apriltags)
    )
    ld.add_action(apriltag_node)
    
    # Servo controller launch file
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'servo_controller.launch.py')
        ]),
        condition=IfCondition(servos)
    )
    ld.add_action(servo_launch)
    
    # GPIO launch file
    gpio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'gpio.launch.py')
        ]),
        condition=IfCondition(gpio)
    )
    ld.add_action(gpio_launch)
    
    return ld 