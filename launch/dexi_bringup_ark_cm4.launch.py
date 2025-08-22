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
    rosbridge = LaunchConfiguration('rosbridge', default='false')
    camera = LaunchConfiguration('camera', default='true')
    yolo = LaunchConfiguration('yolo', default='false')
    
    # Create micro_ros_agent node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyAMA2', '-b', '3000000'],
        condition=IfCondition(rosbridge)
    )
    ld.add_action(micro_ros_agent)
    
    # Create rosbridge websocket node
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '',
            'ssl': False,
            'certfile': '',
            'keyfile': '',
            'authenticate': False,
        }],
        condition=IfCondition(rosbridge)
    )
    ld.add_action(rosbridge_websocket)
    
    # Create rosapi node
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        condition=IfCondition(rosbridge)
    )
    ld.add_action(rosapi)
    
    # Include LED service launch file
    led_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_led'), 'launch', 'led_service.launch.py')
        ])
    )
    ld.add_action(led_launch)
    
    # Camera node for CM4 using camera_ros
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='cam0',
        remappings=[
            ('image_raw', '/cam0/image_raw'),
            ('camera_info', '/cam0/camera_info')
        ],
        parameters=[{
            'camera': '/base/soc/i2c0mux/i2c@1/imx219@10',  # Specify the camera device
            'format': 'XRGB8888',
            'width': 320,
            'height': 240, 
            'jpeg_quality': 60,
            'camera_info_url': 'file://' + os.path.join(get_package_share_directory('dexi_camera'), 'config', 'picam_2.1_csi.yaml'),  # Use calibration file from dexi_camera package
            'frame_id': 'camera',
            'camera_name': 'cam0'
        }],
        condition=IfCondition(camera)
    )
    ld.add_action(camera_node)
    
    # AprilTag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/cam0/image_raw'),
            ('image_rect/compressed', '/cam0/image_raw/compressed'),
            ('camera_info', '/cam0/camera_info')
        ],
        parameters=[{
            'image_transport': 'compressed',
            'image_topic': '/cam0/image_raw/compressed',
            'camera_info_topic': '/cam0/camera_info',
            'tag_family': '36h11',  # Standard AprilTag family
            'tag_size': 0.1,  # Size of the tag in meters
        }],
        condition=IfCondition(apriltags)
    )
    ld.add_action(apriltag_node)
    
    # Throttle node for detections
    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='throttle_node',
        arguments=['messages', '/detections', '1', '/throttled/detections'],
        condition=IfCondition(apriltags)
    )
    ld.add_action(throttle_node)
    
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
    
    # YOLO node
    yolo_node = Node(
        package='dexi_yolo',
        executable='dexi_yolo_node_onnx.py',
        name='dexi_yolo_node',
        condition=IfCondition(yolo)
    )
    ld.add_action(yolo_node)
    
    return ld 