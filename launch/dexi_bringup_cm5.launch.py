from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dexi_bringup = get_package_share_directory('dexi_bringup')
    
    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument('sitl', default_value='false', description='Enable SITL mode (UDP connection)'))
    ld.add_action(DeclareLaunchArgument('apriltags', default_value='false', description='Enable AprilTag detection'))
    ld.add_action(DeclareLaunchArgument('servos', default_value='false', description='Enable servo control'))
    ld.add_action(DeclareLaunchArgument('gpio', default_value='false', description='Enable GPIO control'))
    ld.add_action(DeclareLaunchArgument('offboard', default_value='false', description='Enable offboard control'))
    ld.add_action(DeclareLaunchArgument('keyboard_control', default_value='false', description='Enable keyboard teleop control'))
    ld.add_action(DeclareLaunchArgument('rosbridge', default_value='true', description='Enable ROS bridge'))
    ld.add_action(DeclareLaunchArgument('camera', default_value='true', description='Enable camera'))
    ld.add_action(DeclareLaunchArgument('camera_width', default_value='640', description='Camera width'))
    ld.add_action(DeclareLaunchArgument('camera_height', default_value='480', description='Camera height'))
    ld.add_action(DeclareLaunchArgument('camera_format', default_value='XRGB8888', description='Camera format'))
    ld.add_action(DeclareLaunchArgument('camera_jpeg_quality', default_value='60', description='Camera JPEG quality'))
    ld.add_action(DeclareLaunchArgument('yolo', default_value='false', description='Enable YOLO detection'))

    sitl = LaunchConfiguration('sitl')
    apriltags = LaunchConfiguration('apriltags')
    servos = LaunchConfiguration('servos')
    gpio = LaunchConfiguration('gpio')
    offboard = LaunchConfiguration('offboard')
    keyboard_control = LaunchConfiguration('keyboard_control')
    rosbridge = LaunchConfiguration('rosbridge')
    camera = LaunchConfiguration('camera')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_format = LaunchConfiguration('camera_format')
    camera_jpeg_quality = LaunchConfiguration('camera_jpeg_quality')
    yolo = LaunchConfiguration('yolo')
    
    # Create micro_ros_agent node for hardware (serial connection)
    micro_ros_agent_serial = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyAMA3', '-b', '921600'],
        condition=UnlessCondition(sitl)
    )
    ld.add_action(micro_ros_agent_serial)

    # Create micro_ros_agent node for SITL (UDP connection)
    micro_ros_agent_udp = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        condition=IfCondition(sitl)
    )
    ld.add_action(micro_ros_agent_udp)
    
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
    
    # Include CM5 LED service launch file
    led_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_led'), 'launch', 'led_service_pi5.launch.py')
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
            'format': camera_format,
            'width': camera_width,
            'height': camera_height,
            'jpeg_quality': camera_jpeg_quality,
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
            ('image_rect/compressed', '/cam0/image_raw/compressed_2hz'),
            ('camera_info', '/cam0/camera_info'),
            ('detections', '/apriltag_detections')
        ],
        parameters=[{
            'image_transport': 'compressed',
            'tag_family': '36h11',  # Standard AprilTag family
            'tag_size': 0.1,  # Size of the tag in meters
        }],
        condition=IfCondition(apriltags)
    )
    ld.add_action(apriltag_node)
    
    # Image throttle node for 2fps raw images - for YOLO detection
    image_throttle_raw_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_raw_node',
        arguments=['messages', '/cam0/image_raw', '2.0', '/cam0/image_raw/raw_2hz'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_raw_node)

    # Image throttle node for 2fps compressed images - for AprilTag detection
    image_throttle_compressed_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_compressed_node',
        arguments=['messages', '/cam0/image_raw/compressed', '2.0', '/cam0/image_raw/compressed_2hz'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_compressed_node)
    
    
    # GPIO launch file
    gpio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'tca9555_controller.launch.py')
        ]),
        condition=IfCondition(gpio)
    )
    ld.add_action(gpio_launch)

    # DEXI servo controller launch file
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'servo_controller.launch.py')
        ]),
        condition=IfCondition(servos)
    )
    ld.add_action(servo_launch)
    
    # YOLO node
    yolo_node = Node(
        package='dexi_yolo',
        executable='dexi_yolo_node_onnx.py',
        name='dexi_yolo_node',
        remappings=[
            ('/cam0/image_raw', '/cam0/image_raw/raw_2hz')
        ],
        condition=IfCondition(yolo)
    )
    ld.add_action(yolo_node)

    # Include offboard control nodes
    offboard_manager_node = Node(
        package='dexi_offboard',
        executable='px4_offboard_manager',
        name='px4_offboard_manager',
        namespace='dexi',
        output='screen',
        parameters=[{
            'keyboard_control_enabled': keyboard_control
        }],
        condition=IfCondition(offboard)
    )
    ld.add_action(offboard_manager_node)

    keyboard_teleop_node = Node(
        package='dexi_offboard',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        namespace='dexi',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(keyboard_control)
    )
    ld.add_action(keyboard_teleop_node)

    return ld 