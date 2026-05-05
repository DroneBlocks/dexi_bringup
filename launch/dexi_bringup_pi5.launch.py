from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    ld.add_action(DeclareLaunchArgument('apriltags', default_value='false', description='Enable AprilTag detection'))
    ld.add_action(DeclareLaunchArgument('servos', default_value='false', description='Enable servo control'))
    ld.add_action(DeclareLaunchArgument('gpio', default_value='false', description='Enable GPIO control'))
    ld.add_action(DeclareLaunchArgument('offboard', default_value='false', description='Enable offboard control'))
    ld.add_action(DeclareLaunchArgument('keyboard_control', default_value='false', description='Enable keyboard teleop control'))
    ld.add_action(DeclareLaunchArgument('rosbridge', default_value='true', description='Enable ROS bridge'))
    ld.add_action(DeclareLaunchArgument('camera', default_value='true', description='Enable camera'))
    ld.add_action(DeclareLaunchArgument('camera_width', default_value='1280', description='Camera capture width in pixels'))
    ld.add_action(DeclareLaunchArgument('camera_height', default_value='720', description='Camera capture height in pixels'))
    ld.add_action(DeclareLaunchArgument('camera_jpeg_quality', default_value='60', description='JPEG compression quality (0-100)'))
    ld.add_action(DeclareLaunchArgument('camera_timer_interval', default_value='0.033', description='Camera capture timer interval in seconds (1/fps)'))
    ld.add_action(DeclareLaunchArgument('yolo', default_value='false', description='Enable YOLO detection'))
    ld.add_action(DeclareLaunchArgument('color_detection', default_value='true', description='Enable HSV color detection'))

    apriltags = LaunchConfiguration('apriltags')
    servos = LaunchConfiguration('servos')
    gpio = LaunchConfiguration('gpio')
    offboard = LaunchConfiguration('offboard')
    keyboard_control = LaunchConfiguration('keyboard_control')
    rosbridge = LaunchConfiguration('rosbridge')
    camera = LaunchConfiguration('camera')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    camera_jpeg_quality = LaunchConfiguration('camera_jpeg_quality')
    camera_timer_interval = LaunchConfiguration('camera_timer_interval')
    yolo = LaunchConfiguration('yolo')
    color_detection = LaunchConfiguration('color_detection')
    
    # Create micro_ros_agent node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyAMA3', '-b', '921600']
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

    # Platform params node — exposes platform identity and feature flags for the web dashboard
    platform_params = Node(
        package='dexi_bringup',
        executable='platform_params_node',
        name='dexi_platform_params',
        parameters=[{
            'dexi_platform': 'pi5',
            'dexi_keyboard_control': keyboard_control,
        }],
        condition=IfCondition(rosbridge)
    )
    ld.add_action(platform_params)
    
    # Create rosapi node
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        condition=IfCondition(rosbridge)
    )
    ld.add_action(rosapi)
    
    # Include Pi5 LED service launch file
    led_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_led'), 'launch', 'led_service_pi5.launch.py')
        ])
    )
    ld.add_action(led_launch)
    
    # Camera launch file for Pi5 using dexi_camera (UVC camera).
    # The Arducam 12MP UVC only supports 1280x720 and larger as native
    # MJPG modes, so the Pi 5 platform config default is 1280x720 — see
    # config/dexi_config_pi5.yaml. Setting smaller output sizes in
    # ~/.dexi-config.yaml would require downscaling support in
    # dexi_camera, which is not yet implemented.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_camera'), 'camera.launch.py')
        ]),
        launch_arguments={
            'camera_width': camera_width,
            'camera_height': camera_height,
            'jpeg_quality': camera_jpeg_quality,
            'timer_interval': camera_timer_interval,
        }.items(),
        condition=IfCondition(camera)
    )
    ld.add_action(camera_launch)
    
    # AprilTag node - uses full-rate camera streams (no throttling).
    # tag.ids/sizes/frames are required for apriltag_ros to publish TF poses;
    # without them the node detects tags in 2D but downstream consumers
    # (apriltag_odometry, tag_hop, precision_landing) can't look up TFs.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect/compressed', '/cam0/image_raw/compressed'),
            ('camera_info', '/cam0/camera_info'),
            ('detections', '/apriltag_detections')
        ],
        parameters=[{
            'image_transport': 'compressed',
            'family': '36h11',  # Standard AprilTag family
            'size': 0.1,  # Size of the tag in meters
            'tag.ids': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
            'tag.sizes': [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
            'tag.frames': [
                'tag36h11:0', 'tag36h11:1', 'tag36h11:2', 'tag36h11:3', 'tag36h11:4',
                'tag36h11:5', 'tag36h11:6', 'tag36h11:7', 'tag36h11:8', 'tag36h11:9',
            ],
        }],
        condition=IfCondition(apriltags)
    )
    ld.add_action(apriltag_node)

    # Static transform: base_link -> camera (downward-facing mount, pitch 90°).
    # Required for downstream nodes that look up tag TFs in body frame.
    base_link_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf',
        arguments=['0', '0', '0', '0', '1.5708', '0', 'base_link', 'camera'],
        condition=IfCondition(apriltags)
    )
    ld.add_action(base_link_to_camera_tf)

    # Image throttle node for 2fps raw images - for YOLO detection
    image_throttle_raw_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_raw_node',
        arguments=['messages', '/cam0/image_raw', '2.0', '/cam0/image_raw/raw_2hz'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_raw_node)
    
    # YOLO throttle: 2 FPS for object detection
    image_throttle_yolo_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_yolo_node',
        arguments=['messages', '/cam0/image_raw/compressed', '2.0', '/cam0/image_raw/compressed_2hz_yolo'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_yolo_node)
    
    # DEXI servo controller launch file
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
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'tca9555_controller.launch.py')
        ]),
        condition=IfCondition(gpio)
    )
    ld.add_action(gpio_launch)
    
    # YOLO node - optimized for Pi5
    yolo_node = Node(
        package='dexi_yolo',
        executable='dexi_yolo_node_onnx.py',
        name='dexi_yolo_node',
        remappings=[
            ('/cam0/image_raw/compressed', '/cam0/image_raw/compressed_2hz_yolo')
        ],
        parameters=[{
            'input_size': 320,           # Model trained at 320x320
            'num_threads': 1,            # Single thread to avoid CPU contention
            'detection_frequency': 2.0,  # Process 2 frames per second (matches throttle rate)
            'use_letterbox': True,       # Enable to match training preprocessing (rect=False)
            'confidence_threshold': 0.5, # Lowered from 0.65 (sigmoid fix allows proper filtering)
            'nms_threshold': 0.4,
            'verbose_logging': False,    # Disable verbose logging to save CPU
            'max_detections': 10,        # Limit max detections to reduce processing
        }],
        condition=IfCondition(yolo)
    )
    ld.add_action(yolo_node)

    # Color detection node
    color_detection_node = Node(
        package='dexi_color_detection',
        executable='color_detection_node.py',
        name='color_detection_node',
        parameters=[{
            'detection_frequency': 5.0,
            'min_contour_area': 500,
            'publish_annotated_image': True,
        }],
        condition=IfCondition(color_detection)
    )
    ld.add_action(color_detection_node)

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