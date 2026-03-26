from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Unity simulation with DEXI.
    Starts rosbridge, LED visualization bridge, PX4 offboard manager,
    and CTF challenge runner.
    Note: micro_ros_agent runs in a separate container via docker-compose.

    Launch arguments:
        challenge: The challenge ID to auto-start (default: 'arm_basic')
                   Examples: 'arm_basic', 'takeoff_basic', or '' for no auto-start
    """
    # Declare launch arguments
    challenge_arg = DeclareLaunchArgument(
        'challenge',
        default_value='arm_basic',
        description='Challenge ID to auto-start (e.g., arm_basic, takeoff_basic, or empty for none)'
    )
    color_detection_arg = DeclareLaunchArgument(
        'color_detection',
        default_value='true',
        description='Enable HSV color detection on camera feed'
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(challenge_arg)
    ld.add_action(color_detection_arg)

    # Note: micro_ros_agent runs in its own container via docker-compose

    # Create rosbridge websocket node for Unity communication
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
            'default_call_service_timeout': 120.0,  # Flight commands (takeoff, land) need >5s default
        }],
        output='screen'
    )
    ld.add_action(rosbridge_websocket)

    # Create rosapi node
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi'
    )
    ld.add_action(rosapi)

    # LED Unity Bridge for Unity sim
    # Publishes to /dexi/led_state for Unity visualization
    led_unity_bridge = Node(
        package='dexi_led',
        executable='led_unity_bridge',
        name='led_service',
        namespace='dexi',
        parameters=[{
            'led_count': 45,
            'brightness': 0.2,
            'publish_rate': 15.0
        }],
        output='screen'
    )
    ld.add_action(led_unity_bridge)

    # PX4 Offboard Manager for drone control
    px4_offboard_manager = Node(
        package='dexi_offboard',
        executable='px4_offboard_manager',
        name='px4_offboard_manager',
        namespace='dexi',
        parameters=[{
            'keyboard_control_enabled': True  # Enabled for simulator keyboard/velocity control
        }],
        output='screen',
        emulate_tty=True
    )
    ld.add_action(px4_offboard_manager)

    # AprilTag node for Unity camera stream
    # Unity publishes compressed images to /cam0/image_raw/compressed and /cam0/camera_info
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/cam0/image_raw'),
            ('camera_info', '/cam0/camera_info'),
            ('detections', '/apriltag_detections')
        ],
        parameters=[{
            'image_transport': 'compressed',  # Unity publishes compressed images via rosbridge
            'family': '36h11',
            'size': 0.15,  # Size of the tag in meters (matches Unity Home Tag scale)
        }],
        output='screen'
    )
    ld.add_action(apriltag_node)

    # CTF Challenge Runner
    challenge_runner = Node(
        package='dexi_ctf',
        executable='challenge_runner.py',
        name='challenge_runner',
        parameters=[{
            'auto_start_challenge': LaunchConfiguration('challenge'),
        }],
        output='screen'
    )
    ld.add_action(challenge_runner)

    # Color detection node (subscribes to Unity camera feed)
    color_detection_node = Node(
        package='dexi_color_detection',
        executable='color_detection_node.py',
        name='color_detection_node',
        parameters=[{
            'detection_frequency': 5.0,
            'min_contour_area': 500,
            'publish_annotated_image': True,
        }],
        condition=IfCondition(LaunchConfiguration('color_detection'))
    )
    ld.add_action(color_detection_node)

    return ld
