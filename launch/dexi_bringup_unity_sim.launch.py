from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(challenge_arg)

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
            'keyboard_control_enabled': True
        }],
        output='screen',
        emulate_tty=True
    )
    ld.add_action(px4_offboard_manager)

    # AprilTag node for Unity camera stream
    # Unity publishes compressed images to /image_rect/compressed and /camera_info
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/image_rect'),
            ('camera_info', '/camera_info'),
            ('detections', '/apriltag_detections')
        ],
        parameters=[{
            'image_transport': 'compressed',  # Unity publishes compressed images
            'tag_family': '36h11',
            'tag_size': 0.1,  # Size of the tag in meters
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

    return ld
