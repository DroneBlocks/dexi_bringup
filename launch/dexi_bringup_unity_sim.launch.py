from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Unity simulation with DEXI.
    Starts rosbridge, LED visualization bridge, and PX4 offboard manager.
    Note: micro_ros_agent runs in a separate container via docker-compose.
    """
    # Create the launch description
    ld = LaunchDescription()

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

    return ld
