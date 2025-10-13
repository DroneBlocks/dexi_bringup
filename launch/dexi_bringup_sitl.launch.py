from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for SITL (Software In The Loop) simulation.
    Starts micro_ros_agent with UDP and rosbridge server.
    """
    # Create the launch description
    ld = LaunchDescription()

    # Create micro_ros_agent node with UDP for SITL
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888']
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
        }]
    )
    ld.add_action(rosbridge_websocket)

    # Create rosapi node
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi'
    )
    ld.add_action(rosapi)

    return ld
