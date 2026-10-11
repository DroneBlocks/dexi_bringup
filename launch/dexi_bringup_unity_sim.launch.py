from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Unity simulation with DEXI.
    Starts rosbridge, LED visualization bridge, PX4 offboard manager,
    the AprilTag detector, tag_nav, color detection and YOLO.
    Note: micro_ros_agent runs in a separate container via docker-compose.

    Launch arguments:
        color_detection: Enable HSV color detection on the camera feed (default: true)
        tag_nav: Launch tag_nav with tag_nav_config (default: true, tag_nav_sim.yaml)
        yolo: Launch dexi_yolo with yolo_model at yolo_frequency (default: true, avr_2026, 1 Hz)
    """
    # Declare launch arguments
    color_detection_arg = DeclareLaunchArgument(
        'color_detection',
        default_value='true',
        description='Enable HSV color detection on camera feed'
    )

    # Create the launch description
    ld = LaunchDescription()
    ld.add_action(color_detection_arg)
    # tag_nav (AprilTag navigation primitives) runs in the sim by default, with the sim
    # camera mount, so the Node-RED flow, the blocks and the Python examples work out of
    # the box. tag_nav:=false turns it off.
    ld.add_action(DeclareLaunchArgument('tag_nav', default_value='true', description='Launch tag_nav (dexi_apriltag)'))
    ld.add_action(DeclareLaunchArgument('tag_nav_config', default_value='tag_nav_sim.yaml', description='tag_nav parameter file in dexi_apriltag/config'))
    # Off until the sim image carries a dexi_cpp with telemetry_node; a missing
    # executable aborts the whole launch. telemetry:=true turns it on.
    ld.add_action(DeclareLaunchArgument('telemetry', default_value='false', description='Publish /dexi/telemetry (2 Hz summary for dashboards)'))
    # YOLO on the sim camera at 1 Hz, one thread: about a tenth of a vCPU. yolo:=false turns it off.
    ld.add_action(DeclareLaunchArgument('yolo', default_value='true', description='Launch dexi_yolo (ONNX) on the sim camera'))
    ld.add_action(DeclareLaunchArgument('yolo_model', default_value='avr_2026', description='dexi_yolo model profile or .onnx path'))
    ld.add_action(DeclareLaunchArgument('yolo_frequency', default_value='1.0', description='YOLO detection frequency, Hz'))

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
            # See the hardware bringups and DroneBlocks/dexi-os#44. The sim runs
            # the same rosbridge with the same never-ping default, so a browser
            # tab closed without a clean disconnect leaks here too.
            # The pinned rosbridge declares this a double.
            'websocket_ping_interval': 10.0,
        }],
        output='screen'
    )
    ld.add_action(rosbridge_websocket)

    # Platform params node — exposes platform identity and feature flags for the web dashboard
    platform_params = Node(
        package='dexi_bringup',
        executable='platform_params_node',
        name='dexi_platform_params',
        parameters=[{
            'dexi_platform': 'unity_sim',
            'dexi_keyboard_control': True,
        }],
    )
    ld.add_action(platform_params)

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
            # No tag.ids list: apriltag_ros drops ids not in a given list. Without one,
            # every tag36h11 id is published with a TF framed tag36h11:<id> at the
            # default size, which apriltag_odometry, tag_hop and tag_nav look up.
        }],
        output='screen'
    )
    ld.add_action(apriltag_node)

    # Static transform: base_link -> camera (downward-facing mount, pitch 90 deg), as on
    # the aircraft, so base_link -> tag lookups work in the sim too.
    base_link_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf',
        arguments=['0', '0', '0', '0', '1.5708', '0', 'base_link', 'camera'],
    )
    ld.add_action(base_link_to_camera_tf)

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

    tag_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('dexi_apriltag'), 'launch', 'tag_nav.launch.py'])),
        launch_arguments={'config': LaunchConfiguration('tag_nav_config')}.items(),
        condition=IfCondition(LaunchConfiguration('tag_nav'))
    )
    ld.add_action(tag_nav_launch)

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('dexi_yolo'), 'launch', 'yolo_onnx_launch.py'])),
        launch_arguments={'model': LaunchConfiguration('yolo_model'),
                          'detection_frequency': LaunchConfiguration('yolo_frequency'),
                          'num_threads': '1'}.items(),
        condition=IfCondition(LaunchConfiguration('yolo'))
    )
    ld.add_action(yolo_launch)

    # /dexi/telemetry: a 2 Hz JSON summary of the PX4 topics for dashboards and
    # Node-RED. Subscribing to /fmu/out directly through rosbridge costs about a
    # core on a CM4; this node costs about 4%.
    telemetry_node = Node(
        package='dexi_cpp',
        executable='telemetry_node',
        name='telemetry',
        output='screen',
        condition=IfCondition(LaunchConfiguration('telemetry'))
    )
    ld.add_action(telemetry_node)

    return ld
