from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, LogInfo, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import glob
import os

# Node, capture size and calibration must move together: a calibration
# taken at a different resolution biases every AprilTag range.
CAMERA_PROFILE_PACKAGES = {'usb': 'dexi_camera', 'csi': 'camera_ros'}

CAMERA_PROFILES = {
    'usb': {
        'width': 1280, 'height': 720,
        'calibration': 'arducam_12mp_uvc.yaml',
    },
    'csi': {
        'width': 640, 'height': 480,
        'calibration': 'picam_2.1_csi.yaml',
    },
}


def detect_camera():
    """Return 'csi', 'usb' or None. CSI wins when both are fitted."""
    # Match the i2c bus-address entry (e.g. 10-0010), not uevent/bind/module,
    # which exist whenever the driver is loaded with no sensor attached.
    if glob.glob('/sys/bus/i2c/drivers/imx*/*-00*'):
        return 'csi'
    if glob.glob('/dev/v4l/by-id/*Arducam*'):
        return 'usb'
    return None


def usb_video_index():
    """V4L2 index of the Arducam. The index moves depending on what else
    is attached, so resolve it through the stable by-id path."""
    for link in sorted(glob.glob('/dev/v4l/by-id/*Arducam*video-index0')):
        return os.path.realpath(link).replace('/dev/video', '')
    return '0'


def package_available(name):
    try:
        get_package_share_directory(name)
        return True
    except (PackageNotFoundError, KeyError):
        return False


def select_camera(context, *args, **kwargs):
    requested = LaunchConfiguration('camera_type').perform(context)
    choice = detect_camera() if requested == 'auto' else requested

    if choice is None:
        return [LogInfo(msg='CAMERA: none detected - camera disabled')]

    # Never let a missing camera package take the rest of the stack down
    # with it: fall back if the other path is usable, otherwise run without
    # a camera. camera_ros in particular is in dexi.repos but is not always
    # built into the image.
    if not package_available(CAMERA_PROFILE_PACKAGES[choice]):
        missing = CAMERA_PROFILE_PACKAGES[choice]
        fallback = 'usb' if choice == 'csi' else 'csi'
        if package_available(CAMERA_PROFILE_PACKAGES[fallback]):
            return [LogInfo(msg='CAMERA: %s selected but package %s is not built - '
                                'falling back to %s' % (choice, missing, fallback))
                    ] + select_profile(context, fallback)
        return [LogInfo(msg='CAMERA: package %s not built and no fallback - '
                            'camera disabled' % missing)]

    return select_profile(context, choice)


def select_profile(context, choice):
    profile = CAMERA_PROFILES[choice]
    calibration = 'file://' + os.path.join(
        get_package_share_directory('dexi_camera'), 'config', profile['calibration'])
    jpeg_quality = LaunchConfiguration('camera_jpeg_quality').perform(context)
    announce = LogInfo(msg='CAMERA: %s %dx%d %s' % (
        choice, profile['width'], profile['height'], profile['calibration']))

    if choice == 'csi':
        return [announce, Node(
            package='camera_ros',
            executable='camera_node',
            name='cam0',
            remappings=[('image_raw', '/cam0/image_raw'),
                        ('camera_info', '/cam0/camera_info')],
            parameters=[{
                'format': LaunchConfiguration('camera_format').perform(context),
                'width': profile['width'],
                'height': profile['height'],
                'jpeg_quality': int(jpeg_quality),
                'camera_info_url': calibration,
                'frame_id': 'camera',
                'camera_name': 'cam0',
            }],
        )]

    return [announce, IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_camera'), 'camera.launch.py')
        ]),
        launch_arguments={
            'camera_id': usb_video_index(),
            'camera_name': 'cam0',
            'camera_width': str(profile['width']),
            'camera_height': str(profile['height']),
            'camera_info_url': calibration,
            'jpeg_quality': jpeg_quality,
            'timer_interval': LaunchConfiguration('camera_timer_interval').perform(context),
        }.items(),
    )]


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
    ld.add_action(DeclareLaunchArgument('camera_type', default_value='auto', description='Camera selection: auto, usb or csi'))
    ld.add_action(DeclareLaunchArgument('camera_format', default_value='XRGB8888', description='libcamera pixel format (CSI only)'))
    ld.add_action(DeclareLaunchArgument('camera_width', default_value='1280', description='Camera capture width in pixels'))
    ld.add_action(DeclareLaunchArgument('camera_height', default_value='720', description='Camera capture height in pixels'))
    ld.add_action(DeclareLaunchArgument('camera_jpeg_quality', default_value='60', description='JPEG compression quality (0-100)'))
    ld.add_action(DeclareLaunchArgument('camera_timer_interval', default_value='0.033', description='Camera capture timer interval in seconds (1/fps)'))
    ld.add_action(DeclareLaunchArgument('yolo', default_value='false', description='Enable YOLO detection'))
    ld.add_action(DeclareLaunchArgument('color_detection', default_value='false', description='Enable HSV color detection (opt-in)'))

    apriltags = LaunchConfiguration('apriltags')
    servos = LaunchConfiguration('servos')
    gpio = LaunchConfiguration('gpio')
    offboard = LaunchConfiguration('offboard')
    keyboard_control = LaunchConfiguration('keyboard_control')
    rosbridge = LaunchConfiguration('rosbridge')
    camera = LaunchConfiguration('camera')
    camera_and_yolo = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('camera'), "' == 'true' and '", LaunchConfiguration('yolo'), "' == 'true'"]))
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
    
    # Camera. Resolution and calibration come from the detected profile,
    # so camera_width/camera_height are not forwarded here.
    ld.add_action(GroupAction([OpaqueFunction(function=select_camera)],
                              condition=IfCondition(camera)))
    
    # AprilTag rate limit. apriltag_node JPEG-decodes every frame it
    # receives, so cost scales with this rate, not detector.decimate.
    # 10 Hz matches DEXI-5 v1; measured ~29% of a core on Pi 5.
    image_throttle_apriltag_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_apriltag_node',
        arguments=['messages', '/cam0/image_raw/compressed', '10.0', '/cam0/image_raw/compressed_apriltag'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_apriltag_node)

    # AprilTag node - consumes the 10Hz throttled stream (see above).
    # tag.ids/sizes/frames are required for apriltag_ros to publish TF poses;
    # without them the node detects tags in 2D but downstream consumers
    # (apriltag_odometry, tag_hop, precision_landing) can't look up TFs.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect/compressed', '/cam0/image_raw/compressed_apriltag'),
            ('camera_info', '/cam0/camera_info'),
            ('detections', '/apriltag_detections')
        ],
        parameters=[{
            'image_transport': 'compressed',
            'family': '36h11',  # Standard AprilTag family
            'size': 0.1524,  # 6 in black square
            'detector.decimate': 4.0,  # Decimate input image 4x to keep CPU in budget on Pi 5
            'tag.ids': [0, 1, 2, 3, 4, 5, 6, 7, 8, 9],
            'tag.sizes': [0.1524] * 10,
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
        condition=camera_and_yolo
    )
    ld.add_action(image_throttle_raw_node)
    
    # YOLO throttle: 2 FPS for object detection
    image_throttle_yolo_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_yolo_node',
        arguments=['messages', '/cam0/image_raw/compressed', '2.0', '/cam0/image_raw/compressed_2hz_yolo'],
        condition=camera_and_yolo
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