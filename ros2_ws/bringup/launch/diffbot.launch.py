# Copyright 2020 ros2_control Development Team
# SPDX-License-Identifier: Apache-2.0

import launch_ros.descriptions
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


LOCALIZATION_MODE = "localization"
MAPPING_MODE = "mapping"
TRUE_VALUES = {"1", "true", "yes", "on"}

MAPPING_VX_MAX = 0.35     # m/s
MAPPING_LIN_ACCEL = 0.4   # m/s^2
MAPPING_LIN_DECEL = -0.5  # m/s^2


def _launch_bool(value):
    return str(value).strip().lower() in TRUE_VALUES


def _expanded_path(path):
    return os.path.abspath(os.path.expanduser(os.path.expandvars(path)))


def _rtabmap_mode(context):
    mode = LaunchConfiguration("rtabmap_mode").perform(context).strip().lower()
    if mode not in (LOCALIZATION_MODE, MAPPING_MODE):
        raise RuntimeError(
            f"rtabmap_mode must be '{LOCALIZATION_MODE}' or '{MAPPING_MODE}', got '{mode}'")
    return mode


def _rtabmap_database_path(context, mode):
    return _expanded_path(LaunchConfiguration("rtabmap_database_path").perform(context))


def _validate_rtabmap_configuration(context, *args, **kwargs):
    mode = _rtabmap_mode(context)
    selected_db = _rtabmap_database_path(context, mode)
    delete_db = _launch_bool(LaunchConfiguration("rtabmap_delete_db_on_start").perform(context))

    if mode == LOCALIZATION_MODE:
        if delete_db:
            raise RuntimeError("rtabmap_delete_db_on_start is only allowed in mapping mode")
        if not os.path.isfile(selected_db):
            raise RuntimeError(f"RTAB-Map localization database not found: {selected_db}")
    else:
        parent = os.path.dirname(selected_db)
        if parent:
            os.makedirs(parent, exist_ok=True)

    return []


def _create_rtabmap_nodes(
    context,
    manage_lidar_standby,
    standby_scan_topic,
    rtabmap_parameters,
    rtabmap_remappings
):
    mode = _rtabmap_mode(context)
    selected_db = _rtabmap_database_path(context, mode)
    delete_db = _launch_bool(LaunchConfiguration("rtabmap_delete_db_on_start").perform(context))

    parameters = [dict(rtabmap_parameters[0], **{
        'database_path': selected_db,
        'Mem/IncrementalMemory': 'false' if mode == LOCALIZATION_MODE else 'true',
        'Mem/InitWMWithAllNodes': 'true' if mode == LOCALIZATION_MODE else 'false',
    })]
    arguments = ['-d'] if delete_db else []
    managed_rtabmap_remappings = rtabmap_remappings + [('scan', standby_scan_topic)]

    return [
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            condition=UnlessCondition(manage_lidar_standby),
            parameters=parameters,
            remappings=rtabmap_remappings,
            arguments=arguments),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            condition=IfCondition(manage_lidar_standby),
            parameters=parameters,
            remappings=managed_rtabmap_remappings,
            arguments=arguments),
    ]


def _write_mapping_nav2_params(base_params):
    with open(base_params) as handle:
        params = yaml.safe_load(handle)

    params['controller_server']['ros__parameters']['FollowPath']['vx_max'] = MAPPING_VX_MAX

    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][0] = MAPPING_VX_MAX
    smoother['max_accel'][0] = MAPPING_LIN_ACCEL
    smoother['max_decel'][0] = MAPPING_LIN_DECEL

    out_path = os.path.join(tempfile.gettempdir(), 'diffbot_nav2_params_mapping.yaml')
    with open(out_path, 'w') as handle:
        yaml.safe_dump(params, handle, default_flow_style=False, sort_keys=False)
    return out_path


def _create_nav2(context, *args, **kwargs):
    diffbot_share = get_package_share_directory('diffbot')
    base_params = os.path.join(diffbot_share, 'config', 'nav2_params.yaml')

    if _rtabmap_mode(context) == MAPPING_MODE:
        params_file = _write_mapping_nav2_params(base_params)
    else:
        params_file = base_params

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(diffbot_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': params_file,
            }.items(),
        )
    ]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "manage_lidar_standby",
            default_value="false",
            description="Pause RTAB-Map/ICP and stop the RPLidar motor when Nav2 has no active goals.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "standby_scan_topic",
            default_value="/diffbot/standby_scan",
            description="Managed scan topic used by RTAB-Map and ICP when lidar standby is enabled.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rtabmap_mode",
            default_value=LOCALIZATION_MODE,
            description="RTAB-Map operating mode: localization or mapping.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rtabmap_database_path",
            default_value="~/.ros/diffbot/maps/rtabmap.db",
            description="RTAB-Map database loaded by localization mode and written by mapping mode.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rtabmap_delete_db_on_start",
            default_value="false",
            description="Delete the selected RTAB-Map database on start. Mapping mode only.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_semantic_export",
            default_value="true",
            description="Publish a throttled, compressed, map-frame-posed RGB-D bundle on /semantic/* "
                        "for the offboard semantic map backend (DualMap/OneMap). Off by default.",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    manage_lidar_standby = LaunchConfiguration("manage_lidar_standby")
    standby_scan_topic = LaunchConfiguration("standby_scan_topic")

    rplidar_pkg = get_package_share_directory('rplidar_ros')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffbot"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg, "launch", "rplidar_a1_launch.py")
        ),
        launch_arguments={
            "serial_baudrate": "115200",
            "frame_id": "scan",
            "serial_port": "/dev/rplidar"
        }.items()
    )

    ekf_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "ekf.yaml",
        ]
    )

    imu_filter_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "imu_filter.yaml",
        ]
    )
    external_imu_filter_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "external_imu_filter.yaml",
        ]
    )
    realsense_params = PathJoinSubstitution(
        [
            FindPackageShare("diffbot"),
            "config",
            "realsense_params.yaml",
        ]
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=UnlessCondition(use_mock_hardware),
        parameters=[ekf_params],
        remappings=[
            ("odometry/filtered", "/odom"),
        ],
    )

    nav2 = OpaqueFunction(function=_create_nav2)

    rtabmap_parameters = [{
        'frame_id': 'base_footprint',
        'subscribe_depth': True,
        'subscribe_scan': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'wait_imu_to_init': True,
        'Reg/Force3DoF': 'true',
        'Reg/Strategy': '1',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityPathMaxNeighbors': '10',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityOdomGuess': 'true',
        'RGBD/ProximityPathFilteringRadius': '2.0',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '0',
        'Optimizer/Robust': 'true',
        'Optimizer/Strategy': '1',
        'Icp/MaxTranslation': '0.5',
        'Vis/MinInliers': '12',
        'Kp/DetectorStrategy': '8',
        'Vis/FeatureType': '8',
        'Rtabmap/DetectionRate': '5'
    }]

    rtabmap_remappings = [
        ('imu', '/imu/data_body'),
        ('odom', '/odom'),
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw')]

    icp_odometry_parameters = [{
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'publish_tf': False,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'wait_imu_to_init': False,
        'always_check_imu_tf': True,
        'Reg/Force3DoF': 'true',
        'Odom/ResetCountdown': '5',
        'Icp/MaxCorrespondenceDistance': '0.3',
        'Icp/MaxTranslation': '0.5'
    }]

    icp_odometry_remappings = [
        ('imu', '/imu/data_body'),
        ('odom', '/rtabmap/icp_odom'),
        ('odom_info', '/rtabmap/icp_odom_info'),
        ('scan', '/scan')]

    managed_icp_odometry_remappings = [
        ('imu', '/imu/data_body'),
        ('odom', '/rtabmap/icp_odom'),
        ('odom_info', '/rtabmap/icp_odom_info'),
        ('scan', standby_scan_topic)]

    depth_module = SetParameter(name='depth_module.emitter_enabled', value=1)

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
        launch_arguments={
            'initial_reset': 'true',
            'enable_color': 'true',
            'align_depth.enable': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'enable_motion': 'false',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'rgb_camera.color_profile': '640x360x15',
            'depth_module.depth_profile': '424x240x15',
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'config_file': realsense_params
        }.items(),
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter",
        output="screen",
        parameters=[imu_filter_params],
        remappings=[
            ("imu/data_raw", "/camera/camera/imu"),
            ("imu/data", "/imu/data"),
        ],
    )

    imu_transform = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="imu_data_transformer",
        output="screen",
        parameters=[{
            "target_frame": "camera_imu_frame",
        }],
        remappings=[
            ("imu_in", "/imu/data"),
            ("imu_out", "/imu/data_body"),
        ],
    )

    external_imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="external_imu_filter",
        output="screen",
        parameters=[external_imu_filter_params],
        remappings=[
            ("imu/data_raw", "/imu/data_raw"),
            ("imu/mag", "/imu/mag"),
            ("imu/data", "/imu/external/data"),
        ],
    )

    external_imu_transform = Node(
        package="imu_transformer",
        executable="imu_transformer_node",
        name="external_imu_transformer",
        output="screen",
        parameters=[{
            "target_frame": "base_footprint",
        }],
        remappings=[
            ("imu_in", "/imu/external/data"),
            ("imu_out", "/imu/external/data_body"),
            ("mag_in", "/imu/mag"),
            ("mag_out", "/imu/external/mag_body"),
        ],
    )

    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        condition=UnlessCondition(manage_lidar_standby),
        parameters=icp_odometry_parameters,
        remappings=icp_odometry_remappings,
    )

    managed_icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        condition=IfCondition(manage_lidar_standby),
        parameters=icp_odometry_parameters,
        remappings=managed_icp_odometry_remappings,
    )

    icp_odom_reweighter = Node(
        package='diffbot',
        executable='diffbot_icp_odom_reweighter',
        name='diffbot_icp_odom_reweighter',
        output='screen',
        parameters=[{
            'input_odom_topic': '/rtabmap/icp_odom',
            'imu_topic': '/imu/data_body',
            'output_odom_topic': '/rtabmap/icp_odom_reweighted',
            'yaw_disagreement_gain': 2.0,
            'disagreement_deadband': 0.1,
            'min_yaw_variance': 0.0,
            'max_yaw_variance': 1.0,
            'reweight_twist': True,
            'gyro_timeout_sec': 0.5,
        }],
    )

    rtabmap_nodes = OpaqueFunction(
        function=_create_rtabmap_nodes,
        args=[manage_lidar_standby, standby_scan_topic, rtabmap_parameters, rtabmap_remappings])

    lidar_standby_manager = Node(
        package='diffbot',
        executable='diffbot_lidar_standby_manager',
        name='diffbot_lidar_standby_manager',
        output='screen',
        condition=IfCondition(manage_lidar_standby),
        parameters=[{
            'idle_timeout_sec': 10.0,
            'initial_idle_timeout_sec': 25.0,
            'scan_timeout_sec': 3.0,
            'start_motor_service': '/start_motor',
            'stop_motor_service': '/stop_motor',
            'pause_rtabmap_service': '/rtabmap/pause',
            'resume_rtabmap_service': '/rtabmap/resume',
            'pause_odom_service': '/pause_odom',
            'resume_odom_service': '/resume_odom',
            'scan_topic': '/scan',
            'managed_scan_topic': standby_scan_topic,
            'nav_action_status_topics': [
                '/navigate_to_pose/_action/status',
                '/navigate_through_poses/_action/status',
                '/spin/_action/status',
            ],
            'publish_standby_scan_heartbeat': True,
            'standby_scan_heartbeat_hz': 1.0,
        }],
    )

    semantic_export = Node(
        package='diffbot',
        executable='diffbot_semantic_export',
        name='diffbot_semantic_export',
        output='screen',
        condition=IfCondition(LaunchConfiguration("enable_semantic_export")),
        parameters=[{
            'color_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'map_frame': 'map',
            'camera_frame': 'camera_color_optical_frame',
            'target_rate_hz': 4.0,
            'tf_timeout_sec': 0.1,
        }],
    )

    rosbridge_server_pkg = get_package_share_directory('rosbridge_server')
    rosbridge_server_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_server_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        depth_module,
        realsense,
        imu_filter,
        imu_transform,
        external_imu_filter,
        external_imu_transform,
        ekf,
        rplidar,
        icp_odometry,
        managed_icp_odometry,
        icp_odom_reweighter,
        rtabmap_nodes,
        lidar_standby_manager,
        semantic_export,
        nav2,
        rosbridge_server_launch
    ]

    validate_rtabmap_configuration = OpaqueFunction(function=_validate_rtabmap_configuration)

    return LaunchDescription(declared_arguments + [validate_rtabmap_configuration] + nodes)
