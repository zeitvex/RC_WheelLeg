from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    runtime_params = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare("sim2real_bringup"),
            "config",
            "runtime.yaml",
        ]),
        allow_substs=True,
    )

    # Declare launch configurations
    launch_driver_arg = DeclareLaunchArgument(
        'launch_driver',
        default_value='true',
        description='Whether to launch the odin_ros_driver sensor node'
    )
    
    launch_nav2_arg = DeclareLaunchArgument(
        'launch_nav2',
        default_value='false',
        description='Whether to launch the Nav2 navigation stack'
    )

    launch_remote_arg = DeclareLaunchArgument(
        'launch_remote',
        default_value='true',
        description='Whether to launch the SBUS UART remote control node'
    )

    launch_web_bridge_arg = DeclareLaunchArgument(
        'launch_web_bridge',
        default_value='true',
        description='Whether to launch the Windows/Nano UDP web debug bridge'
    )

    launch_simple_nav_arg = DeclareLaunchArgument(
        'launch_simple_nav',
        default_value='true',
        description='Whether to launch the simple waypoint navigation node'
    )

    localization_mode_arg = DeclareLaunchArgument(
        'localization_mode',
        default_value='relocal',
        description='Localization profile: odom uses bridge fallback; relocal waits for Odin map/odom TF'
    )

    odin_config_file_arg = DeclareLaunchArgument(
        'odin_config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('odin_ros_driver'),
            'config',
            'control_command_relocal.yaml',
        ]),
        description='Odin control config YAML for the selected localization profile'
    )

    event_log_dir_arg = DeclareLaunchArgument(
        'event_log_dir',
        default_value=PythonExpression([
            "'logs_v2_web/run_' + __import__('datetime').datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')[:-3]"
        ]),
        description='Per-run event log directory'
    )

    # Include odin_ros_driver launch
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odin_ros_driver'),
                'launch',
                'odin1_ros2.launch.py'
            ])
        ),
        launch_arguments={
            'launch_rviz': 'false',
            'config_file': LaunchConfiguration('odin_config_file'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_driver'))
    )

    # Include sim2real_nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sim2real_nav2'),
                'launch',
                'nav2.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('launch_nav2'))
    )

    return LaunchDescription([
        launch_driver_arg,
        launch_nav2_arg,
        launch_remote_arg,
        launch_web_bridge_arg,
        launch_simple_nav_arg,
        localization_mode_arg,
        odin_config_file_arg,
        event_log_dir_arg,
        Node(
            package="sim2real_hw",
            executable="sim2real_hw_node",
            name="sim2real_hw_node",
            output="screen",
            parameters=[runtime_params, {"event_log_dir": LaunchConfiguration("event_log_dir")}],
        ),
        Node(
            package="sim2real_runtime",
            executable="sim2real_runtime_node",
            name="sim2real_runtime_node",
            output="screen",
            parameters=[runtime_params, {"event_log_dir": LaunchConfiguration("event_log_dir")}],
        ),
        Node(
            package="sim2real_runtime",
            executable="cmd_mux_node.py",
            name="sim2real_cmd_mux_node",
            output="screen",
            parameters=[runtime_params],
        ),
        Node(
            package="sim2real_runtime",
            executable="web_udp_bridge_node.py",
            name="sim2real_web_udp_bridge_node",
            output="screen",
            parameters=[runtime_params, {"localization_mode": LaunchConfiguration("localization_mode")}],
            condition=IfCondition(LaunchConfiguration('launch_web_bridge')),
        ),
        Node(
            package="sim2real_runtime",
            executable="remote_uart_node.py",
            name="sim2real_remote_uart_node",
            output="screen",
            parameters=[runtime_params],
            condition=IfCondition(LaunchConfiguration('launch_remote')),
        ),
        Node(
            package="sim2real_runtime",
            executable="simple_nav_node.py",
            name="sim2real_simple_nav_node",
            output="screen",
            parameters=[runtime_params],
            condition=IfCondition(LaunchConfiguration('launch_simple_nav')),
        ),
        Node(
            package="sim2real_runtime",
            executable="odom_relay_node",
            name="odom_relay_node",
            output="screen",
            parameters=[{
                "odom_input_topic": "/odin1/odometry",
                "odom_output_topic": "/odom",
                "base_frame": "base_link",
                "publish_tf": True,
            }],
            condition=IfCondition(LaunchConfiguration('launch_driver')),
        ),
        driver_launch,
        nav2_launch,
    ])
