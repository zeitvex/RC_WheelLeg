from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        default_value='true',
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

    # Include odin_ros_driver launch
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('odin_ros_driver'),
                'launch',
                'odin1_ros2.launch.py'
            ])
        ),
        launch_arguments={'launch_rviz': 'false'}.items(),
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
        Node(
            package="sim2real_hw",
            executable="sim2real_hw_node",
            name="sim2real_hw_node",
            output="screen",
            parameters=[runtime_params],
        ),
        Node(
            package="sim2real_runtime",
            executable="sim2real_runtime_node",
            name="sim2real_runtime_node",
            output="screen",
            parameters=[runtime_params],
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
            parameters=[runtime_params],
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

