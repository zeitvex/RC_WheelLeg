import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    my_share_dir = get_package_share_directory('sim2real_nav2')
    
    # Declare launch configuration variables
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(my_share_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    params_file = LaunchConfiguration('params_file')
    
    # Define Nav2 lifecycle nodes to run
    lifecycle_nodes = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
                       'global_costmap', 'local_costmap', 'amcl']
    
    # Controller server node
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Planner server node
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Behavior server node (called recovery_server in Galactic, behavior_server in Humble)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )
    
    # BT Navigator node
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )
    
    # Global costmap node
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[params_file]
    )
    
    # Local costmap node
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[params_file]
    )
    
    # AMCL node (Adaptive Monte Carlo Localization), now receives /scan from pointcloud_to_laserscan
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )
    
    # PointCloud2 to LaserScan converter (AMCL needs LaserScan, LiDAR publishes PointCloud2)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/odin1/cloud_slam'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.05,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,  # ~0.5 degrees
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1
        }]
    )
    
    # Lifecycle manager node to transition Nav2 nodes to ACTIVE state
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Set stdout line buffering
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))
    
    # Add actions
    ld.add_action(params_file_arg)
    ld.add_action(controller_server_node)
    ld.add_action(planner_server_node)
    ld.add_action(behavior_server_node)
    ld.add_action(bt_navigator_node)
    ld.add_action(global_costmap_node)
    ld.add_action(local_costmap_node)
    ld.add_action(amcl_node)
    ld.add_action(pointcloud_to_laserscan_node)
    ld.add_action(lifecycle_manager_node)
    
    return ld
