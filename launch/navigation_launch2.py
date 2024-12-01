import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    #pkg_share = '/home/bojan/ros2_lp/src/lp_nav'
    pkg_share = get_package_share_directory('lp_nav')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    map_yaml_file = os.path.join(pkg_share, 'maps', 'map.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Define nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
        #parameters=[{'robot_description': '/home/bojan/ros2_lp/src/lp_nav/urdf/robodt.urdf'}],

    )

    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    nav2_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{'use_sim_time': True}, {'yaml_filename': nav2_params_file}]    
    )


    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{'use_sim_time': True}, {'yaml_filename': nav2_params_file}] 
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom' ]
    )

    static_transform_publisher_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    fake_odometry_node =Node(
        package='lp_nav',
        executable='fake_odometry_node.py',
        name='fake_odometry_node',
        output='screen'
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': False},
                {'autostart': True},
                #{'node_names': ['map_server', 'planner_server', 'controller_server']}]
                {'node_names': ['map_server']}]

    )

    
    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        joint_state_publisher_node,
        static_transform_publisher_node,
        #static_transform_publisher_odom_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        fake_odometry_node,
        nav2_map_server_node,
        lifecycle_manager_node,
        #planner_server_node,
        #controller_server_node,
        #costmap_server_node,
    ])


 