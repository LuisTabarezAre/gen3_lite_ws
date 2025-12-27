import os
import xacro
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------
    # Launch arguments
    # ------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_name = 'gen3_lite_r1'
    robot_ns = 'r1'
    prefix = 'r1_'

    description_pkg = 'gen3_lite_description'
    description_path = get_package_share_directory(description_pkg)

    controllers_yaml = os.path.join(
        description_path,
        'config',
        'gen3_lite_controllers.yaml'
    )

    # ------------------------------------------------
    # Gazebo resource paths
    # ------------------------------------------------
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path += ':' + str(Path(description_path).parent)
    gz_resource_path += ':' + os.path.join(description_path, 'worlds')
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path

    # ------------------------------------------------
    # Gazebo
    # ------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f"{os.path.join(description_path, 'worlds', 'table.sdf')} -r"
        }.items(),
    )

    # ------------------------------------------------
    # Robot description (XACRO → URDF)
    # ------------------------------------------------
    xacro_file = os.path.join(
        description_path,
        'urdf',
        'gen3_lite.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix,
            'namespace': robot_ns,
            'simulation_controllers': controllers_yaml,
        }
    ).toxml()

    # ------------------------------------------------
    # robot_state_publisher
    # ------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_ns,
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # ------------------------------------------------
    # Spawn robot in Gazebo
    # ------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'table_world',
            '-string', robot_description,
            '-name', robot_name,
            '-z', '0.75'
        ],
    )

    # ------------------------------------------------
    # ros2_control spawners (NAMESPACED)
    # ------------------------------------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_ns}/controller_manager',
        ],
        output='screen',
    )

    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns,
        arguments=[
            'position_controller',
            '--controller-manager', f'/{robot_ns}/controller_manager',
        ],
        output='screen',
    )

    velocity_controller = Node(
    package='controller_manager',
    executable='spawner',
    namespace=robot_ns,
    arguments=[
        'velocity_controller',
        '--inactive',
        '--controller-manager', f'/{robot_ns}/controller_manager',
    ],
    output='screen',
    )

    effort_controller = Node(
    package='controller_manager',
    executable='spawner',
    namespace=robot_ns,
    arguments=[
        'effort_controller',
        '--inactive',
        '--controller-manager', f'/{robot_ns}/controller_manager',
    ],
    output='screen',
    )

    # ------------------------------------------------
    # RViz
    # ------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-d',
            os.path.join(description_path, 'rviz', 'rviz_config.rviz')
        ],
    )

    # ------------------------------------------------
    # Launch sequence
    # ------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        gazebo,
        robot_state_publisher,
        spawn_robot,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[position_controller],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=position_controller,
                on_exit=[velocity_controller],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=velocity_controller,
                on_exit=[effort_controller],
            )
        ),
        rviz,
    ])
