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

    description_pkg = 'gen3_lite_description'
    description_path = get_package_share_directory(description_pkg)

    xacro_file = os.path.join(
        description_path,
        'urdf',
        'gen3_lite.urdf.xacro'
    )

    controllers_yaml = os.path.join(
        description_path,
        'config',
        'multi_gen3_lite_controllers.yaml'
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
            'gz_args': f"{os.path.join(description_path, 'worlds', 'multi_table.sdf')} -r"
        }.items(),
    )

    # ==============================================================
    # ROBOT r1
    # ==============================================================
    robot_ns_1 = 'r1'
    prefix_1 = 'r1_'
    robot_name_1 = 'gen3_lite_r1'

    robot_description_1 = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix_1,
            'namespace': robot_ns_1,
            'simulation_controllers': controllers_yaml,
        }
    ).toxml()

    rsp_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_ns_1,
        parameters=[{
            'robot_description': robot_description_1,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    spawn_1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'table_world',
            '-string', robot_description_1,
            '-name', robot_name_1,
            '-x', '1.0',
            '-y', '1.0',
            '-z', '0.75'
        ],
        output='screen'
    )

    jsb_1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_1,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_ns_1}/controller_manager'
        ],
        output='screen'
    )

    pos_1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_1,
        arguments=[
            'position_controller',
            '--controller-manager', f'/{robot_ns_1}/controller_manager'
        ],
        output='screen'
    )

    vel_1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_1,
        arguments=[
            'velocity_controller',
            '--inactive',
            '--controller-manager', f'/{robot_ns_1}/controller_manager'
        ],
        output='screen'
    )

    eff_1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_1,
        arguments=[
            'effort_controller',
            '--inactive',
            '--controller-manager', f'/{robot_ns_1}/controller_manager'
        ],
        output='screen'
    )

    # ==============================================================
    # ROBOT r2
    # ==============================================================
    robot_ns_2 = 'r2'
    prefix_2 = 'r2_'
    robot_name_2 = 'gen3_lite_r2'

    robot_description_2 = xacro.process_file(
        xacro_file,
        mappings={
            'prefix': prefix_2,
            'namespace': robot_ns_2,
            'simulation_controllers': controllers_yaml,
        }
    ).toxml()

    rsp_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_ns_2,
        parameters=[{
            'robot_description': robot_description_2,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    spawn_2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'table_world',
            '-string', robot_description_2,
            '-name', robot_name_2,
            '-x', '1.0',
            '-y', '-1.0',
            '-z', '0.75'
        ],
        output='screen'
    )

    jsb_2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_2,
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', f'/{robot_ns_2}/controller_manager'
        ],
        output='screen'
    )

    pos_2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_2,
        arguments=[
            'position_controller',
            '--controller-manager', f'/{robot_ns_2}/controller_manager'
        ],
        output='screen'
    )

    vel_2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_2,
        arguments=[
            'velocity_controller',
            '--inactive',
            '--controller-manager', f'/{robot_ns_2}/controller_manager'
        ],
        output='screen'
    )

    eff_2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_ns_2,
        arguments=[
            'effort_controller',
            '--inactive',
            '--controller-manager', f'/{robot_ns_2}/controller_manager'
        ],
        output='screen'
    )

    # ------------------------------------------------
    # Launch sequence (EVENT-BASED)
    # ------------------------------------------------
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        gazebo,

        # r1
        rsp_1,
        spawn_1,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_1,
                on_exit=[jsb_1],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_1,
                on_exit=[pos_1],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=pos_1,
                on_exit=[vel_1],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=vel_1,
                on_exit=[eff_1],
            )
        ),

        # r2
        rsp_2,
        spawn_2,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_2,
                on_exit=[jsb_2],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=jsb_2,
                on_exit=[pos_2],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=pos_2,
                on_exit=[vel_2],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=vel_2,
                on_exit=[eff_2],
            )
        ),
    ])
