import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_name = "fishbot"
    pkg_share = get_package_share_directory('fishbot_description')

    # Paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'world', 'custom_room.world')

    # Launch argument for model
    declare_model_path = launch.actions.DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Absolute path to robot urdf/xacro file'
    )

    # robot_description parameter (Xacro → URDF)
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Include Gazebo launch
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': default_world_path, 'verbose': 'true'}.items()
    )

    # Spawn robot into Gazebo
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_name],
        output='screen'
    )

    # Controller loaders
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_joint_state_broadcaster'],
        output='screen'
    )
    load_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_diff_drive_controller'],
        output='screen'
    )
    load_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_effort_controller'],
        output='screen'
    )
    load_omni_controller = launch.actions.ExecuteProcess( # ros2 control load_controller --set-state active omni_wheel_controller
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'omni_wheel_controller'],
        output='screen'
    )

    # Launch description
    return launch.LaunchDescription([
        declare_model_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,

        # Load joint_state controller after spawn
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller]
            )
        ),

        # Load diff_drive controller after joint_state controller
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller]
            )
        ),

        # Load effort controller after diff_drive
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=load_diff_drive_controller,
                on_exit=[load_effort_controller]
            )
        ),

        # Load omni controller after effort_drive
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=load_effort_controller,
                on_exit=[load_omni_controller]
            )
        ),
    ])
