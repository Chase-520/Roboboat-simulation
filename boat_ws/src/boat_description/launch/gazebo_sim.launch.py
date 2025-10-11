import launch
import launch_ros
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # fetch pkg path
    urdf_package_path = get_package_share_directory('boat_description')
    urdf_path = Path(urdf_package_path) / 'urdf' / 'boat' / 'boat.urdf.xacro'
    default_rviz_config_path = Path(urdf_package_path) / 'config' / 'display_robot_model.rviz'
    default_gazebo_world = Path(urdf_package_path) / 'world' / 'first_world.world'

    # declare model argument
    action_declare_arg_model_path = DeclareLaunchArgument(
        name='model',
        default_value=str(urdf_path),
        description='urdf file path'
    )

    # xacro command
    substitutions_command_result = Command([
        'xacro ', LaunchConfiguration('model')
    ])

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type=str
    )

    # robot_state_publisher node
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    # include Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            str(Path(get_package_share_directory('gazebo_ros')) / 'launch' / 'gazebo.launch.py')
        ]),
        launch_arguments=[('world', str(default_gazebo_world)), ('verbose', 'true')]
    )

    # rviz node (optional)
    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', str(default_rviz_config_path)]
    # )

    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity','Barcopolo']
    )
    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        gazebo_launch,
        # rviz_node
        action_spawn_entity
    ])
