import launch
import launch_ros
import pathlib 
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import launch_ros.parameter_descriptions

def generate_launch_description():
    # fetch default urdf path
    urdf_package_path = get_package_share_directory('boat_description')
    urdf_path = Path(urdf_package_path) / 'urdf' / 'first_robot.urdf'
    default_rviz_config_path = Path(urdf_package_path) / 'config' /'display_robot_model.rviz'
    # generate param for urdf for future manipulation
    action_declare_arg_model_path = launch.actions.DeclareLaunchArgument(
        name='model',
        default_value=str(urdf_path),
        description='urdf file path'
    )

    substitutions_command_result = launch.substitutions.Command([
        'xacro ', launch.substitutions.LaunchConfiguration('model')
    ])

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(
        substitutions_command_result, value_type=str
    )

    # load urdf params
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_value}]
    )

    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(default_rviz_config_path)]
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])
