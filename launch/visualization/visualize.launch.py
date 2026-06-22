import launch

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ocs2_share = get_package_share_directory("dynamics_mpc_controller")
    rviz_default = f"{ocs2_share}/config/rviz/mobile_manipulator.rviz"
    urdf_file = LaunchConfiguration("urdfFile")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    actions = [
        launch.actions.DeclareLaunchArgument(
            name="urdfFile",
        ),
        launch.actions.DeclareLaunchArgument(
            name="test",
            default_value="false",
        ),
        launch.actions.DeclareLaunchArgument(
            name="rviz",
            default_value="true",
        ),
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=rviz_default,
        ),
        launch.actions.DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            arguments=[urdf_file],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    actions.extend([
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            arguments=[urdf_file],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("test")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mobile_manipulator",
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=["-d", LaunchConfiguration("rvizconfig")],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])

    return launch.LaunchDescription(actions)


if __name__ == "__main__":
    generate_launch_description()
