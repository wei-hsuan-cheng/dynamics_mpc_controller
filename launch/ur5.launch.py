import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mpc_share = FindPackageShare("dynamics_mpc_controller")
    mpc_share_dir = get_package_share_directory("dynamics_mpc_controller")
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    initial_pose_default = os.path.join(mpc_share_dir, "config", "ur5", "initial_pose.yaml")
    rviz_default = os.path.join(mpc_share_dir, "config", "rviz", "manipulator.rviz")
    lib_folder_default = os.path.join(package_root, "auto_generated", "ur5")

    urdf_default = PathJoinSubstitution([mpc_share, "description", "ur5", "urdf", "ur5.urdf"])
    controllers_default = PathJoinSubstitution([mpc_share, "config", "ur5", "inverse_dynamics_mpc_controller.yaml"])
    arm_control_xacro = PathJoinSubstitution([mpc_share, "description", "ur5", "urdf", "ur5.ros2_control.xacro"])

    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rvizconfig", default_value=rviz_default),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_mujoco_sim", default_value="true"),
        DeclareLaunchArgument("mujoco_headless", default_value="false"),
        DeclareLaunchArgument("mujoco_wait_to_start", default_value="true"),
        DeclareLaunchArgument("mujoco_real_time_factor", default_value="1.0"),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default),
        DeclareLaunchArgument(
            "libFolder",
            default_value=lib_folder_default,
            description="Writable folder for generated or cached CppAD libraries",
        ),
        DeclareLaunchArgument("mpcFreq", default_value="50", description="MPC update frequency (should be integer)"),
        DeclareLaunchArgument("mrtFreq", default_value="1000", description="MRT update frequency (should be integer)"),
        DeclareLaunchArgument("controllersFile", default_value=controllers_default),
        DeclareLaunchArgument("ros2ControlCommandInterface", default_value="effort"),
        DeclareLaunchArgument("targetTrajectoriesTopic", default_value="/mpc_targets"),
        DeclareLaunchArgument("estimatedEeWrenchTopic", default_value="/estimated_ee_wrench"),
        DeclareLaunchArgument("mpcObservationTopic", default_value="/inverse_dynamics_mpc_observation"),
        DeclareLaunchArgument("optimizedStateTrajectoryVisualization", default_value="true"),
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default),
        DeclareLaunchArgument("mujocoModelFile", default_value="scene_open_door.xml"),
    ]

    use_mujoco_sim = LaunchConfiguration("use_mujoco_sim")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim_time = ParameterValue(use_mujoco_sim, value_type=bool)
    mujoco_model_path = PathJoinSubstitution(
        [mpc_share, "description", "ur5", "mujoco", LaunchConfiguration("mujocoModelFile")]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            arm_control_xacro,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "use_mujoco_sim:=",
            use_mujoco_sim,
            " ",
            "initial_pose_file:=",
            LaunchConfiguration("initialPoseFile"),
            " ",
            "ros2_control_command_interface:=",
            LaunchConfiguration("ros2ControlCommandInterface"),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ParameterFile(LaunchConfiguration("controllersFile"), allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
        condition=UnlessCondition(use_mujoco_sim),
    )

    mujoco_ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        output="screen",
        parameters=[
            robot_description,
            ParameterFile(LaunchConfiguration("controllersFile"), allow_substs=True),
            {"mujoco_model_path": mujoco_model_path},
            {"mujoco_headless": ParameterValue(LaunchConfiguration("mujoco_headless"), value_type=bool)},
            {
                "mujoco_wait_to_start": ParameterValue(
                    LaunchConfiguration("mujoco_wait_to_start"), value_type=bool
                )
            },
            {
                "mujoco_real_time_factor": ParameterValue(
                    LaunchConfiguration("mujoco_real_time_factor"), value_type=float
                )
            },
            {"gt_enabled": False},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_mujoco_sim),
    )

    controller_sequence_script = PathJoinSubstitution(
        [FindPackageShare("dynamics_mpc_controller"), "launch", "controller_sequence.py"]
    )
    controller_sequence = ExecuteProcess(
        cmd=[
            "python3",
            controller_sequence_script,
            "--controller-manager",
            "/controller_manager",
            "--robot-description-topic",
            "/robot_description",
            "--timeout",
            "120",
        ],
        output="screen",
        condition=UnlessCondition(use_mujoco_sim),
    )
    controller_sequence_mujoco = ExecuteProcess(
        cmd=[
            "python3",
            controller_sequence_script,
            "--controller-manager",
            "/controller_manager",
            "--robot-description-topic",
            "/robot_description",
            "--timeout",
            "120",
            "--use-mujoco-sim",
            "--use-sim-time",
        ],
        output="screen",
        condition=IfCondition(use_mujoco_sim),
    )

    optimized_state_trajectory_visualization = Node(
        package="dynamics_mpc_controller",
        executable="optimized_state_trajectory_visualization_node",
        name="optimized_state_trajectory_visualization",
        output="screen",
        parameters=[
            ParameterFile(LaunchConfiguration("controllersFile"), allow_substs=True),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration("optimizedStateTrajectoryVisualization")),
    )

    visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([mpc_share, "launch", "visualization", "visualize.launch.py"])
        ),
        launch_arguments={
            "urdfFile": LaunchConfiguration("urdfFile"),
            "rviz": LaunchConfiguration("rviz"),
            "rvizconfig": LaunchConfiguration("rvizconfig"),
            "use_sim_time": use_mujoco_sim,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            mujoco_ros2_control_node,
            controller_sequence,
            controller_sequence_mujoco,
            optimized_state_trajectory_visualization,
            visualize_launch,
        ]
    )
