from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ar_model_config = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")
    isaac_host = LaunchConfiguration("isaac_host")
    isaac_port = LaunchConfiguration("isaac_port")
    use_sim_time = {"use_sim_time": True}

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_isaac"), "urdf", "ar_isaac.urdf.xacro"
        ]),
        " ",
        "ar_model:=",
        ar_model_config,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "isaac_host:=",
        isaac_host,
        " ",
        "isaac_port:=",
        isaac_port,
    ])
    robot_description = {"robot_description": robot_description_content}

    joint_controllers_cfg = PathJoinSubstitution(
        [FindPackageShare("annin_ar4_driver"), "config", "controllers.yaml"])
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(joint_controllers_cfg, allow_substs=True),
            {
                "tf_prefix": tf_prefix
            },
            use_sim_time,
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time],
    )

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "-c",
                "/controller_manager",
                "--controller-manager-timeout",
                "60",
            ],
        )

    return LaunchDescription([
        DeclareLaunchArgument("ar_model",
                              default_value="mk5",
                              choices=["mk1", "mk2", "mk3", "mk4", "mk5"],
                              description="Model of AR4"),
        DeclareLaunchArgument("tf_prefix",
                              default_value="",
                              description="Prefix for AR4 tf_tree"),
        DeclareLaunchArgument(
            "isaac_host",
            default_value="host.docker.internal",
            description="Host running Isaac Sim with ar4_isaac_bridge.py"),
        DeclareLaunchArgument("isaac_port",
                              default_value="50555",
                              description="TCP port of ar4_isaac_bridge.py"),
        robot_state_publisher_node,
        controller_manager_node,
        spawner("joint_state_broadcaster"),
        spawner("joint_trajectory_controller"),
        spawner("gripper_controller"),
    ])
