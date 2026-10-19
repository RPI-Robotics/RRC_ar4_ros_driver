import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

MOVEIT_CONFIG = "annin_ar4_moveit_config"


def load_yaml(package_name, file_name, substitutions=None):
    path = os.path.join(get_package_share_directory(package_name), file_name)
    with open(path, "r", encoding="utf-8") as file:
        text = file.read()
    for key, value in (substitutions or {}).items():
        text = text.replace(key, value)
    return yaml.safe_load(text)


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    include_gripper = LaunchConfiguration("include_gripper")
    ar_model = LaunchConfiguration("ar_model")
    tf_prefix = LaunchConfiguration("tf_prefix")
    tf_prefix_value = tf_prefix.perform(context)

    robot_description = {
        "robot_description":
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([
                FindPackageShare("annin_ar4_description"), "urdf",
                "ar.urdf.xacro"
            ]),
            " ar_model:=", ar_model,
            " tf_prefix:=", tf_prefix,
            " include_gripper:=", include_gripper,
        ])
    }

    robot_description_semantic = {
        "robot_description_semantic":
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution(
                [FindPackageShare(MOVEIT_CONFIG), "srdf", "ar.srdf.xacro"]),
            " name:=", ar_model,
            " tf_prefix:=", tf_prefix,
            " include_gripper:=", include_gripper,
        ])
    }

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(MOVEIT_CONFIG, "config/kinematics.yaml")
    }

    # joint_limits.yaml and controllers.yaml are scoped to move_group, so pull out
    # their parameters for this node
    joint_limits = load_yaml(MOVEIT_CONFIG, "config/joint_limits.yaml",
                             {"$(var tf_prefix)": tf_prefix_value}
                             )["/**/move_group"]["ros__parameters"]

    # MoveItPy (MoveItCpp) reads pipelines from planning_pipelines.pipeline_names,
    # unlike move_group which takes a plain list
    planning_pipeline_config = {
        "planning_pipelines": {
            "pipeline_names": ["ompl", "pilz"]
        },
        "ompl": load_yaml(MOVEIT_CONFIG, "config/ompl_planning.yaml"),
        "pilz": load_yaml(MOVEIT_CONFIG, "config/pilz_planning.yaml"),
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "planning_time": 5.0,
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0,
        },
    }

    planning_scene_monitor_options = {
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic":
            "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic":
            "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic":
            "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        }
    }

    controllers = load_yaml(MOVEIT_CONFIG, "config/controllers.yaml",
                            {"$(var tf_prefix)": tf_prefix_value})
    moveit_controllers = {
        "moveit_controller_manager":
        "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        **controllers["/**/move_group"]["ros__parameters"],
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    demo_node = Node(
        package="rrc_ar4_pick_place",
        executable="demo.py",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            planning_scene_monitor_options,
            moveit_controllers,
            trajectory_execution,
            {
                "use_sim_time": use_sim_time
            },
        ],
    )
    return [demo_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use simulation time (set true when running with Gazebo)",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix for AR4 tf_tree",
        ),
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            choices=["True", "False"],
            description="Include the servo gripper",
        ),
        DeclareLaunchArgument(
            "ar_model",
            default_value="mk5",
            choices=["mk1", "mk2", "mk3", "mk4", "mk5"],
            description="Model of AR4",
        ),
    ]
    return LaunchDescription(declared_arguments +
                             [OpaqueFunction(function=launch_setup)])
