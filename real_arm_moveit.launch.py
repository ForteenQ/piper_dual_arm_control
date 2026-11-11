import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 从 piper_with_gripper_moveit 构造实机 MoveIt 配置 (强制 use_sim_time: False)
    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_with_gripper_moveit")
        .robot_description(file_path="config/piper.urdf.xacro")
        .robot_description_semantic(file_path="config/piper.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 1. robot_state_publisher: 读取 URDF 与实机 200Hz /joint_states 广播实机 TF
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": False}
        ]
    )

    # 2. move_group 核心服务节点
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": ":0"}
    )

    return LaunchDescription([
        rsp_node,
        move_group_node
    ])
