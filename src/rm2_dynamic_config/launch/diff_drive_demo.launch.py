from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rm2_dynamic_config"))
    robot_description = (package_share / "urdf" / "diff_drive_mock.urdf").read_text()
    controllers_file = package_share / "config" / "diff_drive_demo.controllers.yaml"
    schema_file = package_share / "config" / "diff_drive_controller.schema.yaml"

    start_gui = LaunchConfiguration("start_gui")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[str(controllers_file)],
        remappings=[("~/robot_description", "/robot_description")],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gui_node = Node(
        package="rm2_dynamic_config",
        executable="rm2_dynamic_config_gui",
        output="screen",
        condition=IfCondition(start_gui),
        additional_env={
            "RM2_DYNAMIC_CONFIG_SCHEMA": str(schema_file),
            "QT_QPA_PLATFORM": "offscreen",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gui", default_value="true"),
            robot_state_publisher,
            controller_manager,
            joint_state_broadcaster,
            diff_drive_controller,
            gui_node,
        ]
    )
