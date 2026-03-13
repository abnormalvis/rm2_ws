from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dynamic_config_share = Path(get_package_share_directory("rm2_dynamic_config"))
    gimbal_share = Path(get_package_share_directory("rm2_gimbal_controllers"))

    default_schema_file = (
        dynamic_config_share / "config" / "test_gimbal_controller.schema.yaml"
    )
    default_import_yaml = (
        dynamic_config_share / "config" / "test_gimbal_controller.snapshot.yaml"
    )
    default_controller_yaml = (
        gimbal_share / "config" / "test_gimbal_controller.standard.yaml"
    )

    start_gui = LaunchConfiguration("start_gui")
    gui_platform = LaunchConfiguration("gui_platform")
    target_node = LaunchConfiguration("target_node")
    schema_file = LaunchConfiguration("schema_file")
    startup_delay_sec = LaunchConfiguration("startup_delay_sec")
    import_yaml = LaunchConfiguration("import_yaml")
    export_yaml = LaunchConfiguration("export_yaml")
    use_non_native_dialogs = LaunchConfiguration("use_non_native_dialogs")
    probe_target_params = LaunchConfiguration("probe_target_params")
    probe_joint_states = LaunchConfiguration("probe_joint_states")
    probe_debug_topics = LaunchConfiguration("probe_debug_topics")

    gui_node = Node(
        package="rm2_dynamic_config",
        executable="rm2_dynamic_config_gui",
        output="screen",
        condition=IfCondition(start_gui),
        additional_env={
            "RM2_DYNAMIC_CONFIG_SCHEMA": schema_file,
            "RM2_DYNAMIC_CONFIG_TARGET_NODE": target_node,
            "RM2_DYNAMIC_CONFIG_IMPORT_PATH": import_yaml,
            "RM2_DYNAMIC_CONFIG_EXPORT_PATH": export_yaml,
            "RM2_DYNAMIC_CONFIG_USE_NON_NATIVE_DIALOGS": use_non_native_dialogs,
            "QT_QPA_PLATFORM": gui_platform,
        },
    )

    param_probe = ExecuteProcess(
        cmd=["ros2", "param", "list", target_node],
        output="screen",
        condition=IfCondition(probe_target_params),
    )
    joint_state_probe = ExecuteProcess(
        cmd=["ros2", "topic", "info", "/joint_states"],
        output="screen",
        condition=IfCondition(probe_joint_states),
    )
    yaw_debug_probe = ExecuteProcess(
        cmd=["ros2", "topic", "info", [target_node, "/debug/yaw_target"]],
        output="screen",
        condition=IfCondition(probe_debug_topics),
    )
    pitch_debug_probe = ExecuteProcess(
        cmd=["ros2", "topic", "info", [target_node, "/debug/pitch_target"]],
        output="screen",
        condition=IfCondition(probe_debug_topics),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_gui", default_value="true"),
            DeclareLaunchArgument("gui_platform", default_value="offscreen"),
            DeclareLaunchArgument("target_node", default_value="/test_gimbal_controller"),
            DeclareLaunchArgument("schema_file", default_value=str(default_schema_file)),
            DeclareLaunchArgument("startup_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("import_yaml", default_value=""),
            DeclareLaunchArgument("export_yaml", default_value=""),
            DeclareLaunchArgument("use_non_native_dialogs", default_value="0"),
            DeclareLaunchArgument("probe_target_params", default_value="true"),
            DeclareLaunchArgument("probe_joint_states", default_value="true"),
            DeclareLaunchArgument("probe_debug_topics", default_value="true"),
            LogInfo(
                msg=[
                    "Start rm2_ecat first. Example controller yaml: ",
                    str(default_controller_yaml),
                ]
            ),
            LogInfo(
                msg=[
                    "Default GUI schema: ",
                    str(default_schema_file),
                    " | Default import template: ",
                    str(default_import_yaml),
                ]
            ),
            TimerAction(period=startup_delay_sec, actions=[gui_node]),
            TimerAction(
                period=startup_delay_sec,
                actions=[param_probe, joint_state_probe, yaw_debug_probe, pitch_debug_probe],
            ),
        ]
    )
