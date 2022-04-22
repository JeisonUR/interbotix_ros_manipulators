import os
import yaml
from ament_index_python.packages import get_package_share_directory as pkgpath

from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command,FindExecutable,LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription,OpaqueFunction,OpaqueFunction,ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration("dof")
    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")
    base_link_frame = LaunchConfiguration("base_link_frame")
    show_ar_tag = LaunchConfiguration("show_ar_tag")
    show_gripper_bar = LaunchConfiguration("show_gripper_bar")
    show_gripper_fingers = LaunchConfiguration("show_gripper_fingers")
    use_world_frame = LaunchConfiguration("use_world_frame")
    external_urdf_loc = LaunchConfiguration("external_urdf_loc")
    model = LaunchConfiguration("model")

    xsarm_descriptions_prefix = get_package_share_directory(
        "interbotix_xsarm_descriptions"
    )
    
    urdf_path = PathJoinSubstitution([xsarm_descriptions_prefix, "urdf", robot_model])
    print(urdf_path.)
    model = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
            ".urdf.xacro" " ",
            "robot_name:=",
            robot_name,
            " ",
            "base_link_frame:=",
            base_link_frame,
            " ",
            "show_ar_tag:=",
            show_ar_tag,
            " ",
            "show_gripper_bar:=",
            show_gripper_bar,
            " ",
            "show_gripper_fingers:=",
            show_gripper_fingers,
            " ",
            "use_world_frame:=",
            use_world_frame,
            " ",
            "external_urdf_loc:=",
            external_urdf_loc,
            " "
        ]
    )

    robot_description = {"robot_description": model}

    robot_description_semantic_config = load_file(
        "interbotix_xsarm_moveit", f"config/srdf/{robot_model.perform(context)}.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "interbotix_xsarm_moveit", "config/kinematics.yaml"
    )

    robot_description_kinematics={"robot_description_kinematics": kinematics_yaml["interbotix_arm"]}
    print(robot_description)
    actions_node= Node(
            package = 'locobot_arms',
            executable = 'locobot_arms_action_server',
            name = 'locobot_arms_action_server',
            parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics
            ],
            output = 'screen',
            emulate_tty = True,
        ) 

    return [actions_node]

def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        "robot_model", default_value="", description="robot_model"
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value=LaunchConfiguration("robot_model"), description="robot_name"
    )
    base_link_frame_arg = DeclareLaunchArgument(
        "base_link_frame", default_value="base_link", description="base_link_frame"
    )
    show_ar_tag_arg = DeclareLaunchArgument(
        "show_ar_tag", default_value="False", description="show_ar_tag"
    )
    use_world_frame_arg = DeclareLaunchArgument(
        "use_world_frame", default_value="True", description="use_world_frame"
    ) 
    external_urdf_loc_arg = DeclareLaunchArgument(
        "external_urdf_loc", default_value="", description="external_urdf_loc"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_moveit_rviz", default_value="False", description="use_moveit_rviz"
    )
    dof_arg = DeclareLaunchArgument(
        "dof", default_value="6", description="dof"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_gripper_bar",
                default_value="False",
                description=(
                    "if true, the gripper_bar link is included in the "
                    "'robot_description' parameter; if false, the gripper_bar and "
                    "finger links are not loaded. Set to false if you have a custom "
                    "gripper attachment"
                ),
            ),
            DeclareLaunchArgument(
                "show_gripper_fingers",
                default_value="true",
                description=(
                    "if true, the gripper fingers are included in the "
                    "'robot_description' parameter; if false, the gripper "
                    "finger links are not loaded. Set to false if you have "
                    "custom gripper fingers"
                ),
            ),
            robot_model_arg,
            robot_name_arg,
            base_link_frame_arg,
            show_ar_tag_arg,
            use_world_frame_arg,
            external_urdf_loc_arg,
            use_rviz_arg,
            dof_arg,
            OpaqueFunction(function=launch_setup),
        ]
        
    )
    