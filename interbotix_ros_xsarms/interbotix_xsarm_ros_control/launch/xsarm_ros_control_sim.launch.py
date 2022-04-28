import os
import yaml
from ament_index_python.packages import get_package_share_directory as pkgpath

from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription,OpaqueFunction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,FindExecutable,LaunchConfiguration,PathJoinSubstitution,
)
import xacro


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
    load_gazebo_configs = LaunchConfiguration("load_gazebo_configs")
    model = LaunchConfiguration("model")

    xsarm_descriptions_prefix = get_package_share_directory(
        "interbotix_xsarm_descriptions"
    )

    urdf_path = PathJoinSubstitution([xsarm_descriptions_prefix, "urdf", robot_model])

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
            " ",
            "use_sim:=",
            "true",
            " "
        ]
    )
   
    robot_description = {"robot_description": model}
    
    ros2_controllers_path = os.path.join(
        get_package_share_directory("interbotix_xsarm_ros_control"),
        "config",
        dof.perform(context)+"dof_controllers_sim.yaml",
    )

 
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #namespace=robot_model.perform(context),
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Load controllers
    load_controllers = []
    for controller in [ 
        "arm_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    

    return [ros2_control_node]+load_controllers

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
        "use_world_frame", default_value="False", description="use_world_frame"
    ) 
    external_urdf_loc_arg = DeclareLaunchArgument(
        "external_urdf_loc", default_value="", description="external_urdf_loc"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="use_rviz"
    )
    mode_configs_arg = DeclareLaunchArgument(
        "mode_configs", default_value=load_yaml("interbotix_xsarm_moveit_config", "config/modes.yaml"), description="mode_configs"
    )

    dof_arg = DeclareLaunchArgument(
        "dof", default_value="6", description="dof"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_gripper_bar",
                default_value="true",
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
            mode_configs_arg,
            dof_arg,
            OpaqueFunction(function=launch_setup),
        ]
        
    )

