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


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        "robot_model", default_value="", description="robot_model"
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value=LaunchConfiguration("robot_model"), description="robot_name"
    )
    base_link_frame_arg = DeclareLaunchArgument(
        "base_link_frame", default_value="base_arm_link", description="base_link_frame"
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
    external_srdf_loc_arg = DeclareLaunchArgument(
        "external_srdf_loc", default_value="", description="external_srdf_loc"
    )
    use_moveit_rviz_arg = DeclareLaunchArgument(
        "use_moveit_rviz", default_value="True", description="use_moveit_rviz"
    )
    rviz_frame_arg = DeclareLaunchArgument(
        "rviz_frame", default_value="world", description="rviz_frame"
    )
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo", default_value="False", description="use_gazebo"
    )
    use_actual_arg = DeclareLaunchArgument(
        "use_actual", default_value="False", description="use_actual"
    )
    use_fake_arg = DeclareLaunchArgument(
        "use_fake", default_value="False", description="use_fake"
    )
    dof_arg = DeclareLaunchArgument(
        "dof", default_value="6", description="dof"
    )
    
    mode_configs = load_yaml("interbotix_xsarm_moveit_config", "config/modes.yaml")

    """
    world_name = load_yaml("interbotix_xsarm_gazebo", "worlds/xsarm_gazebo.world")

    gazebo_launch=IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                pkgpath("interbotix_xsarm_gazebo") + "/launch/xsarm_gazebo.launch.py"
            ),
            launch_arguments={
                "robot_model"               : LaunchConfiguration("robot_model"               ),
                "robot_name"                : LaunchConfiguration("robot_name"                ),
                "base_link_frame"           : LaunchConfiguration("base_link_frame"           ),
                "show_ar_tag"               : LaunchConfiguration("show_ar_tag"               ),
                "use_world_frame"           : LaunchConfiguration("use_world_frame"           ),
                "external_urdf_loc"         : LaunchConfiguration("external_urdf_loc"         ),
                "world_name"                : world_name,
                "use_trajectory_controllers": "True"
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_gazebo")),
        )"""

    actual_launch=IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                pkgpath("interbotix_xsarm_ros_control") + "/launch/xsarm_ros_control.launch.py"
            ),
            launch_arguments={
               "robot_model"                       : LaunchConfiguration("robot_model"),
               "robot_name"                        : LaunchConfiguration("robot_name"),
               "base_link_frame"                   : LaunchConfiguration("base_link_frame"),
               "show_ar_tag"                       : LaunchConfiguration("show_ar_tag"),
               "use_world_frame"                   : LaunchConfiguration("use_world_frame"),
               "external_urdf_loc"                 : LaunchConfiguration("external_urdf_loc"),
               "use_rviz"                          :"false",
               "mode_configs"                      : mode_configs,
               "dof"                               : LaunchConfiguration("dof"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_actual")),
        )

    fake_launch=IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                pkgpath("interbotix_xsarm_ros_control") + "/launch/xsarm_ros_control_sim.launch.py"
            ),
            launch_arguments={
               "robot_model"                       : LaunchConfiguration("robot_model"),
               "robot_name"                        : LaunchConfiguration("robot_name"),
               "base_link_frame"                   : LaunchConfiguration("base_link_frame"),
               "show_ar_tag"                       : LaunchConfiguration("show_ar_tag"),
               "use_world_frame"                   : LaunchConfiguration("use_world_frame"),
               "external_urdf_loc"                 : LaunchConfiguration("external_urdf_loc"),
               "use_rviz"                          :"false",
               "mode_configs"                      : mode_configs,
               "dof"                               : LaunchConfiguration("dof"),
               "use_sim"                           : "true",
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_fake")),
        )

    move_launch_sim=IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                pkgpath("interbotix_xsarm_moveit_config") + "/launch/move_group_sim.launch.py"
            ),
            launch_arguments={
               "robot_model"                       : LaunchConfiguration("robot_model"),
               "robot_name"                        : LaunchConfiguration("robot_name"),
               "base_link_frame"                   : LaunchConfiguration("base_link_frame"),
               "show_ar_tag"                       : LaunchConfiguration("show_ar_tag"),
               "use_world_frame"                   : LaunchConfiguration("use_world_frame"),
               "external_urdf_loc"                 : LaunchConfiguration("external_urdf_loc"),
               "use_moveit_rviz"                   : LaunchConfiguration("use_moveit_rviz"),
               "dof"                               : LaunchConfiguration("dof"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_fake")),
        )
    
    move_launch=IncludeLaunchDescription(
            launch_description_sources.PythonLaunchDescriptionSource(
                pkgpath("interbotix_xsarm_moveit_config") + "/launch/move_group.launch.py"
            ),
            launch_arguments={
               "robot_model"                       : LaunchConfiguration("robot_model"),
               "robot_name"                        : LaunchConfiguration("robot_name"),
               "base_link_frame"                   : LaunchConfiguration("base_link_frame"),
               "show_ar_tag"                       : LaunchConfiguration("show_ar_tag"),
               "use_world_frame"                   : LaunchConfiguration("use_world_frame"),
               "external_urdf_loc"                 : LaunchConfiguration("external_urdf_loc"),
               "use_moveit_rviz"                   : LaunchConfiguration("use_moveit_rviz"),
               "dof"                               : LaunchConfiguration("dof"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_actual")),
        )

    return LaunchDescription(
        [
            robot_model_arg,
            robot_name_arg,
            base_link_frame_arg,
            show_ar_tag_arg,
            use_world_frame_arg,
            external_urdf_loc_arg,
            external_srdf_loc_arg,
            use_moveit_rviz_arg,
            rviz_frame_arg,
            use_gazebo_arg,
            use_actual_arg,
            use_fake_arg,
            dof_arg,

            #gazebo_launch,
            actual_launch,
            fake_launch,
            move_launch,
            move_launch_sim
        ]
    )

