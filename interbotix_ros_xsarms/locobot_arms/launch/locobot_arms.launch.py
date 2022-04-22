from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

ROS_NAMESPACE = ""


def generate_launch_description():

    ld = LaunchDescription()

    moveit_config = MoveItConfigsBuilder("interbotix_xsarm").to_moveit_configs()

    print(moveit_config.robot_description_kinematics)
    ld.add_action(
        Node(
            namespace=ROS_NAMESPACE,
            package="locobot_arms",
            executable="locobot_arms_action_server",
            name="locobot_arms_action_server",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            output="screen",
            emulate_tty=True,
        )
    )

    return ld
